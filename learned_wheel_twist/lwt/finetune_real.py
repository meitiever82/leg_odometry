#!/usr/bin/env python
# lwt/finetune_real.py —— 阶段六 Step C:真机激光监督 fine-tune(留一法 LOO)。
#
# 从 sim 预训练 lwt_pre_wn 出发,在 2 个真机 bag 上用**激光轨迹损失**(lidar_traj_loss)微调,
# 留出第 3 个 bag 做 held-out 验证。低 LR + 少 epoch + L2-to-pretrained 锚,防只 2-bag 过拟合。
# 真机 npz 无逐帧 gt twist → 段只含特征 + 窗末时戳;监督全部来自激光 TUM(老师)。
#
# 用法:
#   ~/.venv-py312/bin/python lwt/finetune_real.py --pretrain runs/lwt_pre_wn.pt \
#       --fold A --out runs/lwt_ft_foldA
# 折定义见 FOLDS;bag→(npz, tum) 见 REAL_BAGS。
import argparse, sys, numpy as np, torch
from pathlib import Path
from torch.utils.tensorboard import SummaryWriter
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from lwt.model import TwistTCN
from lwt.dataset import build_features
from lwt.lidar_loss import lidar_traj_loss
from lwt.kinematics_torch import integrate_twist_torch

HOME = str(Path.home())
REAL_BAGS = {
    "0522":    (f"{HOME}/Documents/Datasets/w2_real_cache/0522.npz",
                f"{HOME}/Documents/Datasets/w2/rosbag2_2026_05_22-13_58_40/traj_imu.txt"),
    "0605_37": (f"{HOME}/Documents/Datasets/w2_real_cache/0605_37.npz",
                f"{HOME}/Documents/Datasets/w2/rosbag2_2026_06_05-17_37_48/traj_imu.txt"),
    "0605_42": (f"{HOME}/Documents/Datasets/w2_real_cache/0605_42.npz",
                f"{HOME}/Documents/Datasets/w2/rosbag2_2026_06_05-17_42_57/kiss_path_tum.txt"),
}
FOLDS = {
    "A": {"train": ["0522", "0605_37"], "test": "0605_42"},
    "B": {"train": ["0522", "0605_42"], "test": "0605_37"},
    "C": {"train": ["0605_37", "0605_42"], "test": "0522"},
}


def make_real_segments(npz_path, norm, window, dd, wza, segment_sec, stride_sec, hz):
    """真机 npz → 连续 L 秒分段(无 gt twist)。每段:
      X (S,C,window) 标准化后的逐窗输入(C×T 排布,模型可直接 roll)
      t (S,)         窗末时戳(float64,绝对 epoch;用于激光插值 + 积分)
      yr (S,)        窗末 imu_yawrate(未标准化;wz_anchor 先验 = [0,0,yr];free-wz 忽略)
    监督来自激光,故不需要 G。"""
    d = np.load(npz_path)
    F = build_features(d, with_imu=(not dd and not wza and False),
                       data_driven=dd, wz_anchor=wza).astype(np.float32)   # (N,C)
    t = np.asarray(d["t_wheel"], float).reshape(-1)
    keys = d.files if hasattr(d, "files") else d
    yawrate = (np.asarray(d["imu_yawrate"]).reshape(-1).astype(np.float32)
               if "imu_yawrate" in keys else np.zeros(len(F), np.float32))  # 去偏陀螺 base yaw-rate
    N = len(F)
    S = int(round(segment_sec * hz))
    stride = max(1, int(round(stride_sec * hz)))
    segs = []
    last_out0 = N - S
    for out0 in range(window - 1, last_out0 + 1, stride):
        idx = np.arange(out0, out0 + S)
        Xseg = np.stack([F[j - window + 1:j + 1] for j in idx], 0)         # (S,window,C)
        if norm is not None:
            Xseg = (Xseg - norm[0]) / norm[1]
        x = np.transpose(Xseg, (0, 2, 1)).copy()                          # (S,C,window)
        segs.append((torch.from_numpy(x).float(), t[idx].copy(), yawrate[idx].copy()))
    return segs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pretrain", required=True)
    ap.add_argument("--fold", required=True, choices=list(FOLDS))
    ap.add_argument("--out", required=True)
    ap.add_argument("--epochs", type=int, default=None)
    ap.add_argument("--lr", type=float, default=None)
    a = ap.parse_args()

    dev = "cuda" if torch.cuda.is_available() else "cpu"
    ck = torch.load(a.pretrain, weights_only=False)
    cfg = ck["cfg"]; norm = ck["norm"]; W = cfg["data"]["window"]; hz = cfg["data"]["wheel_hz"]
    dd = cfg["model"].get("data_driven", False); wza = cfg["model"].get("wz_anchor", False)
    prior = cfg["model"].get("prior", "none")
    # finetune 超参用**当前** config(ckpt 的 cfg 快照可能早于 disp_weight 等新键)。
    import yaml as _yaml
    _live = _yaml.safe_load((Path(__file__).resolve().parent.parent / "config" / "default.yaml").read_text())
    ft = _live["finetune"]
    epochs = a.epochs if a.epochs is not None else ft["epochs"]
    lr = a.lr if a.lr is not None else ft["lr"]

    m = TwistTCN(ck["in_ch"], cfg["model"]["tcn_channels"], cfg["model"]["tcn_layers"],
                 cfg["model"]["kernel"], cfg["model"]["dropout"], prior=prior).to(dev)
    m.load_state_dict(ck["model"])
    # L2-to-pretrained 锚:保存预训练权重快照(detach,不更新)。
    pre_params = {n: p.detach().clone() for n, p in m.named_parameters()}

    fold = FOLDS[a.fold]
    print(f"[阶段六 Step C fold {a.fold}] 训练 bags={fold['train']} → held-out={fold['test']}")
    print(f"  pretrain={a.pretrain}  prior={prior} dd={dd} wza={wza}  LR={lr} epochs={epochs} "
          f"l2_anchor={ft['l2_anchor']} seg={ft['segment_sec']}s/stride{ft['stride_sec']}s")

    # 训练段:2 个 bag 的激光监督段。
    train_segs = []   # list of (x(S,C,W), wt(S,), yr(S,), tum(N,8), bag)
    for bag in fold["train"]:
        npz, tum_path = REAL_BAGS[bag]
        T = np.loadtxt(tum_path)
        segs = make_real_segments(npz, norm, W, dd, wza, ft["segment_sec"], ft["stride_sec"], hz)
        for x, wt, yr in segs:
            train_segs.append((x, wt, yr, T, bag))
    print(f"  train segments={len(train_segs)} (across {len(fold['train'])} bags)")

    opt = torch.optim.Adam(m.parameters(), lr=lr)
    out_path = Path(a.out); out_path.parent.mkdir(parents=True, exist_ok=True)
    writer = SummaryWriter(a.out + "_tb")
    strides = ft["sub_strides"]; yaw_w = ft["yaw_weight"]; l2w = ft["l2_anchor"]
    clip = ft["grad_clip"]; seg_bs = ft["seg_batch"]
    disp_w = ft.get("disp_weight", 2.0)   # 旧 ckpt cfg 无此键 → 默认 2.0
    print(f"  yaw_weight={yaw_w} disp_weight={disp_w} (scale 锚防 vx 塌缩)")
    rng = np.random.default_rng(0)
    idx = np.arange(len(train_segs))
    in_ch = ck["in_ch"]; ls_zero = torch.zeros(1, 3, device=dev)

    for ep in range(epochs):
        m.train(); rng.shuffle(idx)
        ltraj, lanchor = [], []
        for bstart in range(0, len(idx), seg_bs):
            bidx = idx[bstart:bstart + seg_bs]
            opt.zero_grad()
            traj_loss_acc = 0.0
            for j in bidx:
                x, wt, yr, T, _bag = train_segs[j]
                x = x.to(dev)                              # (S,C,W)
                S = x.shape[0]
                # wz_anchor:先验 = [0,0,imu_yawrate(窗末)],twist = 先验 + 残差,
                # 激光只教 vx/vy 头 + wz 残差(陀螺锚保留)。free-wz(prior='none')忽略先验。
                if wza:
                    lsp = torch.zeros(S, 3, device=dev)
                    lsp[:, 2] = torch.as_tensor(yr, device=dev)
                else:
                    lsp = ls_zero.expand(S, 3)
                tw, _ = m(x, lsp)                          # (S,3) twist
                wt_t = torch.as_tensor(wt, dtype=torch.float64, device=dev)
                tl = lidar_traj_loss(tw, wt_t, T, strides=strides, yaw_weight=yaw_w, disp_weight=disp_w)
                traj_loss_acc = traj_loss_acc + tl
            traj_loss_acc = traj_loss_acc / len(bidx)
            # L2-to-pretrained 锚(防 2-bag 过拟合,把权重拉回预训练结构)。
            anchor = sum(((p - pre_params[n]) ** 2).sum() for n, p in m.named_parameters())
            loss = traj_loss_acc + l2w * anchor
            loss.backward()
            torch.nn.utils.clip_grad_norm_(m.parameters(), clip)
            opt.step()
            ltraj.append(float(traj_loss_acc.item())); lanchor.append(float((l2w * anchor).item()))
        print(f"ep{ep} traj={np.mean(ltraj):.4f} anchor={np.mean(lanchor):.5f}")
        writer.add_scalar("loss/traj", float(np.mean(ltraj)), ep)
        writer.add_scalar("loss/anchor", float(np.mean(lanchor)), ep)
    # fine-tune 无 held-out 监督做模型选择(那会泄漏)→ 保存最终 epoch 权重(gentle FT 已限过拟合)。
    torch.save({"model": m.state_dict(), "norm": norm, "in_ch": in_ch, "cfg": cfg,
                "fold": a.fold, "fold_def": fold, "pretrain": a.pretrain}, a.out + ".pt")
    writer.close()
    print(f"saved -> {a.out}.pt  (tensorboard: {a.out}_tb)")


if __name__ == "__main__":
    main()
