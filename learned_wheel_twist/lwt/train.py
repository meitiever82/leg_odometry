#!/usr/bin/env python
# lwt/train.py —— [venv torch] 训练纯轮速 TwistTCN。MSE 预热 → NLL。+ TensorBoard。
# 用法: ~/rtabmap_ws/.venv-py312/bin/python lwt/train.py [--with-imu] [--epochs N] [--out runs/lwt_wheel]
import argparse, sys, glob, yaml, numpy as np, torch
from pathlib import Path
from torch.utils.tensorboard import SummaryWriter
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from lwt.dataset import TwistDataset, SegmentDataset, stratified_split
from lwt.model import TwistTCN
from lwt.losses import weighted_mse, gaussian_nll, batch_bias_penalty, trajectory_rpe_loss
from lwt.kinematics_torch import integrate_twist_torch
from lwt.kinematics import integrate_twist, ape_percent, rpe_segment

def episode_list(cache_root):
    eps = []
    for p in sorted(glob.glob(str(Path(cache_root).expanduser()/"*"/"*.npz"))):
        if Path(p).name == "test.npz": continue
        d = np.load(p)
        k = float(d["kappa_theory"]) if "kappa_theory" in d else 0.0
        eps.append((p, k))
    return eps

def _roll_segment(m, x_seg, ls_zero):
    """对一段逐输出帧 roll 模型(流式/窗式部署形态),返回 (pred_twists (S,3), logvar (S,3))。
    x_seg: (S, C, W);模型一次吃整批 S 个窗(等价逐帧调用,因 TCN 因果窗内独立)。"""
    S = x_seg.shape[0]
    ls = ls_zero.expand(S, 3)
    tw, lv = m(x_seg, ls)              # (S,3),(S,3)
    return tw, lv


def _sim_traj_metrics(m, seg_ds, dev, max_segs=40, rpe_seg_len=1.0):
    """验证集分段:逐段 roll 模型 → 积分 → numpy APE / RPE。

    注意:sim 段 10s 内位移中位仅 ~1.3m(机器人慢),故 RPE 用 1m 弧长(@10m 会全段无效返 0)。
    APE 用首点对齐位置 RMSE / 段路径长;短段路径短时 APE 数值会偏大但可用于相对比较。"""
    m.eval()
    apes, rpes, yawerr = [], [], []
    ls_zero = torch.zeros(1, 3, device=dev)
    with torch.no_grad():
        for i in range(min(len(seg_ds), max_segs)):
            x, t, g, _ = seg_ds[i]
            x = x.to(dev)
            tw, _ = _roll_segment(m, x, ls_zero)
            tw = tw.cpu().numpy(); tn = t.numpy()
            pr = integrate_twist(tn, tw)
            gt = integrate_twist(tn, g.numpy())
            apes.append(ape_percent(pr, gt[:, :3]))
            rpes.append(rpe_segment(pr, gt[:, :3], rpe_seg_len))
            yawerr.append(abs(float(pr[-1, 3] - gt[-1, 3])))   # 段末积分 yaw 绝对误差(rad)
    f = lambda a: float(np.mean(a)) if a else 0.0
    return f(apes), f(rpes), np.degrees(f(yawerr))


def train_traj(cfg, a):
    """approach-2 step1:端到端可微长程轨迹损失训练(纯 data-driven 14ch)。

    管线:连续 L 秒分段 → 每帧消费自身 W 窗 → roll 模型得逐帧 twist → 可微 SE2 积分
    → RPE 风格轨迹损失(短/长子段,长程 Δyaw 暴露 wz 直流偏置)。
    总损失 = traj + λ_pf·逐帧 NLL + bias_penalty。前 pf_warmup_epochs 仅逐帧 MSE 预热。
    模型保持流式/窗式(可逐窗部署),rollout 只是逐时刻应用。"""
    prior = "none"   # data-driven 变体:raw 轮速[8]+imu_g_base[3]+imu_a_base[3],无 LS/无陀螺锚
    cfg["model"]["prior"] = prior
    cfg["model"]["data_driven"] = True
    cfg["model"]["wz_anchor"] = False
    cfg["model"]["imu_wz_prior"] = False
    cfg["model"]["traj_loss"] = True
    cfg["model"]["wheel_noise"] = bool(getattr(a, "wheel_noise", False))
    tj = cfg["traj"]
    print(f"[approach-2 step1 traj-loss] data-driven 14ch,prior='none'。"
          f"segment_sec={tj['segment_sec']} stride_sec={tj['stride_sec']} "
          f"seg_batch={tj['seg_batch']} sub_strides={tj['sub_strides']} "
          f"yaw_w={tj['yaw_weight']} λ_pf={tj['lambda_pf']} pf_warmup={tj['pf_warmup_epochs']}")
    dev = cfg["train"]["device"] if torch.cuda.is_available() else "cpu"
    eps = episode_list(cfg["data"]["cache_root"])
    tr, va, te = stratified_split(eps, (cfg["split"]["train"], cfg["split"]["val"], cfg["split"]["test"]), cfg["split"]["seed"])
    print(f"episodes train/val/test = {len(tr)}/{len(va)}/{len(te)}")
    W = cfg["data"]["window"]; hz = cfg["data"]["wheel_hz"]; ka = cfg["kappa_aug"]
    in_ch = 14
    # approach-2 step2a: sim 保真增广(--realism)。同时作用于逐帧锚集 + 分段轨迹集(仅 SIM 训练输入)。
    realism = cfg.get("realism") if getattr(a, "realism", False) else None
    if realism:
        rb = realism
        print(f"[approach-2 step2a realism] κ-bulk(std={rb['kappa_bulk']['kappa_std']}) "
              f"gyro(σb={rb['gyro']['sigma_bias']},σn={rb['gyro']['sigma_noise']}) "
              f"bumps(rate={rb['bumps']['event_rate']}) accel(σ={rb['accel_noise']['sigma']})")
    # 阶段六 Step A:仅轮速通道高斯输入噪声(IMU 不动)。--wheel-noise 时启用,与 realism 正交。
    wheel_noise = cfg.get("pretrain", {}).get("wheel_noise_std") if getattr(a, "wheel_noise", False) else None
    if wheel_noise and wheel_noise.get("enable"):
        print(f"[阶段六 Step A wheel-noise] 仅轮速通道:speed_std={wheel_noise['speed_std']} m/s, "
              f"steer_std={wheel_noise['steer_std_deg']}° (IMU 通道不加噪)")
    else:
        wheel_noise = None
    # 逐帧训练集:用于 fit_norm + 逐帧 NLL 正则 + per-frame std 权重。复用既有 data_driven 路径。
    # 注意:norm 在 fit_norm() 里用 **未增广** 的 self.X 统计(增广在 __getitem__),故 norm 仍是干净 sim 分布。
    dtr_pf = TwistDataset([p for p, _ in tr], W, augment=True, with_imu=False,
                          steer_bias_max_deg=ka["steer_bias_max_deg"], speed_scale_std=ka["speed_scale_std"],
                          data_driven=True, realism=realism, wheel_noise=wheel_noise)
    norm = dtr_pf.fit_norm()
    ystd = dtr_pf.Y.std(0) + 1e-6; w = torch.tensor(1.0/ystd, dtype=torch.float32, device=dev)
    Lpf = torch.utils.data.DataLoader(dtr_pf, batch_size=cfg["train"]["batch"], shuffle=True, num_workers=4)
    # 分段训练集(轨迹损失)。norm 复用逐帧统计。
    seg_tr = SegmentDataset([p for p, _ in tr], W, tj["segment_sec"], hz, norm=norm,
                            data_driven=True, stride_sec=tj["stride_sec"], realism=realism,
                            wheel_noise=wheel_noise)
    # 验证段保持干净 sim(不增广)——val 是 in-distribution sim 门槛的口径基准。
    seg_va = SegmentDataset([p for p, _ in va], W, tj["segment_sec"], hz, norm=norm,
                            data_driven=True, stride_sec=tj["segment_sec"])
    print(f"train segments={len(seg_tr)} val segments={len(seg_va)} (S={int(round(tj['segment_sec']*hz))} frames/seg)")
    # 段以 list-of-(S,C,W) 返回(各段 S 相同);用 indices 自己组 batch。
    seg_idx = np.arange(len(seg_tr))
    m = TwistTCN(in_ch, cfg["model"]["tcn_channels"], cfg["model"]["tcn_layers"], cfg["model"]["kernel"],
                 cfg["model"]["dropout"], prior=prior).to(dev)
    opt = torch.optim.Adam(m.parameters(), lr=cfg["train"]["lr"])
    out_path = Path(a.out); out_path.parent.mkdir(parents=True, exist_ok=True)
    writer = SummaryWriter(a.out + "_tb")
    xs, _, lss = next(iter(Lpf))
    try: writer.add_graph(m, (xs.to(dev), lss.to(dev)))
    except Exception as e: print("add_graph skip:", e)
    bp_w = cfg["train"]["bias_penalty"]
    strides = tj["sub_strides"]; yaw_w = tj["yaw_weight"]; lam_pf = tj["lambda_pf"]
    pf_warm = tj["pf_warmup_epochs"]; seg_bs = tj["seg_batch"]
    lr_scale = tj.get("traj_lr_scale", 1.0)
    ls_zero = torch.zeros(1, 3, device=dev)
    rng = np.random.default_rng(0)
    best = 1e9
    for ep in range(a.epochs):
        m.train()
        tl_traj, tl_pf, tl_bias = [], [], []
        if ep == pf_warm:
            for g in opt.param_groups:        # 进入轨迹阶段:降 LR 稳 rollout 梯度。
                g["lr"] = cfg["train"]["lr"] * lr_scale
            print(f"  [traj phase] LR -> {cfg['train']['lr'] * lr_scale:.2e}")
        if ep < pf_warm:
            # 逐帧 MSE 预热(同 data-driven phase-3 起点),稳定 vx/vy 尺度后再上轨迹损失。
            for x, y, ls in Lpf:
                x, y, ls = x.to(dev), y.to(dev), ls.to(dev)
                tw, lv = m(x, ls)
                loss = weighted_mse(tw, y, w) + batch_bias_penalty(tw, y, bp_w, w)
                opt.zero_grad(); loss.backward(); opt.step(); tl_pf.append(loss.item())
            print(f"ep{ep} [pf-warmup] mean loss={np.mean(tl_pf):.4f}")
            writer.add_scalar("loss/per_frame", float(np.mean(tl_pf)), ep)
            writer.add_scalar("loss/traj", 0.0, ep)
            writer.add_scalar("loss/bias", 0.0, ep)
        else:
            rng.shuffle(seg_idx)
            pf_iter = iter(Lpf)
            for bstart in range(0, len(seg_idx), seg_bs):
                bidx = seg_idx[bstart:bstart + seg_bs]
                xb = torch.stack([seg_tr[j][0] for j in bidx]).to(dev)   # (B,S,C,W)
                tb = torch.stack([seg_tr[j][1] for j in bidx]).to(dev)   # (B,S) float64
                gb = torch.stack([seg_tr[j][2] for j in bidx]).to(dev)   # (B,S,3)
                B, S = xb.shape[0], xb.shape[1]
                # roll:把 (B,S) 个窗摊平成 (B*S,C,W) 一次前向(等价逐帧,TCN 窗内独立)。
                tw, lv = m(xb.reshape(B * S, in_ch, W), ls_zero.expand(B * S, 3))
                tw = tw.reshape(B, S, 3); lv = lv.reshape(B, S, 3)
                pred_traj = integrate_twist_torch(tb, tw.double())
                gt_traj = integrate_twist_torch(tb, gb.double())
                tloss = trajectory_rpe_loss(pred_traj, gt_traj, strides, yaw_w)
                # 逐帧 MSE 正则(锚 vx/vy/wz 尺度,防轨迹损失把 twist 推向退化解)。
                # 用 MSE 而非 NLL:NLL 的 logvar 项易塌缩成大负数、与轨迹损失尺度打架。
                pf = weighted_mse(tw.reshape(-1, 3), gb.reshape(-1, 3).float(), w)
                bias = batch_bias_penalty(tw.reshape(-1, 3), gb.reshape(-1, 3).float(), bp_w, w)
                loss = tloss + lam_pf * pf + bias
                opt.zero_grad(); loss.backward()
                torch.nn.utils.clip_grad_norm_(m.parameters(), 5.0)
                opt.step()
                tl_traj.append(float(tloss.item())); tl_pf.append(float(pf.item())); tl_bias.append(float(bias.item()))
            print(f"ep{ep} traj={np.mean(tl_traj):.4f} pf_nll={np.mean(tl_pf):.4f} bias={np.mean(tl_bias):.5f}")
            writer.add_scalar("loss/traj", float(np.mean(tl_traj)), ep)
            writer.add_scalar("loss/per_frame", float(np.mean(tl_pf)), ep)
            writer.add_scalar("loss/bias", float(np.mean(tl_bias)), ep)
        # 验证:分段积分 APE / RPE / 段末 yaw 漂移(让用户看到积分航向漂移被压住)。
        ape, rpe, yawd = _sim_traj_metrics(m, seg_va, dev)
        print(f"     val sim APE%={ape:.2f}  RPE@1m%={rpe:.2f}  seg-end |Δyaw|={yawd:.2f}°")
        writer.add_scalar("val/APE", ape, ep)
        writer.add_scalar("val/RPE10", rpe, ep)            # 名义 RPE(此处 1m 弧长,sim 段位移短)
        writer.add_scalar("val/yaw_drift_deg", yawd, ep)
        for name, p in m.named_parameters():
            writer.add_histogram(f"weight/{name}", p.detach().cpu(), ep)
            if p.grad is not None:
                writer.add_histogram(f"grad/{name}", p.grad.detach().cpu(), ep)
        # 模型选择:轨迹损失阶段用 (RPE + yaw 漂移) 选最优(APE 在短段上太噪);
        # 预热阶段加大数偏置,避免选到未上轨迹损失的中间态。
        sc = (rpe + yawd) if ep >= pf_warm else 1e8 + rpe + yawd
        if sc < best:
            best = sc
            torch.save({"model": m.state_dict(), "norm": norm, "in_ch": in_ch, "cfg": cfg,
                        "split": {"train": tr, "val": va, "test": te}}, a.out + ".pt")
    writer.close()
    print(f"BEST val (RPE+yawDrift) {best:.4f} -> {a.out}.pt  (tensorboard: {a.out}_tb)")


def main():
    cfg_path = Path(__file__).resolve().parent.parent / "config" / "default.yaml"
    cfg = yaml.safe_load(cfg_path.read_text())
    ap = argparse.ArgumentParser()
    ap.add_argument("--with-imu", action="store_true")
    ap.add_argument("--imu-wz-prior", action="store_true",
                    help="phase-2b: 残差先验 wz 用窗末 imu_yawrate 取代 LS wz(需 --with-imu)")
    ap.add_argument("--data-driven", action="store_true",
                    help="phase-3: 纯 data-driven,14ch(raw 轮速+base IMU),prior='none' 直接回归,无 LS 先验")
    ap.add_argument("--wz-anchor", action="store_true",
                    help="phase-4: 14ch(raw 轮速+base IMU),先验=[0,0,imu_yawrate],wz=去偏陀螺+残差,vx/vy 从 raw 学(无 LS)")
    ap.add_argument("--traj-loss", action="store_true",
                    help="approach-2 step1: 端到端可微长程轨迹损失(data-driven 14ch,prior='none')。"
                         "对连续 L 秒分段 roll 模型→积分→RPE 轨迹损失,使积分航向漂移可见。")
    ap.add_argument("--realism", action="store_true",
                    help="approach-2 step2a: sim 保真增广(κ-bulk + 真实陀螺噪声/残差偏置 + 合成颠簸 + 加速度噪声),"
                         "把真机域退化注入 SIM 训练输入闭合 sim→real gap(仅配 --traj-loss;标签不变)。")
    ap.add_argument("--wheel-noise", action="store_true",
                    help="阶段六 Step A: sim 预训练时仅对**轮速通道**(steer+speed)注入小高斯输入噪声(IMU 不动),"
                         "为真机激光 fine-tune 准备鲁棒的轮速→twist 结构(配 --traj-loss)。")
    ap.add_argument("--epochs", type=int, default=cfg["train"]["epochs"])
    ap.add_argument("--out", default="learned_wheel_twist/runs/lwt_wheel")
    a = ap.parse_args()
    if a.traj_loss:
        return train_traj(cfg, a)
    wz_anchor = a.wz_anchor and not a.data_driven   # data_driven 优先
    data_driven = a.data_driven
    with_imu = a.with_imu and not data_driven and not wz_anchor   # dd/wz_anchor 自带 base IMU
    imu_wz_prior = a.imu_wz_prior and with_imu
    prior = ("none" if data_driven else
             ("wz_anchor" if wz_anchor else ("imu_wz" if imu_wz_prior else "ls")))
    cfg["model"]["imu_wz_prior"] = imu_wz_prior   # 存入 ckpt cfg,供 eval/eval_real 复现
    cfg["model"]["prior"] = prior
    cfg["model"]["data_driven"] = data_driven
    cfg["model"]["wz_anchor"] = wz_anchor
    if a.imu_wz_prior and not with_imu:
        print("[警告] --imu-wz-prior 需配 --with-imu(且非 --data-driven/--wz-anchor),已忽略。")
    if data_driven:
        print("[phase-3 data-driven] 14ch raw 轮速+base IMU,prior='none' 直接回归,无 LS 先验。")
    if wz_anchor:
        print("[phase-4 wz-anchor] 14ch raw 轮速+base IMU,先验=[0,0,imu_yawrate],wz=陀螺+残差,vx/vy 从 raw 学。")
    dev = cfg["train"]["device"] if torch.cuda.is_available() else "cpu"
    eps = episode_list(cfg["data"]["cache_root"])
    tr, va, te = stratified_split(eps, (cfg["split"]["train"], cfg["split"]["val"], cfg["split"]["test"]), cfg["split"]["seed"])
    print(f"episodes train/val/test = {len(tr)}/{len(va)}/{len(te)}")
    W = cfg["data"]["window"]; ka = cfg["kappa_aug"]
    in_ch = 14 if (data_driven or wz_anchor) else (9 if with_imu else 8)   # phase-3/4: 8 wheel + 6 base IMU
    # 阶段七 Step 1:wz-anchor 预训练同样支持 --wheel-noise(仅轮速通道,IMU 不动);
    # 给真机激光 fine-tune 准备鲁棒的 vx/vy 轮速尺度,而陀螺保持可信 wz 锚源。与 κ-aug 正交。
    wheel_noise = cfg.get("pretrain", {}).get("wheel_noise_std") if getattr(a, "wheel_noise", False) else None
    if wheel_noise and wheel_noise.get("enable"):
        print(f"[阶段七 Step1 wheel-noise] 仅轮速通道:speed_std={wheel_noise['speed_std']} m/s, "
              f"steer_std={wheel_noise['steer_std_deg']}° (IMU 通道不加噪)")
    else:
        wheel_noise = None
    dtr = TwistDataset([p for p, _ in tr], W, augment=True, with_imu=with_imu,
                       steer_bias_max_deg=ka["steer_bias_max_deg"], speed_scale_std=ka["speed_scale_std"],
                       imu_wz_prior=imu_wz_prior, data_driven=data_driven, wz_anchor=wz_anchor,
                       wheel_noise=wheel_noise)
    norm = dtr.fit_norm()
    dva = TwistDataset([p for p, _ in va], W, augment=False, with_imu=with_imu, norm=norm,
                       imu_wz_prior=imu_wz_prior, data_driven=data_driven, wz_anchor=wz_anchor)
    ystd = dtr.Y.std(0) + 1e-6; w = torch.tensor(1.0/ystd, dtype=torch.float32, device=dev)
    Ltr = torch.utils.data.DataLoader(dtr, batch_size=cfg["train"]["batch"], shuffle=True, num_workers=4)
    Lva = torch.utils.data.DataLoader(dva, batch_size=512)
    # wz_anchor: zero_init_residual=True → 初始 vx=vy=0、wz=陀螺;残差从 0 学起。
    m = TwistTCN(in_ch, cfg["model"]["tcn_channels"], cfg["model"]["tcn_layers"], cfg["model"]["kernel"], cfg["model"]["dropout"],
                 prior=prior, zero_init_residual=wz_anchor).to(dev)
    opt = torch.optim.Adam(m.parameters(), lr=cfg["train"]["lr"])
    out_path = Path(a.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    writer = SummaryWriter(a.out + "_tb")
    xs, _, lss = next(iter(Ltr))
    try: writer.add_graph(m, (xs.to(dev), lss.to(dev)))
    except Exception as e: print("add_graph skip:", e)
    bp_w = cfg["train"]["bias_penalty"]
    best = 1e9
    for ep in range(a.epochs):
        m.train(); tl = []; bpl = []
        for x, y, ls in Ltr:
            x, y, ls = x.to(dev), y.to(dev), ls.to(dev)
            tw, lv = m(x, ls)
            base = weighted_mse(tw, y, w) if ep < cfg["train"]["mse_warmup_epochs"] else gaussian_nll(tw, y, lv, w)
            bp = batch_bias_penalty(tw, y, bp_w, w)
            loss = base + bp
            opt.zero_grad(); loss.backward(); opt.step(); tl.append(loss.item()); bpl.append(bp.item())
        m.eval(); ve = []
        with torch.no_grad():
            for x, y, ls in Lva:
                x, y, ls = x.to(dev), y.to(dev), ls.to(dev)
                tw, _ = m(x, ls); ve.append(((tw-y)**2).mean(0).cpu().numpy())
        vrmse = np.sqrt(np.mean(ve, 0))
        print(f"ep{ep} val twist RMSE vx={vrmse[0]:.4f} vy={vrmse[1]:.4f} wz={vrmse[2]:.4f}")
        writer.add_scalar("loss/train", float(np.mean(tl)), ep)
        writer.add_scalar("loss/bias_penalty", float(np.mean(bpl)), ep)
        for ax, nm in enumerate(["vx", "vy", "wz"]):
            writer.add_scalar(f"val_rmse/{nm}", float(vrmse[ax]), ep)
        for name, p in m.named_parameters():
            writer.add_histogram(f"weight/{name}", p.detach().cpu(), ep)
            if p.grad is not None:
                writer.add_histogram(f"grad/{name}", p.grad.detach().cpu(), ep)
        sc = vrmse.sum()
        if sc < best:
            best = sc
            torch.save({"model": m.state_dict(), "norm": norm, "in_ch": in_ch, "cfg": cfg,
                        "split": {"train": tr, "val": va, "test": te}}, a.out + ".pt")
    writer.close()
    print(f"BEST val sum RMSE {best:.4f} -> {a.out}.pt  (tensorboard: {a.out}_tb)")

if __name__ == "__main__":
    main()
