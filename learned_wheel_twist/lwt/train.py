#!/usr/bin/env python
# lwt/train.py —— [venv torch] 训练纯轮速 TwistTCN。MSE 预热 → NLL。+ TensorBoard。
# 用法: ~/rtabmap_ws/.venv-py312/bin/python lwt/train.py [--with-imu] [--epochs N] [--out runs/lwt_wheel]
import argparse, sys, glob, yaml, numpy as np, torch
from pathlib import Path
from torch.utils.tensorboard import SummaryWriter
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from lwt.dataset import TwistDataset, stratified_split
from lwt.model import TwistTCN
from lwt.losses import weighted_mse, gaussian_nll, batch_bias_penalty

def episode_list(cache_root):
    eps = []
    for p in sorted(glob.glob(str(Path(cache_root).expanduser()/"*"/"*.npz"))):
        if Path(p).name == "test.npz": continue
        d = np.load(p)
        k = float(d["kappa_theory"]) if "kappa_theory" in d else 0.0
        eps.append((p, k))
    return eps

def main():
    cfg_path = Path(__file__).resolve().parent.parent / "config" / "default.yaml"
    cfg = yaml.safe_load(cfg_path.read_text())
    ap = argparse.ArgumentParser()
    ap.add_argument("--with-imu", action="store_true")
    ap.add_argument("--imu-wz-prior", action="store_true",
                    help="phase-2b: 残差先验 wz 用窗末 imu_yawrate 取代 LS wz(需 --with-imu)")
    ap.add_argument("--epochs", type=int, default=cfg["train"]["epochs"])
    ap.add_argument("--out", default="learned_wheel_twist/runs/lwt_wheel")
    a = ap.parse_args()
    imu_wz_prior = a.imu_wz_prior and a.with_imu
    cfg["model"]["imu_wz_prior"] = imu_wz_prior   # 存入 ckpt cfg,供 eval/eval_real 复现
    if a.imu_wz_prior and not a.with_imu:
        print("[警告] --imu-wz-prior 需配 --with-imu,wheel-only 下忽略。")
    dev = cfg["train"]["device"] if torch.cuda.is_available() else "cpu"
    eps = episode_list(cfg["data"]["cache_root"])
    tr, va, te = stratified_split(eps, (cfg["split"]["train"], cfg["split"]["val"], cfg["split"]["test"]), cfg["split"]["seed"])
    print(f"episodes train/val/test = {len(tr)}/{len(va)}/{len(te)}")
    W = cfg["data"]["window"]; ka = cfg["kappa_aug"]
    in_ch = 9 if a.with_imu else 8   # phase-2: 8 wheel + 1 imu_yawrate
    dtr = TwistDataset([p for p, _ in tr], W, augment=True, with_imu=a.with_imu,
                       steer_bias_max_deg=ka["steer_bias_max_deg"], speed_scale_std=ka["speed_scale_std"],
                       imu_wz_prior=imu_wz_prior)
    norm = dtr.fit_norm()
    dva = TwistDataset([p for p, _ in va], W, augment=False, with_imu=a.with_imu, norm=norm,
                       imu_wz_prior=imu_wz_prior)
    ystd = dtr.Y.std(0) + 1e-6; w = torch.tensor(1.0/ystd, dtype=torch.float32, device=dev)
    Ltr = torch.utils.data.DataLoader(dtr, batch_size=cfg["train"]["batch"], shuffle=True, num_workers=4)
    Lva = torch.utils.data.DataLoader(dva, batch_size=512)
    m = TwistTCN(in_ch, cfg["model"]["tcn_channels"], cfg["model"]["tcn_layers"], cfg["model"]["kernel"], cfg["model"]["dropout"]).to(dev)
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
