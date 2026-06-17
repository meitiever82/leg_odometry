#!/usr/bin/env python
# lwt/eval_real.py —— [venv torch] 真机 sim-to-real:纯轮速模型 vs 激光参考 TUM。
# 模型/LS twist → 积分轨迹 → 与激光 TUM 比 RPE(分段)+ 漂移%。诚实:不比全程绝对 APE。
# 用法: ~/...venv.../python lwt/eval_real.py runs/lwt_wheel.pt <real.npz> <lidar_tum.txt>
import argparse, sys, numpy as np, torch
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from lwt.model import TwistTCN
from lwt.kinematics import ls_twist, integrate_twist, rpe_segment, _path_len

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("ckpt"); ap.add_argument("npz"); ap.add_argument("tum")
    a = ap.parse_args()
    ck = torch.load(a.ckpt, weights_only=False); dev = "cuda" if torch.cuda.is_available() else "cpu"
    m = TwistTCN(ck["in_ch"], ck["cfg"]["model"]["tcn_channels"], ck["cfg"]["model"]["tcn_layers"],
                 ck["cfg"]["model"]["kernel"]).to(dev); m.load_state_dict(ck["model"]); m.eval()
    W = ck["cfg"]["data"]["window"]
    d = np.load(a.npz); t = d["t_wheel"][W-1:]
    ls = ls_twist(d["steer"][W-1:], d["speed"][W-1:])
    # TwistDataset.make_windows requires gt_vx/vy/wz (sim-only); real bags have none.
    # Build inference windows directly from features without needing ground truth.
    with_imu = (ck["in_ch"] == 9)
    feat = [d["steer"], d["speed"]]
    if with_imu:
        if "imu_yawrate" not in d:
            raise KeyError(f"{a.npz} 缺 imu_yawrate(需 phase-2 extract.py 重新抽取)")
        feat += [d["imu_yawrate"]]   # phase-2 单通道重力投影 yaw-rate
    F = np.concatenate(feat, axis=1).astype(np.float32)  # (N, C)
    norm = ck["norm"]
    preds = []
    with torch.no_grad():
        for i in range(len(F) - W + 1):
            w = F[i:i+W].copy()  # (W, C)
            lsp = np.array(ls_twist(w[-1, :4], w[-1, 4:8]), dtype=np.float32)
            if norm is not None:
                w = (w - norm[0]) / norm[1]
            x = torch.from_numpy(w.T.copy()).float()[None].to(dev)
            lsp_t = torch.from_numpy(lsp).float()[None].to(dev)
            twp, _ = m(x, lsp_t)
            preds.append(twp[0].cpu().numpy())
    pred = np.array(preds)
    T = np.loadtxt(a.tum); ref = np.column_stack([T[:, 0], T[:, 1], T[:, 2]])
    def metr(tw):
        tr = integrate_twist(t, tw)
        return rpe_segment(tr, ref, 10.0), rpe_segment(tr, ref, 50.0)
    plen = _path_len(ref[:, 1:3])
    print(f"=== sim-to-real {Path(a.npz).name} (激光路径 {plen:.0f}m) ===")
    print(f"  RPE@10m / @50m  LS   = {metr(ls)[0]:.2f}% / {metr(ls)[1]:.2f}%")
    print(f"  RPE@10m / @50m  模型 = {metr(pred)[0]:.2f}% / {metr(pred)[1]:.2f}%")
    print("  sim-to-real 门槛(模型 RPE < LS):", "PASS" if metr(pred)[0] < metr(ls)[0] else "FAIL")

if __name__ == "__main__":
    main()
