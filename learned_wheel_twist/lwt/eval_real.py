#!/usr/bin/env python
# lwt/eval_real.py —— [venv torch] 真机 sim-to-real:纯轮速模型 vs 激光参考 TUM。
# 模型/LS twist → 积分轨迹 → 与激光 TUM 比 RPE(分段)+ 漂移%。诚实:不比全程绝对 APE。
# 用法: ~/...venv.../python lwt/eval_real.py runs/lwt_wheel.pt <real.npz> <lidar_tum.txt>
import argparse, sys, numpy as np, torch
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from lwt.model import TwistTCN
from lwt.dataset import build_features
from lwt.kinematics import ls_twist, integrate_twist, rpe_segment, _path_len, ape_se2_percent

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("ckpt"); ap.add_argument("npz"); ap.add_argument("tum")
    a = ap.parse_args()
    ck = torch.load(a.ckpt, weights_only=False); dev = "cuda" if torch.cuda.is_available() else "cpu"
    prior = ck["cfg"]["model"].get("prior", "ls")
    m = TwistTCN(ck["in_ch"], ck["cfg"]["model"]["tcn_channels"], ck["cfg"]["model"]["tcn_layers"],
                 ck["cfg"]["model"]["kernel"], prior=prior).to(dev); m.load_state_dict(ck["model"]); m.eval()
    W = ck["cfg"]["data"]["window"]
    d = np.load(a.npz); t = d["t_wheel"][W-1:]
    ls = ls_twist(d["steer"][W-1:], d["speed"][W-1:])
    # TwistDataset.make_windows requires gt_vx/vy/wz (sim-only); real bags have none.
    # Build inference windows directly from features without needing ground truth.
    dd = ck["cfg"]["model"].get("data_driven", False)         # phase-3
    wza = ck["cfg"]["model"].get("wz_anchor", False)          # phase-4
    with_imu = (ck["in_ch"] == 9)
    F = build_features(d, with_imu=with_imu, data_driven=dd, wz_anchor=wza).astype(np.float32)  # (N, C)
    yawrate = np.asarray(d["imu_yawrate"]).reshape(-1).astype(np.float32)  # (N,) 去偏陀螺 base yaw-rate
    norm = ck["norm"]
    iwp = ck["cfg"]["model"].get("imu_wz_prior", False)   # phase-2b
    if iwp: print("  [imu_wz_prior] 残差先验 wz = imu_yawrate")
    if dd: print("  [data-driven] prior='none' 直接回归(14ch raw 轮速+base IMU,无 LS 先验)")
    if wza: print("  [wz-anchor] 先验=[0,0,imu_yawrate],wz=陀螺+残差,vx/vy 从 raw 学(14ch,无 LS)")
    preds = []; gyro_end = []
    with torch.no_grad():
        for i in range(len(F) - W + 1):
            w = F[i:i+W].copy()  # (W, C)
            ye = float(yawrate[i+W-1])   # 窗末 imu_yawrate(未标准化)
            gyro_end.append(ye)
            if dd:
                lsp = np.zeros(3, dtype=np.float32)   # data-driven:无 LS 先验
            elif wza:
                lsp = np.array([0.0, 0.0, ye], dtype=np.float32)  # wz-anchor:[0,0,窗末陀螺]
            else:
                lsp = np.array(ls_twist(w[-1, :4], w[-1, 4:8]), dtype=np.float32)
                if iwp:
                    lsp[2] = w[-1, 8]   # 窗末 imu_yawrate(第 9 通道,未标准化)
            if norm is not None:
                w = (w - norm[0]) / norm[1]
            x = torch.from_numpy(w.T.copy()).float()[None].to(dev)
            lsp_t = torch.from_numpy(lsp).float()[None].to(dev)
            twp, _ = m(x, lsp_t)
            preds.append(twp[0].cpu().numpy())
    pred = np.array(preds); gyro_end = np.array(gyro_end)
    T = np.loadtxt(a.tum); ref = np.column_stack([T[:, 0], T[:, 1], T[:, 2]])
    def metr(tw):
        tr = integrate_twist(t, tw)
        return (rpe_segment(tr, ref, 10.0), rpe_segment(tr, ref, 50.0),
                ape_se2_percent(tr, ref))
    plen = _path_len(ref[:, 1:3])
    mls, mpr = metr(ls), metr(pred)
    print(f"=== sim-to-real {Path(a.npz).name} (激光路径 {plen:.0f}m) ===")
    print(f"  RPE@10m / @50m / SE2-APE  LS   = {mls[0]:.2f}% / {mls[1]:.2f}% / {mls[2]:.2f}%")
    print(f"  RPE@10m / @50m / SE2-APE  模型 = {mpr[0]:.2f}% / {mpr[1]:.2f}% / {mpr[2]:.2f}%")
    print("  sim-to-real 门槛(模型 RPE@10m < LS):", "PASS" if mpr[0] < mls[0] else "FAIL")
    # 机理检查:wz_model 相对 imu_yawrate(去偏陀螺)的有符号 DC 偏差 + 积分 Δyaw[deg]。
    # wz-anchor 期望 ≈0(陀螺锚定杀死直流偏置);phase-3 data-driven 曾 -42/-51/-86°。
    dwz = pred[:, 2] - gyro_end
    dt = np.diff(t); dyaw_int = float(np.sum(dwz[:-1] * dt))   # 中点近似积分残差 wz·dt
    print(f"  [机理] mean(wz_model - imu_yawrate) = {float(dwz.mean()):+.5f} rad/s, "
          f"积分 Δyaw = {np.degrees(dyaw_int):+.1f}°")
    # 航向漂移 vs 激光:模型积分末端 yaw - 激光参考末端 yaw(由参考 xy 切线估计)。
    tr_pred = integrate_twist(t, pred)
    yaw_pred_end = float(tr_pred[-1, 3])
    # 激光参考末端航向:用末段位移方向近似(参考 TUM 无显式 yaw 列时)。
    if ref.shape[0] >= 2:
        yaw_ref_end = float(np.arctan2(ref[-1, 2] - ref[-2, 2], ref[-1, 1] - ref[-2, 1]))
        yaw_pred_dir = float(np.arctan2(tr_pred[-1, 2] - tr_pred[-2, 2], tr_pred[-1, 1] - tr_pred[-2, 1]))
        hd = np.degrees(np.arctan2(np.sin(yaw_pred_dir - yaw_ref_end), np.cos(yaw_pred_dir - yaw_ref_end)))
        print(f"  [航向漂移] 末端切向 yaw 差(模型-激光) = {hd:+.1f}°  (积分末端 yaw={np.degrees(yaw_pred_end):+.1f}°)")

if __name__ == "__main__":
    main()
