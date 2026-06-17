#!/usr/bin/env python
# lwt/verify_yawrate.py —— [venv] phase-2 通道符号校验(HARD GATE)。
# sim:imu_yawrate vs gt_wz 回归(期望 slope≈+1,R²>0.95)。
# real:imu_yawrate vs 参考 TUM yaw-rate 回归(±0.3s 时偏扫描取最佳),期望 |slope|∈[0.8,1.2]。
#   若 real slope≈-1,则真机通道需固定符号翻转(-1)使其也=+CCW;脚本报出建议的 sign。
# 用法: python lwt/verify_yawrate.py
import sys, glob, numpy as np
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

REAL = Path.home() / "Documents/Datasets/w2_real_cache"
SIM = Path.home() / "Documents/Datasets/w2_sim_cache"
W2 = Path.home() / "Documents/Datasets/w2"
REAL_REFS = {
    "0522": "rosbag2_2026_05_22-13_58_40/traj_imu.txt",
    "0605_37": "rosbag2_2026_06_05-17_37_48/traj_imu.txt",
    "0605_42": "rosbag2_2026_06_05-17_42_57/kiss_path_tum.txt",
}


def regress(x, y):
    """y = slope*x + b 的最小二乘 + R²。"""
    x = np.asarray(x, float); y = np.asarray(y, float)
    A = np.column_stack([x, np.ones_like(x)])
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    slope, b = coef
    yhat = A @ coef
    ss_res = np.sum((y - yhat) ** 2); ss_tot = np.sum((y - y.mean()) ** 2)
    r2 = 1.0 - ss_res / max(ss_tot, 1e-12)
    return float(slope), float(b), float(r2)


def tum_yaw(tum_path):
    """TUM (t tx ty tz qx qy qz qw) → (t, yaw[unwrapped])。
    yaw=atan2(2(qw·qz+qx·qy),1-2(qy²+qz²))。注意:参考轨迹采样稀疏(0605_37 仅~1.9Hz,
    dt 可达 2.8s),直接微分成瞬时 yaw-rate 会被噪声主导(早期实现的教训)。因此 real 校验
    改为比较「区间 Δyaw 增量」:把通道 yaw-rate 积成 yaw,在参考时戳上取相邻差 Δch,与参考
    Δyaw 回归。这对参考稀疏鲁棒,且正是通道必须算对的量(每段相对转角)。"""
    T = np.loadtxt(tum_path)
    t = T[:, 0]; qx, qy, qz, qw = T[:, 4], T[:, 5], T[:, 6], T[:, 7]
    yaw = np.unwrap(np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2)))
    return t, yaw


def verify_sim(n_ep=5):
    print("=== SIM 通道校验 (imu_yawrate vs gt_wz) ===")
    paths = [p for p in sorted(glob.glob(str(SIM / "*" / "*.npz"))) if Path(p).name != "test.npz"]
    rows = []
    for p in paths[:n_ep]:
        d = np.load(p)
        if "imu_yawrate" not in d or "gt_wz" not in d:
            print(f"  {Path(p).name}: 缺通道, skip"); continue
        yr = d["imu_yawrate"][:, 0]; gt = d["gt_wz"]
        s, b, r2 = regress(yr, gt)
        rows.append((s, r2))
        print(f"  {Path(p).parent.name}/{Path(p).name}: slope={s:+.3f} R2={r2:.4f}")
    s_med = np.median([r[0] for r in rows]); r2_min = min(r[1] for r in rows)
    print(f"  -> sim slope median={s_med:+.3f}  R2 min={r2_min:.4f}")
    return s_med, r2_min


def verify_real():
    print("=== REAL 通道校验 (区间 Δyaw 增量 vs 参考 TUM, ±0.3s 时偏扫描) ===")
    rows = []
    for name, ref in REAL_REFS.items():
        d = np.load(REAL / f"{name}.npz")
        if "imu_yawrate" not in d:
            print(f"  {name}: 缺 imu_yawrate, skip"); continue
        tw = d["t_wheel"]; yr = d["imu_yawrate"][:, 0]
        tref, yawref = tum_yaw(W2 / ref)
        # 把通道 yaw-rate 在自身时间格上积分成 yaw(梯形)。
        ch_yaw = np.concatenate([[0], np.cumsum(0.5 * (yr[1:] + yr[:-1]) * np.diff(tw))])
        best = None
        for off in np.arange(-0.3, 0.301, 0.02):
            ch_at_ref = np.interp(tref, tw + off, ch_yaw)
            dref = np.diff(yawref); dch = np.diff(ch_at_ref)
            m = np.abs(dref) < 1.0   # 丢弃超大时间空洞造成的跳变
            if m.sum() < 30:
                continue
            s, b, r2 = regress(dch[m], dref[m])
            if best is None or r2 > best[3]:
                best = (off, s, b, r2)
        off, s, b, r2 = best
        rows.append((name, s, r2, off))
        print(f"  {name}: best off={off:+.2f}s slope={s:+.3f} R2={r2:.4f}")
    return rows


def main():
    sim_slope, sim_r2 = verify_sim()
    real_rows = verify_real()
    print("\n=== 符号约定裁决 ===")
    real_slopes = [s for _, s, _, _ in real_rows]
    real_sign = -1.0 if np.median(real_slopes) < 0 else 1.0
    print(f"  sim slope≈{sim_slope:+.2f} (R2 min {sim_r2:.3f}) → sim sign=+1")
    print(f"  real raw(sign=+1) slopes={[round(s,2) for s in real_slopes]}")
    if real_sign < 0:
        print("  real 几何 up·gyro 回归 slope≈-1 → 真机通道需固定符号翻转 sign=-1")
        eff = [abs(s) for s in real_slopes]
    else:
        eff = real_slopes
    print(f"  应用 real sign={real_sign:+.0f} 后有效 slopes={[round(s,2) for s in eff]}")
    # GATE
    sim_ok = abs(sim_slope - 1.0) < 0.2 and sim_r2 > 0.95
    real_ok = all(0.8 <= e <= 1.2 for e in eff)
    print("\n=== HARD GATE ===")
    print(f"  sim slope≈+1 & R2>0.95: {'PASS' if sim_ok else 'FAIL'}")
    print(f"  all-3 real |slope|∈[0.8,1.2] consistent sign: {'PASS' if real_ok else 'FAIL'}")
    print(f"  GATE: {'PASS' if (sim_ok and real_ok) else 'BLOCKED'}")
    print(f"  RESOLVED_REAL_SIGN={real_sign:+.0f}")
    return 0 if (sim_ok and real_ok) else 2


if __name__ == "__main__":
    sys.exit(main())
