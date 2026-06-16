#!/usr/bin/env python3
"""scripts/diag_slip.py — 量化轮地打滑(wheel-LS ω_z vs 真值 ω_z RMS)。

诊断 PhysX 轮地接触打滑用:解码 /robot/wheel_status(8×3 LS 解 ω_z),与
/sim/ground_truth_odom twist.angular.z 比,在 moving 帧上算 RMS 和 corr。
目标:把 RMS 从 baseline ~0.29 压到 ≤ ~0.08(真机 0.05)。

用法: python3 diag_slip.py <episode目录或 rosbag2 目录>
"""
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from w2_sim.swerve_kinematics import forward_kinematics_ls


def read_two(bag_dir, t1, mt1, t2, mt2):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir)),
                rosbag2_py.ConverterOptions("", ""))
    a, b = [], []
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name == t1:
            a.append(deserialize_message(data, mt1))
        elif name == t2:
            b.append(deserialize_message(data, mt2))
    return a, b


def stamps(msgs):
    return np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                     for m in msgs])


def main():
    arg = Path(sys.argv[1])
    bag = arg / "rosbag2" if (arg / "rosbag2").exists() else arg

    ws, odom = read_two(bag, "/robot/wheel_status", JointState,
                        "/sim/ground_truth_odom", Odometry)
    assert ws, "no /robot/wheel_status"
    assert odom, "no /sim/ground_truth_odom"

    t_ws = stamps(ws)
    angs = np.array([m.position for m in ws])
    spds = np.array([m.velocity for m in ws])
    # 8x3 LS per frame -> (vx,vy,wz) and per-wheel max |v| for moving mask
    twists = np.array([forward_kinematics_ls(a, s) for a, s in zip(angs, spds)])
    wz_wheel = twists[:, 2]
    wvmax = np.max(np.abs(spds), axis=1)

    t_od = stamps(odom)
    wz_truth_raw = np.array([m.twist.twist.angular.z for m in odom])
    # interpolate truth wz onto wheel timeline
    order = np.argsort(t_od)
    wz_truth = np.interp(t_ws, t_od[order], wz_truth_raw[order])

    moving = wvmax > 0.05
    n = int(np.sum(moving))
    d = wz_wheel[moving] - wz_truth[moving]
    rms = float(np.sqrt(np.mean(d ** 2)))
    if n >= 2 and np.std(wz_wheel[moving]) > 1e-9 and np.std(wz_truth[moving]) > 1e-9:
        corr = float(np.corrcoef(wz_wheel[moving], wz_truth[moving])[0, 1])
    else:
        corr = float("nan")
    print(f"bag={bag}")
    print(f"frames total={len(t_ws)} moving={n}")
    print(f"wz_wheel std={np.std(wz_wheel[moving]):.4f}  "
          f"wz_truth std={np.std(wz_truth[moving]):.4f}")
    print(f"RMS(wz_wheel - wz_truth) = {rms:.4f} rad/s   corr = {corr:.4f}")
    # also report mean speed-scale-ish: median wheel speed / truth lin speed
    lin_truth = np.array([np.hypot(m.twist.twist.linear.x,
                                   m.twist.twist.linear.y) for m in odom])
    lin_truth_i = np.interp(t_ws, t_od[order], lin_truth[order])
    vx = twists[:, 0]
    mv2 = moving & (lin_truth_i > 0.1)
    if np.any(mv2):
        ratio = np.median(np.abs(vx[mv2]) / np.maximum(lin_truth_i[mv2], 1e-6))
        print(f"median |vx_wheel|/|v_truth| (moving,>0.1) = {ratio:.4f}")
    print(f"RMS_SLIP {rms:.4f}")


if __name__ == "__main__":
    main()
