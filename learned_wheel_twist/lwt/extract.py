#!/usr/bin/env python3
# lwt/extract.py —— [系统 python3 + rosbag2_py] 把一个 episode 的 bag 抽成 .npz。
# 用法: source /opt/ros/jazzy/setup.bash; python3 lwt/extract.py <episode_dir> <out.npz> [--storage mcap]
#   <episode_dir> 含 rosbag2/(仿真)或直接是真机 bag 目录;真机用 --storage sqlite3 --no-gt
import argparse, sys, numpy as np, yaml
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry

WHEEL_KEYS = ("front_left", "front_right", "rear_left", "rear_right")

def _stamp(h): return h.stamp.sec + h.stamp.nanosec*1e-9

def read(bag, storage):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage),
           rosbag2_py.ConverterOptions("", ""))
    wt, wp, wv, it, ig, ia, ot, ovx, ovy, owz = ([] for _ in range(10))
    idx = None
    while r.has_next():
        n, d, _ = r.read_next()
        if n == "/robot/wheel_status":
            m = deserialize_message(d, JointState)
            if idx is None:
                idx = [next(i for i, nm in enumerate(m.name) if k in nm) for k in WHEEL_KEYS]
            wt.append(_stamp(m.header)); wp.append([m.position[i] for i in idx]); wv.append([m.velocity[i] for i in idx])
        elif n == "/sim/imu":
            m = deserialize_message(d, Imu); it.append(_stamp(m.header))
            ig.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
            ia.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
        elif n == "/sim/ground_truth_odom":
            m = deserialize_message(d, Odometry); ot.append(_stamp(m.header))
            tw = m.twist.twist; ovx.append(tw.linear.x); ovy.append(tw.linear.y); owz.append(tw.angular.z)
    return (np.array(wt), np.array(wp), np.array(wv), np.array(it), np.array(ig), np.array(ia),
            np.array(ot), np.array(ovx), np.array(ovy), np.array(owz))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("episode_dir"); ap.add_argument("out")
    ap.add_argument("--storage", default="mcap"); ap.add_argument("--no-gt", action="store_true")
    a = ap.parse_args()
    ep = Path(a.episode_dir).expanduser()
    bag = ep/"rosbag2" if (ep/"rosbag2").exists() else ep
    wt, wp, wv, it, ig, ia, ot, ovx, ovy, owz = read(bag, a.storage)
    assert len(wt) > 50, "wheel_status 太少"
    d = dict(t_wheel=wt, steer=wp, speed=wv)
    if not a.no_gt and len(ot) > 1:
        d["gt_vx"] = np.interp(wt, ot, ovx); d["gt_vy"] = np.interp(wt, ot, ovy); d["gt_wz"] = np.interp(wt, ot, owz)
    if len(it) > 1:
        d["imu_g"] = np.stack([np.interp(wt, it, ig[:, k]) for k in range(3)], 1)
        d["imu_a"] = np.stack([np.interp(wt, it, ia[:, k]) for k in range(3)], 1)
    pf = ep/"episode_params.yaml"
    if pf.exists():
        d["kappa_theory"] = float(yaml.safe_load(pf.read_text()).get("kappa_theory", np.nan))
    np.savez_compressed(a.out, **d)
    print(f"EXTRACT_OK {a.out} wheel={len(wt)} gt={'y' if 'gt_vx' in d else 'n'} imu={'y' if 'imu_g' in d else 'n'}")

if __name__ == "__main__":
    main()
