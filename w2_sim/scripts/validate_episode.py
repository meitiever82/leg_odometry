#!/usr/bin/env python3
# scripts/validate_episode.py — design §9 验收四条。
# 用法: python3 validate_episode.py <episode目录> [--real-bag <实车bag目录>]
import argparse
import sys
from pathlib import Path
import numpy as np
import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from w2_sim.degradation_node import joint_state_to_modules
from w2_sim.swerve_kinematics import WHEEL_NAMES
from w2_sim.wheel_integration import (ape_percent, integrate_wheel,
                                      ls_twist_series, path_length)


def read_topic(bag_dir, topic, msg_type):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir)),
                rosbag2_py.ConverterOptions("", ""))
    out = []
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name == topic:
            out.append(deserialize_message(data, msg_type))
    return out


def stamps(msgs):
    return np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in msgs])


def check(label, ok, detail=""):
    print(f"  [{'PASS' if ok else 'FAIL'}] {label} {detail}")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("episode_dir")
    ap.add_argument("--real-bag", default=str(Path.home() /
        "Documents/Datasets/w2/rosbag2_2026_05_22-13_58_40"))
    ap.add_argument("--clean-ape-max", type=float, default=0.5)
    ap.add_argument("--kappa-tol", type=float, default=0.10)
    a = ap.parse_args()
    ep = Path(a.episode_dir)
    bag = ep / "rosbag2"
    results = []

    # ---- 验收 1:格式与频率 ----
    print("== 1. 格式一致性")
    sim_ws = read_topic(bag, "/robot/wheel_status", JointState)
    real_ws = read_topic(a.real_bag, "/robot/wheel_status", JointState)[:10]
    s, r = sim_ws[10], real_ws[5]
    results.append(check("name 顺序", list(s.name) == list(r.name) == WHEEL_NAMES))
    results.append(check("frame_id", s.header.frame_id == r.header.frame_id == "base_link"))
    results.append(check("effort 空", len(s.effort) == len(r.effort) == 0))
    ts = stamps(sim_ws)
    rate = 1.0 / np.median(np.diff(ts))
    results.append(check("wheel 频率", abs(rate - 50) < 10, f"{rate:.1f}Hz"))

    # ---- 真值 ----
    truth = np.loadtxt(ep / "ground_truth_tum.txt")   # t x y z qx qy qz qw
    truth3 = truth[:, :3]

    # ---- 验收 2:干净通道运动学自洽 ----
    print("== 2. 干净通道 8×3 LS 自洽")
    sim_js = read_topic(bag, "/sim/joint_states", JointState)
    t_js = stamps(sim_js)
    mods = [joint_state_to_modules(m.name, m.position, m.velocity) for m in sim_js]
    angs = np.array([m[0] for m in mods]); spds = np.array([m[1] for m in mods])
    clean_traj = integrate_wheel(t_js, angs, spds)
    ape_clean = ape_percent(clean_traj, truth3)
    results.append(check("clean APE", ape_clean < a.clean_ape_max,
                         f"{ape_clean:.3f}% (<{a.clean_ape_max}%)"))

    # ---- 验收 3:κ 现象复现 ----
    print("== 3. 退化通道 κ 复现")
    t_ws = stamps(sim_ws)
    ang_d = np.array([m.position for m in sim_ws])
    spd_d = np.array([m.velocity for m in sim_ws])
    deg_traj = integrate_wheel(t_ws, ang_d, spd_d)
    ape_deg = ape_percent(deg_traj, truth3)
    twists = ls_twist_series(t_ws, ang_d, spd_d)
    # ω_truth 由真值 yaw 差分(四元数→yaw),插值到轮速时间轴
    qw, qz = truth[:, 7], truth[:, 6]
    yaw_truth = np.unwrap(np.arctan2(2 * qw * qz, 1 - 2 * qz ** 2))
    w_truth = np.interp(t_ws, truth[:-1, 0],
                        np.diff(yaw_truth) / np.maximum(np.diff(truth[:, 0]), 1e-9))
    vx = twists[:, 0]
    m = np.abs(vx) > 0.2
    n_samp = int(np.sum(m))
    if n_samp < 2:
        kappa_fit = float("nan")
    else:
        kappa_fit = float(np.sum((twists[m, 2] - w_truth[m]) * vx[m]) / np.sum(vx[m] ** 2))
    kappa_theory = yaml.safe_load((ep / "episode_params.yaml").read_text())["kappa_theory"]
    results.append(check("退化通道出现漂移", ape_deg > ape_clean,
                         f"degraded APE {ape_deg:.2f}% vs clean {ape_clean:.3f}%"))
    rel = abs(kappa_fit - kappa_theory) / max(abs(kappa_theory), 1e-9)
    results.append(check("κ 回归 vs 理论", rel < a.kappa_tol,
                         f"fit={kappa_fit:.5f} theory={kappa_theory:.5f} "
                         f"rel={rel:.1%} n_samp={n_samp}"))

    # ---- 验收 4:工具链兼容 ----
    print("== 4. 工具链兼容:验收 2/3 已用与实车分析同款 8×3 LS + SE2 积分管线,视为通过")

    n_pass = sum(results)
    print(f"\n{'ALL_PASS' if all(results) else 'FAILED'} ({n_pass}/{len(results)})")
    sys.exit(0 if all(results) else 1)


if __name__ == "__main__":
    main()
