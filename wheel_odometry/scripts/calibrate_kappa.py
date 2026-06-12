#!/usr/bin/env python3
"""Estimate the wheel-odometry yaw-bias constant κ from a rosbag2.

Background (see ``doc/wheel_odometry_plan.md`` §"轮速 yaw 标定 κ"):
    The swerve 8×3 least-squares solves a body twist (vx, vy, ω_z) from the
    four (steering-angle, drive-speed) pairs. The instantaneous ω_z tracks the
    IMU/GLIM yaw rate almost perfectly (corr ~0.996), but *integrated* heading
    drifts ~1.9 deg/s on w2 — a constant calibration bias from sub-degree
    steering zero-points (~0.25°) plus ~0.4% left/right wheel-speed asymmetry,
    NOT slip. The whole effect collapses to one speed-proportional term:

        ω_z_corrected = ω_z_raw - κ · vx          (κ in rad/m)

    Calibrating κ against the gyro cuts the heading drift ~27× (1.9 -> 0.07
    deg/s). κ is a physical property of the chassis: stable within a session
    (~3%) but drifts across weeks (~24% over 2 weeks on w2), so re-estimate it
    per deployment session — that is exactly what this tool is for.

What it does:
    1. reads /robot/wheel_status (JointState) and the IMU topic from a bag
    2. IMU mount auto-calibration: R_base_imu from the static-window avg_accel
       (the rslidar IMU is mounted ~58° tilted), plus gyro bias b_g
    3. builds the constant 8×3 geometry LS, solves ω_z_raw and vx per frame
    4. κ = argmin Σ_moving (ω_raw - κ·vx - yaw_imu)²  (closed-form)
    5. reports yaw-rate and integrated-heading drift before/after, and — if a
       TUM ``traj_imu.txt`` (GLIM truth) sits next to the bag — absolute APE-ish
       heading error against it.

Usage:
    python3 calibrate_kappa.py <bag_dir> [--wheelbase L] [--track W]
        [--wheel-radius r] [--imu-topic /rslidar_imu_data]
        [--chassis-topic /robot/wheel_status] [--static-sec 2.0]
        [--speed-min 0.05] [--output kappa.yaml]

Dependencies: numpy, ROS2 python (rclpy.serialization, rosbag2_py,
rosidl_runtime_py). scipy is NOT required (κ is closed-form).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

# JointState.name -> FL,FR,RL,RR ordering (same convention as dump_wheel_csv.py)
WHEEL_KEYS = ("front_left", "front_right", "rear_left", "rear_right")


def resolve_idx(names):
    out = []
    for k in WHEEL_KEYS:
        match = next((i for i, n in enumerate(names) if k in n), None)
        out.append(match)
    if any(i is None for i in out):
        return [0, 1, 2, 3]
    return out


def from_two_vectors(a, b):
    """Minimal rotation taking unit(a) onto unit(b) (Eigen::FromTwoVectors)."""
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3) if c > 0 else -np.eye(3)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))


def quat2R(q):  # q = [x, y, z, w]
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w),     s * (x * z + y * w)],
        [s * (x * y + z * w),     1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w),     s * (y * z + x * w),     1 - s * (x * x + y * y)],
    ])


def log_so3_z(R):
    """z-component of the rotation vector of R (yaw about world up)."""
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    th = np.arccos(c)
    if th < 1e-9:
        return 0.0
    return th / (2.0 * np.sin(th)) * (R[1, 0] - R[0, 1])


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def read_bag(bag_dir, chassis_topic, imu_topic):
    so = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3")
    co = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr")
    reader = rosbag2_py.SequentialReader()
    reader.open(so, co)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for need in (chassis_topic, imu_topic):
        if need not in types:
            sys.stderr.write(f"ERROR: topic {need} not in bag (have {list(types)})\n")
            sys.exit(2)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[chassis_topic, imu_topic]))
    msgcls = {n: get_message(t) for n, t in types.items()}

    wt, wp, wv = [], [], []
    it, ig, ia = [], [], []
    idx = None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        m = deserialize_message(data, msgcls[topic])
        if topic == chassis_topic:
            if idx is None:
                idx = resolve_idx(list(m.name))
            pos = list(m.position)
            vel = list(m.velocity)
            wt.append(stamp(m.header))
            wp.append([pos[i] for i in idx])
            wv.append([vel[i] for i in idx])
        else:
            it.append(stamp(m.header))
            ig.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
            ia.append([m.linear_acceleration.x, m.linear_acceleration.y,
                       m.linear_acceleration.z])
    return (np.array(wt), np.array(wp), np.array(wv),
            np.array(it), np.array(ig), np.array(ia))


def ls_rows(wheelbase, track):
    """Constant geometry rows of the 8×3 LS; returns the ω_z and vx solve rows."""
    L, W = wheelbase, track
    P = np.array([[L / 2, W / 2], [L / 2, -W / 2],
                  [-L / 2, W / 2], [-L / 2, -W / 2]])
    A = np.zeros((8, 3))
    for i, (x, y) in enumerate(P):
        A[2 * i] = [1, 0, -y]
        A[2 * i + 1] = [0, 1, x]
    pinv = np.linalg.inv(A.T @ A) @ A.T
    return pinv[2], pinv[0]  # rows producing ω_z and vx from the 8-vector b


def solve_twist(theta, speed, row_w, row_vx):
    """Vectorised ω_z_raw and vx over all frames (b = v·[cosθ, sinθ] per wheel)."""
    c = np.cos(theta)
    s = np.sin(theta)
    om = ((row_w[0::2] * c + row_w[1::2] * s) * speed).sum(1)
    vx = ((row_vx[0::2] * c + row_vx[1::2] * s) * speed).sum(1)
    return om, vx


def integrate_heading(t, omega):
    dt = np.diff(t, prepend=t[0])
    return np.cumsum(omega * dt)


def main():
    ap = argparse.ArgumentParser(description="Estimate wheel-odometry yaw-bias κ from a bag.")
    ap.add_argument("bag_dir", type=str)
    ap.add_argument("--wheelbase", type=float, default=0.435, help="L, fore-aft wheel spacing (m)")
    ap.add_argument("--track", type=float, default=0.400, help="W, left-right wheel spacing (m)")
    ap.add_argument("--wheel-radius", type=float, default=1.0,
                    help="speed-field -> m/s multiplier (1.0 if speed already m/s)")
    ap.add_argument("--chassis-topic", type=str, default="/robot/wheel_status")
    ap.add_argument("--imu-topic", type=str, default="/rslidar_imu_data")
    ap.add_argument("--static-sec", type=float, default=2.0,
                    help="startup static window for mount cal + gyro bias (s)")
    ap.add_argument("--speed-min", type=float, default=0.05,
                    help="min |wheel speed| to count a frame as moving (m/s)")
    ap.add_argument("--output", type=str, default=None, help="write κ + params to this YAML")
    args = ap.parse_args()

    bag = Path(args.bag_dir).expanduser().resolve()
    wt, wp, wv, it, ig, ia = read_bag(bag, args.chassis_topic, args.imu_topic)
    if len(wt) < 10 or len(it) < 10:
        sys.stderr.write("ERROR: too few messages\n")
        sys.exit(2)

    # speed -> m/s
    wv = wv * args.wheel_radius

    # --- IMU mount cal + gyro bias from the static window ---
    m0 = it < it[0] + args.static_sec
    if m0.sum() < 5:
        sys.stderr.write("ERROR: static window has <5 IMU samples\n")
        sys.exit(2)
    b_g = ig[m0].mean(0)
    avg_a = ia[m0].mean(0)
    R_base_imu = from_two_vectors(avg_a, np.array([0, 0, 1.0]))
    yaw_imu_full = (R_base_imu @ (ig - b_g).T).T[:, 2]
    yaw_imu = np.interp(wt, it, yaw_imu_full)   # base-frame yaw rate at wheel stamps

    # --- wheel LS ω_z_raw and vx ---
    row_w, row_vx = ls_rows(args.wheelbase, args.track)
    om_raw, vx = solve_twist(wp, wv, row_w, row_vx)

    moving = np.abs(wv).max(1) > args.speed_min
    if moving.sum() < 50:
        sys.stderr.write("ERROR: <50 moving frames; need a bag with motion\n")
        sys.exit(2)

    # --- closed-form κ: argmin Σ (om_raw - κ·vx - yaw_imu)^2 over moving frames ---
    e = (om_raw - yaw_imu)[moving]
    kappa = float(np.dot(e, vx[moving]) / np.dot(vx[moving], vx[moving]))
    om_cal = om_raw - kappa * vx

    # --- report ---
    dur = wt[-1] - wt[0]
    def yawrate_rms(om):
        return float(np.sqrt(np.mean(((om - yaw_imu)[moving]) ** 2)))
    def heading_drift(om):
        h = integrate_heading(wt, om)
        hi = integrate_heading(wt, yaw_imu)
        return float(np.degrees(h[-1] - hi[-1]) / dur)  # deg/s vs gyro

    avg_a_rpy = np.degrees([
        np.arctan2(R_base_imu[2, 1], R_base_imu[2, 2]),
        np.arcsin(-np.clip(R_base_imu[2, 0], -1, 1)),
        np.arctan2(R_base_imu[1, 0], R_base_imu[0, 0]),
    ])

    print(f"bag                 : {bag.name}")
    print(f"duration / moving   : {dur:.0f}s  ({moving.sum()}/{len(wt)} moving frames)")
    print(f"geometry            : L={args.wheelbase} W={args.track} wheel_radius={args.wheel_radius}")
    print(f"gyro bias b_g       : {np.round(b_g, 5)} rad/s")
    print(f"R_base_imu rpy(deg) : {np.round(avg_a_rpy, 2)}  (|avg_accel|={np.linalg.norm(avg_a):.3f})")
    print()
    print(f">>> KAPPA           = {kappa:+.5f} rad/m   <<<")
    print()
    print(f"yaw-rate RMS vs gyro:  raw {yawrate_rms(om_raw):.4f}  ->  +κ {yawrate_rms(om_cal):.4f} rad/s")
    print(f"heading drift vs gyro: raw {heading_drift(om_raw):+.3f}  ->  +κ {heading_drift(om_cal):+.3f} deg/s")

    # --- optional absolute check against GLIM truth ---
    truth = bag / "traj_imu.txt"
    if truth.exists():
        T = np.loadtxt(truth)
        gt = T[:, 0]
        Rs = [quat2R(q) for q in T[:, 4:8]]
        hg = np.zeros(len(gt))
        for k in range(1, len(gt)):
            hg[k] = hg[k - 1] + log_so3_z(Rs[k] @ Rs[k - 1].T)
        hg -= hg[0]
        def vs_truth(om):
            h = integrate_heading(wt, om)
            h_at = np.interp(gt, wt, h)
            h_at -= h_at[0]
            e = h_at - hg
            return np.degrees(np.sqrt(np.mean(e ** 2))), np.degrees(e[-1])
        rms_r, fin_r = vs_truth(om_raw)
        rms_c, fin_c = vs_truth(om_cal)
        rms_i, fin_i = vs_truth(yaw_imu)
        print()
        print(f"vs GLIM truth ({gt[-1]-gt[0]:.0f}s, heading RMS / final-err deg):")
        print(f"  wheel raw   : {rms_r:8.1f} / {fin_r:+7.1f}")
        print(f"  wheel +κ    : {rms_c:8.1f} / {fin_c:+7.1f}")
        print(f"  IMU gyro    : {rms_i:8.1f} / {fin_i:+7.1f}")

    if args.output:
        out = Path(args.output).expanduser().resolve()
        out.write_text(
            "# generated by calibrate_kappa.py\n"
            "wheel_odometry:\n"
            "  ros__parameters:\n"
            f"    yaw_kappa: {kappa:.6f}      # rad/m, curvature-bias correction\n"
            f"    wheelbase: {args.wheelbase}\n"
            f"    track: {args.track}\n"
            f"    wheel_radius: {args.wheel_radius}\n"
        )
        print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
