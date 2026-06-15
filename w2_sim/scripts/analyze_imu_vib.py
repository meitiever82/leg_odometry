#!/usr/bin/env python3
"""IMU noise-vs-speed 分桶分析 (design: 运动相关振动标定)。

把 /rslidar_imu_data 每个样本按时间戳对应到当时车速(由 /robot/wheel_status 插值),
按速度分箱,计算每箱的陀螺/加速度"相邻差分噪声"std。

相邻差分法:n_i = (x_i - x_{i-1}) / sqrt(2)。这样消去缓慢漂移/真实运动的低频分量,
留下高频噪声+振动。除以 sqrt(2) 把"两样本独立噪声之差"还原成单样本 std。

车速:对 /robot/wheel_status,真机 velocity 字段直接是各轮线速度 m/s。
取 |v| = mean(|velocity|)(与 degradation_node 在线计算口径一致)。

支持 db3 (sqlite3) 与 mcap。用法:
  analyze_imu_vib.py <bag_dir_or_file> [--imu-topic T] [--wheel-topic T]
"""
import argparse
import sys
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

GYRO_BINS = [0.0, 0.05, 0.2, 0.4, 0.6, 0.8, 1.0, 1.5, 99.0]


def read_db3(path, topics):
    import sqlite3
    con = sqlite3.connect(path)
    cur = con.cursor()
    tmap = {name: (tid, ttype) for tid, name, ttype in
            cur.execute("SELECT id, name, type FROM topics").fetchall()}
    out = {}
    for t in topics:
        if t not in tmap:
            out[t] = []
            continue
        tid, ttype = tmap[t]
        msgtype = get_message(ttype)
        rows = cur.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (tid,)).fetchall()
        out[t] = [(ts, deserialize_message(bytes(d), msgtype)) for ts, d in rows]
    con.close()
    return out


def read_mcap(path, topics):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    reader = SequentialReader()
    reader.open(StorageOptions(uri=path, storage_id="mcap"),
                ConverterOptions("", ""))
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msgtypes = {t: get_message(type_map[t]) for t in topics if t in type_map}
    out = {t: [] for t in topics}
    while reader.has_next():
        topic, data, ts = reader.read_next()
        if topic in msgtypes:
            out[topic].append((ts, deserialize_message(data, msgtypes[topic])))
    return out


def load_bag(path, topics):
    p = Path(path)
    if p.is_dir():
        # sim episodes nest the bag under a rosbag2/ subdir
        if (p / "rosbag2").is_dir() and not list(p.glob("*.db3")) \
                and not list(p.glob("*.mcap")):
            p = p / "rosbag2"
        db3 = list(p.glob("*.db3"))
        mcap = list(p.glob("*.mcap"))
        if db3:
            return read_db3(str(db3[0]), topics)
        if mcap:
            # mcap reader wants the directory uri
            return read_mcap(str(p), topics)
        raise FileNotFoundError(f"no .db3/.mcap in {p}")
    if p.suffix == ".db3":
        return read_db3(str(p), topics)
    return read_mcap(str(p), topics)


def _stamp(m):
    """Use message header stamp (sensor sample time), not record receive time."""
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def wheel_speed_series(wheel_msgs):
    """-> (t[s], speed[m/s]) by |mean(|velocity|)|."""
    ts, sp = [], []
    for _, m in wheel_msgs:
        v = np.asarray(m.velocity, dtype=float)
        if v.size == 0:
            continue
        ts.append(_stamp(m))
        sp.append(float(np.mean(np.abs(v))))
    return np.asarray(ts), np.asarray(sp)


def imu_series(imu_msgs):
    ts, gyro, acc = [], [], []
    for _, m in imu_msgs:
        ts.append(_stamp(m))
        gyro.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
        acc.append([m.linear_acceleration.x, m.linear_acceleration.y,
                    m.linear_acceleration.z])
    return np.asarray(ts), np.asarray(gyro), np.asarray(acc)


def diff_noise(x):
    """相邻差分噪声 -> 每样本(对应 x[1:])的 std 估计向量,逐轴。
    返回 magnitude per-sample: sqrt(sum_axes((dx/sqrt2)^2)) 不取,改逐轴后合成 RMS。"""
    dx = np.diff(x, axis=0) / np.sqrt(2.0)
    return dx  # shape (N-1, naxes), aligned to x[1:]


def bin_curve(speed_at_sample, dgyro, dacc, bins):
    """每个差分样本已有对应车速 -> 按 bin 聚合,算每轴 RMS 再三轴合成。"""
    rows = []
    for lo, hi in zip(bins[:-1], bins[1:]):
        m = (speed_at_sample >= lo) & (speed_at_sample < hi)
        n = int(m.sum())
        if n < 30:
            rows.append((lo, hi, n, np.nan, np.nan))
            continue
        # per-axis std (about 0, since diff already removed mean), then 3-axis RMS
        g = np.sqrt(np.mean(dgyro[m] ** 2))          # rad/s combined RMS over 3 axes
        a = np.sqrt(np.mean(dacc[m] ** 2))           # m/s^2
        rows.append((lo, hi, n, np.degrees(g), a))   # gyro -> deg/s
    return rows


def analyze(path, imu_topic, wheel_topic, label):
    bag = load_bag(path, [imu_topic, wheel_topic])
    wt, ws = wheel_speed_series(bag[wheel_topic])
    it, ig, ia = imu_series(bag[imu_topic])
    if len(it) < 10:
        print(f"[{label}] IMU 样本太少: {len(it)}", file=sys.stderr)
        return None
    if len(wt) < 2:
        print(f"[{label}] wheel 样本太少: {len(wt)}", file=sys.stderr)
        return None

    order = np.argsort(wt)
    wt, ws = wt[order], ws[order]
    iorder = np.argsort(it)
    it, ig, ia = it[iorder], ig[iorder], ia[iorder]

    # interp speed at each imu sample
    speed_at_imu = np.interp(it, wt, ws)

    dg = diff_noise(ig)                 # aligned to it[1:]
    da = diff_noise(ia)
    speed_diff = speed_at_imu[1:]       # speed at the later sample of each pair

    # guard against time jumps: only keep pairs with small dt
    dt = np.diff(it)
    med = np.median(dt)
    keep = (dt < 5 * med) & (dt > 0)
    dg, da, speed_diff = dg[keep], da[keep], speed_diff[keep]

    rows = bin_curve(speed_diff, dg, da, GYRO_BINS)
    rate = 1.0 / med
    print(f"\n=== [{label}] noise-vs-speed  (IMU {len(it)} smp @ ~{rate:.0f}Hz, "
          f"wheel {len(wt)} smp) ===")
    print(f"{'speed bin (m/s)':>16} {'N':>7} {'gyro deg/s':>12} {'accel m/s^2':>12}")
    for lo, hi, n, g, a in rows:
        gs = f"{g:.3f}" if not np.isnan(g) else "  -  "
        as_ = f"{a:.3f}" if not np.isnan(a) else "  -  "
        print(f"{lo:6.2f}-{hi:<6.2f}{'':>3} {n:>7} {gs:>12} {as_:>12}")
    # overall full-trace diff noise for reference
    gall = np.degrees(np.sqrt(np.mean(dg ** 2)))
    aall = np.sqrt(np.mean(da ** 2))
    print(f"{'ALL':>16} {len(dg):>7} {gall:>12.3f} {aall:>12.3f}")
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--imu-topic", default="/rslidar_imu_data")
    ap.add_argument("--wheel-topic", default="/robot/wheel_status")
    ap.add_argument("--label", default=None)
    args = ap.parse_args()
    analyze(args.bag, args.imu_topic, args.wheel_topic,
            args.label or Path(args.bag).name)


if __name__ == "__main__":
    main()
