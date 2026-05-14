#!/usr/bin/env python3
"""Dump /robot/wheel_status (sensor_msgs/JointState) from a rosbag2 to CSV.

Columns:
    t_abs        — header.stamp as floating seconds (Unix epoch)
    t_rel        — seconds since the first message
    dt           — seconds since the previous message
    theta_FL ... — steering angle (rad), assumes JointState.position
    speed_FL ... — wheel-status speed field (rad/s on w2 — multiply by 0.075
                   for ground m/s)

Wheel index → name mapping is taken from JointState.name on the first message,
matching by substring 'front_left' / 'front_right' / 'rear_left' / 'rear_right'.
Falls back to 0..3 = FL,FR,RL,RR if names are missing.

Usage:
    python3 dump_wheel_csv.py <bag_dir> [output.csv]
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

import rclpy.serialization
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


WHEEL_KEYS = ("front_left", "front_right", "rear_left", "rear_right")


def resolve_idx(names):
    out = []
    for k in WHEEL_KEYS:
        match = next((i for i, n in enumerate(names) if k in n), None)
        out.append(match)
    if any(i is None for i in out):
        return [0, 1, 2, 3]
    return out


def main():
    if len(sys.argv) < 2:
        sys.stderr.write(f"usage: {sys.argv[0]} <bag_dir> [output.csv]\n")
        sys.exit(2)
    bag_dir = Path(sys.argv[1]).expanduser().resolve()
    out_csv = Path(sys.argv[2]).expanduser().resolve() if len(sys.argv) > 2 \
        else bag_dir.with_suffix('.wheel.csv')

    storage_options = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    type_by_topic = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if '/robot/wheel_status' not in type_by_topic:
        sys.stderr.write(f"/robot/wheel_status not in bag (topics: {list(type_by_topic)})\n")
        sys.exit(1)
    msg_cls = get_message(type_by_topic['/robot/wheel_status'])

    storage_filter = rosbag2_py.StorageFilter(topics=['/robot/wheel_status'])
    reader.set_filter(storage_filter)

    idx = None
    t0 = None
    t_prev = None
    n = 0
    with open(out_csv, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow([
            't_abs', 't_rel', 'dt',
            'theta_FL', 'theta_FR', 'theta_RL', 'theta_RR',
            'speed_FL', 'speed_FR', 'speed_RL', 'speed_RR',
        ])
        while reader.has_next():
            (_, raw, _) = reader.read_next()
            msg = deserialize_message(raw, msg_cls)
            if idx is None:
                idx = resolve_idx(list(msg.name))
                sys.stderr.write(
                    f"resolved wheel indices: FL={idx[0]} FR={idx[1]} "
                    f"RL={idx[2]} RR={idx[3]}  (names={list(msg.name)})\n")
            if len(msg.position) < 4 or len(msg.velocity) < 4:
                continue
            t_abs = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if t0 is None: t0 = t_abs
            dt = (t_abs - t_prev) if t_prev is not None else 0.0
            t_prev = t_abs
            row = [
                f"{t_abs:.6f}", f"{t_abs - t0:.6f}", f"{dt:.6f}",
                *[f"{msg.position[i]:+.6f}" for i in idx],
                *[f"{msg.velocity[i]:+.6f}" for i in idx],
            ]
            w.writerow(row)
            n += 1
    sys.stderr.write(f"wrote {n} rows → {out_csv}\n")


if __name__ == '__main__':
    main()
