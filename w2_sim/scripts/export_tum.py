#!/usr/bin/env python3
# scripts/export_tum.py — 从 bag 的 /sim/ground_truth_odom 导出 TUM 轨迹。
# 用法: python3 export_tum.py <bag目录> <out.txt>
import sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry


def read_odom(bag_dir, topic="/sim/ground_truth_odom"):
    reader = rosbag2_py.SequentialReader()
    # storage_id 显式给 "mcap";不显式则依赖 metadata.yaml 自动探测,
    # 若 bag 尚未 finalize(无 metadata)会抛 "No storage could be initialized"。
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name == topic:
            m = deserialize_message(data, Odometry)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            p, q = m.pose.pose.position, m.pose.pose.orientation
            yield t, p.x, p.y, p.z, q.x, q.y, q.z, q.w


def main():
    rows = list(read_odom(sys.argv[1]))
    assert rows, "bag 里没有 ground_truth_odom"
    with open(sys.argv[2], "w") as f:
        for r in rows:
            f.write(" ".join(f"{v:.6f}" for v in r) + "\n")
    print(f"TUM_OK {len(rows)} poses -> {sys.argv[2]}")


if __name__ == "__main__":
    main()
