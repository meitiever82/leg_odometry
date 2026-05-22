#!/usr/bin/env python3
"""Subscribe to an Odometry topic and dump it as a TUM trajectory file on
shutdown, so it can be compared against a reference (e.g. GLIM's traj_imu.txt)
with evo.

TUM line:  timestamp  tx ty tz  qx qy qz qw   (timestamp from the message header,
which wheel_only_node copies from /robot/wheel_status — i.e. absolute sensor
time, the same clock GLIM stamps with, so evo can associate by time directly).

ROS2 params:
    odom_topic (str)  topic to record         (default /wheel_odometry)
    out_tum    (str)  output TUM file          (default /tmp/wheel_odom.tum)
"""

from __future__ import annotations

import signal
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry


class OdomToTum(Node):
    def __init__(self):
        super().__init__('odom_to_tum')
        self.declare_parameter('odom_topic', '/wheel_odometry')
        self.declare_parameter('out_tum', '/tmp/wheel_odom.tum')
        topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.out_tum = Path(
            self.get_parameter('out_tum').get_parameter_value().string_value).expanduser()
        self.rows: list[str] = []

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2000,
        )
        self.create_subscription(Odometry, topic, self.cb, qos)
        self.get_logger().info(f"odom_to_tum recording {topic} -> {self.out_tum}")

    def cb(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.rows.append(
            f"{t:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} "
            f"{q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}")

    def save(self):
        if len(self.rows) < 2:
            self.get_logger().warn(f"only {len(self.rows)} samples; not writing TUM")
            return
        self.out_tum.parent.mkdir(parents=True, exist_ok=True)
        self.out_tum.write_text("\n".join(self.rows) + "\n")
        self.get_logger().info(f"wrote {len(self.rows)} poses to {self.out_tum}")


def main():
    rclpy.init(args=sys.argv)
    node = OdomToTum()
    signal.signal(signal.SIGTERM, lambda *_: rclpy.shutdown())
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.save()
        finally:
            try:
                node.destroy_node()
            except Exception:
                pass
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
