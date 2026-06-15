#!/usr/bin/env python3
# grab_camera_frame.py — 验证工具:订阅 /camera/color/image_raw(BEST_EFFORT),
# 抓几帧,打印像素统计(min/max/mean/std),把第一帧非空图存成 PNG。
# 用法: python3 grab_camera_frame.py [--out /tmp/w2_cam.png] [--timeout 60] [--n 5]
import argparse
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


class Grabber(Node):
    def __init__(self, n):
        super().__init__("w2_cam_grabber")
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST
        self.sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.cb, qos)
        self.frames = []
        self.n = n

    def cb(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        self.frames.append((msg.width, msg.height, msg.encoding, arr.copy()))
        self.get_logger().info(
            f"frame {len(self.frames)}: {msg.width}x{msg.height} {msg.encoding} "
            f"min={arr.min()} max={arr.max()} mean={arr.mean():.2f} std={arr.std():.2f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/w2_cam.png")
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--n", type=int, default=5)
    a = ap.parse_args()

    rclpy.init()
    node = Grabber(a.n)
    import time
    t0 = time.time()
    while rclpy.ok() and len(node.frames) < a.n and time.time() - t0 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.5)

    if not node.frames:
        print("RESULT: NO_FRAMES_RECEIVED", flush=True)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(2)

    # 选 max 最大的一帧存图
    best = max(node.frames, key=lambda f: int(f[3].max()))
    w, h, enc, arr = best
    print(f"RESULT: got {len(node.frames)} frames; "
          f"best min={arr.min()} max={arr.max()} mean={arr.mean():.2f} std={arr.std():.2f}",
          flush=True)
    try:
        ch = arr.size // (w * h)
        img = arr.reshape(h, w, ch)
        from PIL import Image as PImage
        if enc.startswith("rgb"):
            PImage.fromarray(img[:, :, :3], "RGB").save(a.out)
        elif enc.startswith("bgr"):
            PImage.fromarray(img[:, :, :3][:, :, ::-1], "RGB").save(a.out)
        else:
            PImage.fromarray(img[:, :, 0], "L").save(a.out)
        print(f"SAVED_PNG: {a.out}", flush=True)
        # 简单内容描述:分块均值,判断是否单一颜色
        q = img[:, :, :3].astype(float)
        print(f"CONTENT: per-channel mean R={q[:,:,0].mean():.1f} "
              f"G={q[:,:,1].mean():.1f} B={q[:,:,2].mean():.1f}; "
              f"spatial std of luminance={q.mean(2).std():.2f}", flush=True)
    except Exception as e:
        print(f"SAVE_FAILED: {e}", flush=True)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
