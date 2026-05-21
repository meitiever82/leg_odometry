#!/usr/bin/env python3
"""Subscribe to /wheel_odometry, accumulate the pose, save a trajectory PNG
when the node is shut down (Ctrl+C / SIGTERM from launch).

No intermediate CSV — only one PNG at the end.

ROS2 params:
    odom_topic   (str)  topic to listen on (default /wheel_odometry)
    out_png      (str)  output PNG path     (default /tmp/wheel_odom_trajectory.png)
"""

from __future__ import annotations

# Import matplotlib eagerly: the first import takes ~1 s and launch's
# shutdown grace period is short. Doing it now means save_plot is fast.
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402
import numpy as np  # noqa: E402

import math
import signal
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry


class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')
        self.declare_parameter('odom_topic', '/wheel_odometry')
        self.declare_parameter('out_png', '/tmp/wheel_odom_trajectory.png')
        # NaN = unknown → no threshold line drawn. Launch fills this from yaml.
        self.declare_parameter('slip_threshold', float('nan'))
        topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.out_png = Path(self.get_parameter(
            'out_png').get_parameter_value().string_value).expanduser()
        self.slip_thr: float = self.get_parameter(
            'slip_threshold').get_parameter_value().double_value

        self.t:    list[float] = []
        self.x:    list[float] = []
        self.y:    list[float] = []
        self.yaw:  list[float] = []
        self.vx:   list[float] = []   # twist.linear.x = vx_body, m/s (signed)
        self.vy:   list[float] = []   # twist.linear.y = vy_body, m/s (signed)
        self.wz:   list[float] = []   # twist.angular.z, rad/s
        self.res:  list[float] = []   # LS residual (m/s), encoded in cov[0]

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=500,
        )
        self.sub = self.create_subscription(Odometry, topic, self.cb, qos)
        self.get_logger().info(
            f"trajectory_plotter listening on {topic}  →  {self.out_png}")

    def cb(self, msg: Odometry):
        s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.t.append(s)
        self.x.append(msg.pose.pose.position.x)
        self.y.append(msg.pose.pose.position.y)
        self.yaw.append(math.atan2(siny, cosy))
        # twist is in child_frame_id (body) per ROS convention; wheel_only_node
        # publishes the LS body-frame velocity directly.
        self.vx.append(msg.twist.twist.linear.x)
        self.vy.append(msg.twist.twist.linear.y)
        self.wz.append(msg.twist.twist.angular.z)
        # wheel_only_node squats twist.covariance[0] with LS residual (m/s).
        self.res.append(float(msg.twist.covariance[0]))

    def save_plot(self):
        n = len(self.t)
        if n < 2:
            self.get_logger().warn(
                f"only {n} odom sample(s) received; nothing to plot")
            return

        xs   = np.asarray(self.x);    ys = np.asarray(self.y)
        yaws = np.asarray(self.yaw)
        vx   = np.asarray(self.vx);   vy = np.asarray(self.vy)
        wz   = np.asarray(self.wz)
        res  = np.asarray(self.res)
        t    = np.asarray(self.t) - self.t[0]

        path = float(np.sum(np.hypot(np.diff(xs), np.diff(ys))))
        disp = float(math.hypot(xs[-1] - xs[0], ys[-1] - ys[0]))

        fig, axes = plt.subplots(2, 2, figsize=(13, 10),
                                 gridspec_kw=dict(width_ratios=[1.15, 1]))

        # ---- (0,0) XY trajectory ----
        ax = axes[0, 0]
        pts = np.stack([xs, ys], axis=1).reshape(-1, 1, 2)
        segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
        lc = LineCollection(segs, cmap='viridis', linewidth=1.6)
        lc.set_array(t)
        ax.add_collection(lc)

        dt = float(t[1] - t[0]) if n > 1 else 0.02
        step = max(1, int(3.0 / max(dt, 1e-3)))
        ax.quiver(xs[::step], ys[::step],
                  np.cos(yaws[::step]), np.sin(yaws[::step]),
                  scale=40, width=0.003, color='gray', alpha=0.55)
        ax.scatter([xs[0]],  [ys[0]],  c='lime', s=120, marker='o',
                   edgecolors='k', zorder=5, label='start')
        ax.scatter([xs[-1]], [ys[-1]], c='red',  s=140, marker='*',
                   edgecolors='k', zorder=5, label='end')
        margin = max(1.0, 0.05 * max(float(xs.max() - xs.min()),
                                     float(ys.max() - ys.min())))
        ax.set_xlim(xs.min() - margin, xs.max() + margin)
        ax.set_ylim(ys.min() - margin, ys.max() + margin)
        ax.set_aspect('equal'); ax.grid(True, ls=':', alpha=0.6)
        ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
        ax.set_title(f'XY trajectory   path={path:.2f} m   disp={disp:.2f} m   '
                     f'end=({xs[-1]:+.2f},{ys[-1]:+.2f})   '
                     f'yaw_end={yaws[-1]*180/math.pi:+.1f}°')
        ax.legend(loc='upper right')
        fig.colorbar(lc, ax=ax, shrink=0.8, pad=0.02).set_label('time [s]')

        # ---- (0,1) v_body (signed components) ----
        ax = axes[0, 1]
        ax.plot(t, vx, lw=1.0, color='C0', label='vx_body')
        ax.plot(t, vy, lw=1.0, color='C1', label='vy_body')
        ax.axhline(0, color='k', lw=0.5, alpha=0.5)
        ax.grid(True, ls=':', alpha=0.6)
        ax.set_xlabel('t [s]'); ax.set_ylabel('v_body  [m/s]')
        ax.set_title(
            f'v_body   vx ∈ [{vx.min():+.2f}, {vx.max():+.2f}]   '
            f'vy ∈ [{vy.min():+.2f}, {vy.max():+.2f}] m/s')
        ax.legend(loc='upper right')

        # ---- (1,0) wz_used ----
        ax = axes[1, 0]
        ax.plot(t, wz, lw=1.0, color='C2')
        ax.axhline(0, color='k', lw=0.5, alpha=0.5)
        ax.grid(True, ls=':', alpha=0.6)
        ax.set_xlabel('t [s]'); ax.set_ylabel('wz_used  [rad/s]')
        ax.set_title(f'wz_used (yaw rate published)   '
                     f'min={wz.min():+.3f}  max={wz.max():+.3f} rad/s')

        # ---- (1,1) LS residual ----
        ax = axes[1, 1]
        ax.plot(t, res, lw=1.0, color='C3', label='residual')
        gated_pct = float('nan')
        if math.isfinite(self.slip_thr):
            ax.axhline(self.slip_thr, color='k', ls='--', lw=1.0, alpha=0.8,
                       label=f'slip_thr={self.slip_thr:.3f}')
            gated_pct = 100.0 * float((res >= self.slip_thr).sum()) / len(res)
            ax.legend(loc='upper right')
        ax.grid(True, ls=':', alpha=0.6)
        ax.set_xlabel('t [s]'); ax.set_ylabel('LS residual  [m/s]')
        title = f'LS residual   mean={res.mean():.3f}   max={res.max():.3f} m/s'
        if math.isfinite(self.slip_thr):
            title += f'   gated={gated_pct:.1f}%'
        ax.set_title(title)

        fig.suptitle(f'wheel_only_odom  —  {self.out_png.name}   '
                     f'samples={n}   dur={float(t[-1]):.1f} s',
                     fontsize=11, y=0.995)
        plt.tight_layout(rect=(0, 0, 1, 0.97))

        self.out_png.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(self.out_png, dpi=120)
        plt.close(fig)
        self.get_logger().info(
            f"saved {self.out_png}  ({n} samples, path={path:.2f} m)")


def main():
    rclpy.init(args=sys.argv)
    node = TrajectoryPlotter()

    # rclpy installs its own SIGINT handler that makes spin() return cleanly.
    # We only need to add a SIGTERM handler that triggers the same path — ros2
    # launch sends SIGTERM during shutdown, and Python's default SIGTERM
    # behaviour kills the process before we can save.
    signal.signal(signal.SIGTERM, lambda *_: rclpy.shutdown())

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.save_plot()
        except Exception as e:
            sys.stderr.write(f"[trajectory_plotter] save_plot failed: {e}\n")
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
