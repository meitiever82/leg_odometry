"""退化层节点 — design §6。
订阅 Isaac 干净话题,注入扰动,发布实车格式 /robot/wheel_status 与 /rslidar_imu_data。
时间戳保留 Isaac 原始戳。参数采样写 episode_params.yaml。
"""
import numpy as np
import rclpy
import yaml
from pathlib import Path
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from w2_sim.perturbation import (DEFAULT_CFG, GyroCorruptor, params_to_dict,
                                 sample_params)
from w2_sim.swerve_kinematics import WHEEL_NAMES, WHEEL_RADIUS


def joint_state_to_modules(names, positions, velocities):
    """8 关节 JointState(任意顺序)→ (转向角[4] rad, 轮速[4] m/s),按 WHEEL_NAMES 序。"""
    pos = dict(zip(names, positions))
    vel = dict(zip(names, velocities))
    angles = np.array([pos[f"{w}_steer_joint"] for w in WHEEL_NAMES])
    speeds = np.array([vel[f"{w}_wheel_joint"] for w in WHEEL_NAMES]) * WHEEL_RADIUS
    return angles, speeds


class DegradationNode(Node):
    def __init__(self):
        super().__init__("w2_sim_degradation")
        self.declare_parameter("seed", 0)
        self.declare_parameter("params_out", "/tmp/episode_params.yaml")
        for k, v in DEFAULT_CFG.items():
            self.declare_parameter(k, v)
        cfg = {k: self.get_parameter(k).value for k in DEFAULT_CFG}
        seed = self.get_parameter("seed").value
        self.p = sample_params(seed, cfg)
        self.rng = np.random.default_rng(seed + 10_000)   # 白噪声流与参数采样分离
        self.gyro = GyroCorruptor(self.p, np.random.default_rng(seed + 20_000))
        self.last_imu_t = None
        out = Path(self.get_parameter("params_out").value)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(yaml.safe_dump(params_to_dict(self.p), sort_keys=False))
        self.get_logger().info(
            f"seed={seed} kappa_theory={params_to_dict(self.p)['kappa_theory']:.5f} -> {out}")
        from w2_sim.perturbation import corrupt_wheel
        self._corrupt_wheel = corrupt_wheel
        self.pub_wheel = self.create_publisher(JointState, "/robot/wheel_status", 50)
        self.pub_imu = self.create_publisher(Imu, "/rslidar_imu_data", 200)
        self.create_subscription(JointState, "/sim/joint_states", self.on_js, 50)
        self.create_subscription(Imu, "/sim/imu", self.on_imu, 200)

    def on_js(self, msg):
        try:
            angles, speeds = joint_state_to_modules(msg.name, msg.position, msg.velocity)
        except KeyError as e:
            self.get_logger().warn(f"joint 缺失: {e}", throttle_duration_sec=5.0)
            return
        ca, cs = self._corrupt_wheel(angles, speeds, self.p, self.rng)
        out = JointState()
        out.header.stamp = msg.header.stamp        # 保留原始仿真时间戳
        out.header.frame_id = "base_link"
        out.name = list(WHEEL_NAMES)
        out.position = ca.tolist()
        out.velocity = cs.tolist()
        self.pub_wheel.publish(out)

    def on_imu(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = (t - self.last_imu_t) if self.last_imu_t is not None else 1.0 / 200
        self.last_imu_t = t
        if dt <= 0:
            dt = 1.0 / 200
        w = np.array([msg.angular_velocity.x, msg.angular_velocity.y,
                      msg.angular_velocity.z])
        a = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y,
                      msg.linear_acceleration.z])
        cw = self.gyro.step(dt, w)
        ca = self.gyro.step_accel(a)
        out = Imu()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "imu_link"            # 与实车 bag 一致(实测)
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = cw
        (out.linear_acceleration.x, out.linear_acceleration.y,
         out.linear_acceleration.z) = ca
        out.orientation_covariance[0] = -1.0         # 无姿态估计,ROS 惯例
        self.pub_imu.publish(out)


def main():
    rclpy.init()
    node = DegradationNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
