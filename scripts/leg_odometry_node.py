#!/usr/bin/env python3
"""腿式里程计 ROS2 节点。

订阅 /joint_states 和 /imu，运行 Bloesch EKF，发布 /leg_odometry。
"""

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from leg_odometry.ekf import BloeSchEKF
from leg_odometry.kinematics import LegKinematics
from leg_odometry.contact_detector import ContactDetector


class LegOdometryNode(Node):
    def __init__(self):
        super().__init__('leg_odometry_node')

        # 加载参数
        mapping_path = self.declare_parameter('mapping_config', '').value
        ekf_path = self.declare_parameter('ekf_config', '').value

        if not mapping_path or not ekf_path:
            self.get_logger().fatal(
                'Must provide mapping_config and ekf_config parameters')
            raise SystemExit(1)

        with open(mapping_path) as f:
            mapping_cfg = yaml.safe_load(f)
        with open(ekf_path) as f:
            ekf_cfg = yaml.safe_load(f)

        # 关节映射
        self.joint_mapping = mapping_cfg.get('joint_mapping', {})
        self.ignored = set(mapping_cfg.get('ignored_joints', []))
        self.signs = mapping_cfg.get('joint_sign', {})
        self.offsets = mapping_cfg.get('joint_offset', {})

        # 接触检测参数
        contact_cfg = ekf_cfg.get('contact', {})
        self.effort_joint_left = contact_cfg.get('effort_joint_left', 'LJPITCH')
        self.effort_joint_right = contact_cfg.get('effort_joint_right', 'RJPITCH')

        # 运动学
        kin_cfg = ekf_cfg.get('kinematics', {})
        urdf_path = kin_cfg.get('urdf_path', '')
        if urdf_path:
            with open(urdf_path) as f:
                urdf_str = f.read()
        else:
            # 从 robot_description 参数获取
            self.declare_parameter('robot_description', '')
            urdf_str = self.get_parameter('robot_description').value
            if not urdf_str:
                self.get_logger().fatal(
                    'No URDF: set urdf_path in config or robot_description parameter')
                raise SystemExit(1)

        self.kinematics = LegKinematics(
            urdf_str,
            base_link=kin_cfg.get('base_link', 'base_link'),
            left_foot_link=kin_cfg.get('left_foot_link', 'left_leg_ankle_roll_link'),
            right_foot_link=kin_cfg.get('right_foot_link', 'right_leg_ankle_roll_link'),
        )
        self.get_logger().info(
            f'Left leg chain joints: {self.kinematics.left_joint_names}')
        self.get_logger().info(
            f'Right leg chain joints: {self.kinematics.right_joint_names}')

        # EKF
        self.ekf = BloeSchEKF(ekf_cfg.get('ekf', {}))

        # 接触检测
        self.contact = ContactDetector(
            threshold=contact_cfg.get('effort_threshold', 5.0),
            hysteresis=contact_cfg.get('hysteresis', 1.0),
        )

        # 最新关节状态缓存
        self._latest_joints = {}        # {urdf_name: position}
        self._latest_efforts = {}       # {bag_name: effort}
        self._joint_stamp = None

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # 订阅
        self.create_subscription(
            Imu, '/imu', self.imu_callback, sensor_qos)
        self.create_subscription(
            JointState, '/joint_states', self.joint_callback, sensor_qos)

        # 发布
        self.odom_pub = self.create_publisher(Odometry, '/leg_odometry', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self._last_imu_stamp = None
        self._init_count = 0
        self._init_accel_sum = np.zeros(3)
        self._INIT_FRAMES = 50  # 用前 50 帧 IMU 估计初始重力方向

        self.get_logger().info('Leg odometry node started')

    def joint_callback(self, msg: JointState):
        """缓存最新关节状态。"""
        for i, name in enumerate(msg.name):
            if name in self.ignored:
                continue
            # 存原始 bag 名的 effort（用于接触检测）
            if len(msg.effort) > i:
                self._latest_efforts[name] = msg.effort[i]

            # 映射到 URDF 名存位置
            urdf_name = self.joint_mapping.get(name)
            if urdf_name is None:
                continue
            sign = self.signs.get(name, 1.0)
            offset = self.offsets.get(name, 0.0)
            if len(msg.position) > i:
                self._latest_joints[urdf_name] = msg.position[i] * sign + offset

        self._joint_stamp = msg.header.stamp

    def imu_callback(self, msg: Imu):
        """IMU 驱动 EKF 预测 + 运动学更新。"""
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])
        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ])
        stamp = msg.header.stamp

        # 初始化阶段：收集 IMU 数据估计重力方向
        if not self.ekf.initialized:
            self._init_accel_sum += accel
            self._init_count += 1
            if self._init_count >= self._INIT_FRAMES and self._latest_joints:
                avg_accel = self._init_accel_sum / self._init_count
                p_fl = self.kinematics.fk_left(self._latest_joints)
                p_fr = self.kinematics.fk_right(self._latest_joints)
                self.ekf.initialize(avg_accel, p_fl, p_fr)
                self.get_logger().info(
                    f'EKF initialized with gravity estimate from {self._init_count} IMU frames')
            return

        # 计算 dt
        if self._last_imu_stamp is None:
            self._last_imu_stamp = stamp
            return
        dt = (stamp.sec - self._last_imu_stamp.sec) + \
             (stamp.nanosec - self._last_imu_stamp.nanosec) * 1e-9
        self._last_imu_stamp = stamp

        if dt <= 0 or dt > 0.1:
            return

        # 接触检测
        effort_l = self._latest_efforts.get(self.effort_joint_left, 0.0)
        effort_r = self._latest_efforts.get(self.effort_joint_right, 0.0)
        contact_l, contact_r = self.contact.update(effort_l, effort_r)

        # EKF 预测
        self.ekf.predict(accel, gyro, dt, contact_l, contact_r)

        # EKF 更新（需要关节数据）
        if self._latest_joints:
            p_fl = self.kinematics.fk_left(self._latest_joints)
            p_fr = self.kinematics.fk_right(self._latest_joints)
            self.ekf.update(p_fl, p_fr, contact_l, contact_r)

        # 发布
        self._publish_odometry(stamp, contact_l, contact_r)

    def _publish_odometry(self, stamp, contact_l: bool, contact_r: bool):
        pos, quat = self.ekf.get_pose()
        vel = self.ekf.get_velocity()

        # Odometry
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = pos[0]
        odom.pose.pose.position.y = pos[1]
        odom.pose.pose.position.z = pos[2]
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        odom.twist.twist.linear.x = vel[0]
        odom.twist.twist.linear.y = vel[1]
        odom.twist.twist.linear.z = vel[2]

        self.odom_pub.publish(odom)

        # TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link_leg_odom'
        t.transform.translation.x = pos[0]
        t.transform.translation.y = pos[1]
        t.transform.translation.z = pos[2]
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = LegOdometryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
