#!/usr/bin/env python3
"""读取原始 /joint_states，按映射表转换关节名后重新发布。

用途：
  1. Isaac Sim / RViz 可视化验证关节映射是否正确
  2. 为 robot_state_publisher 提供正确的关节名

用法：
  ros2 run leg_odometry joint_state_remapper --ros-args -p config:=<path/to/joint_mapping.yaml>
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml


class JointStateRemapper(Node):
    def __init__(self):
        super().__init__('joint_state_remapper')

        config_path = self.declare_parameter('config', '').value
        if not config_path:
            self.get_logger().fatal('Must provide config parameter (joint_mapping.yaml path)')
            raise SystemExit(1)

        with open(config_path) as f:
            cfg = yaml.safe_load(f)

        self.mapping = cfg.get('joint_mapping', {})
        self.ignored = set(cfg.get('ignored_joints', []))
        self.signs = cfg.get('joint_sign', {})
        self.offsets = cfg.get('joint_offset', {})

        self.get_logger().info(f'Loaded {len(self.mapping)} joint mappings, ignoring {len(self.ignored)} joints')

        self.sub = self.create_subscription(
            JointState, '/joint_states', self.callback, 10)
        self.pub = self.create_publisher(
            JointState, '/joint_states_remapped', 10)

        self._warned = set()

    def callback(self, msg: JointState):
        out = JointState()
        out.header = msg.header

        for i, name in enumerate(msg.name):
            if name in self.ignored:
                continue

            urdf_name = self.mapping.get(name)
            if urdf_name is None:
                # 手指等未映射关节，静默跳过（只警告一次）
                if name not in self._warned:
                    self.get_logger().debug(f'No mapping for joint: {name}, skipping')
                    self._warned.add(name)
                continue

            sign = self.signs.get(name, 1.0)
            offset = self.offsets.get(name, 0.0)

            out.name.append(urdf_name)
            out.position.append(msg.position[i] * sign + offset)
            if len(msg.velocity) > i:
                out.velocity.append(msg.velocity[i] * sign)
            if len(msg.effort) > i:
                out.effort.append(msg.effort[i] * sign)

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateRemapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
