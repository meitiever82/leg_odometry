#!/usr/bin/env python3
"""腿式里程计 launch (C++ 版本): leg_odom_node + robot_state_publisher。

用法:
  ros2 launch leg_odometry leg_odometry.launch.py
  # 另一终端:
  ros2 bag play <bag_path> --clock -r 1.0
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node


def _find_pkg_dir():
    this_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.dirname(this_dir)


def _find_urdf():
    pkg_dir = _find_pkg_dir()
    candidates = [
        os.path.join(pkg_dir, '..', '..', '..', '..', 'urdf',
                     'casbot02_7dof_shell.urdf'),
        os.path.join(pkg_dir, '..', '..', '..', '..', 'glim_ros2', 'urdf',
                     'casbot02_7dof_shell.urdf'),
        '/home/steve/casbot_ws/urdf/casbot02_7dof_shell.urdf',
    ]
    for p in candidates:
        p = os.path.abspath(p)
        if os.path.exists(p):
            return p
    raise FileNotFoundError('URDF not found')


def generate_launch_description():
    urdf_path = _find_urdf()

    with open(urdf_path) as f:
        urdf_content = f.read()
    meshes_dir = os.path.abspath(
        os.path.join(os.path.dirname(urdf_path), '..', 'meshes'))
    urdf_content = urdf_content.replace(
        'filename="../meshes/',
        f'filename="file://{meshes_dir}/')

    return LaunchDescription([
        # Robot state publisher (for RViz visualization, optional)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_content}],
        ),

        # C++ leg odometry node (ESKF + GTSAM hybrid)
        Node(
            package='leg_odometry',
            executable='leg_odom_node',
            name='leg_odom_node',
            output='screen',
            parameters=[{
                'urdf_path': urdf_path,
                # ESKF noise (defaults from progress_report.md "最优参数配置")
                'accel_noise': 0.1,
                'gyro_noise': 0.01,
                'accel_bias_walk': 0.0,
                'gyro_bias_walk': 0.001,
                'foot_contact_noise': 0.002,
                'foot_swing_noise': 1.0,
                'fk_position_noise': 0.005,
                'zupt_noise': 0.03,
                'flat_z_noise': 0.001,
                'flat_vz_noise': 0.001,
                # Contact detection
                'effort_threshold': 5.0,
                'effort_hysteresis': 1.0,
                'effort_joint_left': 'LJPITCH',
                'effort_joint_right': 'RJPITCH',
                # Init / smoother
                'init_frames': 50,
                'smoother_enabled': True,
                'smoother_window': 60,
                'smoother_interval': 20,
                'smoother_kf_interval': 10,
                'smoother_alpha': 0.05,
            }],
        ),
    ])
