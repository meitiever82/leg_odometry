#!/usr/bin/env python3
"""Launch the swerve wheel-odometry node.

Loads all parameters from config/wheel_only_params.yaml. Override the YAML
path with `params_file:=/path/to/other.yaml` if you keep per-platform variants.

State: p, R, gyro_bias.  No Kalman filter.
Pipeline: 4-wheel LS → (vx_b, vy_b, ω_z); optional gyro+Mahony for R when
enable_imu=true; LS yaw rate (default) or gyro yaw for R; FlatZ clamp on p.z.

Usage:
    ros2 launch wheel_odometry wheel_only_node.launch.py
    # or with a custom yaml:
    ros2 launch wheel_odometry wheel_only_node.launch.py \\
        params_file:=/abs/path/to/wheel_only_params.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_yaml = os.path.join(
        get_package_share_directory('wheel_odometry'),
        'config', 'wheel_only_params.yaml',
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_yaml,
            description='YAML with parameters for wheel_only_odom'),
        Node(
            package='wheel_odometry',
            executable='wheel_only_node',
            name='wheel_only_odom',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
