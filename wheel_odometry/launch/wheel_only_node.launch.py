#!/usr/bin/env python3
"""Launch the swerve wheel-odometry node, optionally with rviz2 attached.

All algorithm parameters come from config/wheel_only_params.yaml (override
the YAML path with `params_file:=/path/to/other.yaml`). rviz2 spawns by
default with config/wheel_only.rviz so you can watch the odom→base TF and
the /wheel_odometry trail live; set `use_rviz:=false` to skip it.

Usage:
    ros2 launch wheel_odometry wheel_only_node.launch.py
    ros2 launch wheel_odometry wheel_only_node.launch.py use_rviz:=false
    ros2 launch wheel_odometry wheel_only_node.launch.py \\
        params_file:=/abs/path/to/wheel_only_params.yaml \\
        rviz_config:=/abs/path/to/custom.rviz
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('wheel_odometry')
    default_yaml = os.path.join(pkg_share, 'config', 'wheel_only_params.yaml')
    default_rviz = os.path.join(pkg_share, 'config', 'wheel_only.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_yaml,
            description='YAML with parameters for wheel_only_odom'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Spawn rviz2 alongside the node (true / false)'),
        DeclareLaunchArgument(
            'rviz_config', default_value=default_rviz,
            description='rviz2 config path'),

        Node(
            package='wheel_odometry',
            executable='wheel_only_node',
            name='wheel_only_odom',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
