#!/usr/bin/env python3
"""Launch the swerve wheel-odometry node + rviz + auto trajectory plotter.

Layout:
- `wheel_only_odom` (C++) — reads /robot/wheel_status, publishes /wheel_odometry
- `rviz2` (default ON) — live visualization, uses config/wheel_only.rviz
- `trajectory_plotter` (default ON) — Python node subscribed to /wheel_odometry,
  accumulates the pose in-memory and writes a PNG when shut down (Ctrl+C from
  this launch). No intermediate CSV.

The plotter reads `slip_threshold` out of the same YAML to draw the threshold
line on the residual subplot — no duplicate-of-truth.

Usage:
    ros2 launch wheel_odometry wheel_only_node.launch.py
    ros2 launch wheel_odometry wheel_only_node.launch.py use_rviz:=false
    ros2 launch wheel_odometry wheel_only_node.launch.py auto_plot:=false
    ros2 launch wheel_odometry wheel_only_node.launch.py \\
        trajectory_png:=/tmp/my_run.png
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _read_slip_threshold(yaml_path: str) -> float:
    """Pull `wheel_only_odom.ros__parameters.slip_threshold` from yaml.

    Returns NaN if missing/unreadable — the plotter treats NaN as "no line".
    """
    try:
        import yaml
        with open(yaml_path) as f:
            cfg = yaml.safe_load(f) or {}
        return float(cfg.get('wheel_only_odom', {})
                     .get('ros__parameters', {})
                     .get('slip_threshold', float('nan')))
    except Exception:
        return float('nan')


def _spawn(context, *_):
    params_file = LaunchConfiguration('params_file').perform(context)
    slip_thr = _read_slip_threshold(params_file)
    # When playing a bag with `ros2 bag play --clock`, set use_sim_time:=true so
    # the node, rviz AND the plotter all key off /clock. If rviz keeps wall-clock
    # time while the data carries bag (sim) stamps, its TF and Odometry displays
    # drift apart in time — TF (fed by the TransformListener thread) looks current
    # while the Odometry display lags. Same clock everywhere = no lag.
    sim = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    return [
        Node(
            package='wheel_odometry',
            executable='wheel_only_node',
            name='wheel_only_odom',
            output='screen',
            parameters=[params_file, {'use_sim_time': sim}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': sim}],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
        Node(
            package='wheel_odometry',
            executable='trajectory_plotter.py',
            name='trajectory_plotter',
            output='screen',
            parameters=[{
                'odom_topic':     LaunchConfiguration('odom_topic'),
                'out_png':        LaunchConfiguration('trajectory_png'),
                'slip_threshold': slip_thr,
                'use_sim_time':   sim,
            }],
            condition=IfCondition(LaunchConfiguration('auto_plot')),
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('wheel_odometry')
    default_yaml = os.path.join(pkg_share, 'config', 'wheel_only_params.yaml')
    default_rviz = os.path.join(pkg_share, 'config', 'wheel_only.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_yaml,
            description='YAML with parameters for wheel_only_odom + plotter'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Set true when replaying a bag with `ros2 bag play --clock`'),
        DeclareLaunchArgument(
            'rviz_config', default_value=default_rviz),
        DeclareLaunchArgument(
            'auto_plot', default_value='true',
            description='Spawn trajectory_plotter; saves PNG on shutdown'),
        DeclareLaunchArgument(
            'odom_topic', default_value='/wheel_odometry'),
        DeclareLaunchArgument(
            'trajectory_png', default_value='/tmp/wheel_odom_trajectory.png'),
        OpaqueFunction(function=_spawn),
    ])
