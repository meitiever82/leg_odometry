#!/usr/bin/env python3
"""One-click visualization to verify URDF frames vs. wheel odometry / lidar data.

Brings up:
  - robot_state_publisher  (URDF from the SolidWorks-exported xacro overlay)
  - joint_state_publisher_gui  (manual sliders, no chassis-state → JointState bridge yet)
  - rviz2 with a preset config (grid, TF, RobotModel, /rslidar_points, /wheel_odometry)
  - wheel_only_node  (optional; on by default so /wheel_odometry + odom→base TF exist)

Typical use:
    # terminal 1 — this launch
    ros2 launch wheel_odometry verify_mapping.launch.py
    # terminal 2 — feed it with a recorded bag
    ros2 bag play <bag-with-/robot/wheel_status-and-/rslidar_*>

The mesh package W2EVT1Urdf.SLDASM is *not* part of the colcon workspace, so
package:// URIs in the URDF can't be resolved by ament_index. We expand the
xacro at launch time and rewrite package:// to file:// pointing at the absolute
directory. Re-export from SolidWorks is fine — change xacro_path / mesh_pkg_root
to wherever the new export lands.
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_XACRO_PATH = (
    '/home/steve/rtabmap_ws/w2/urdf/'
    'W2EVT1Urdf_with_wheels.urdf.xacro'
)
DEFAULT_MESH_PKG_ROOT = (
    '/home/steve/rtabmap_ws/w2'
)
MESH_PKG_NAME = 'W2EVT1Urdf.SLDASM'

def _expand_urdf(xacro_path, mesh_pkg_root):
    """Run xacro, then rewrite package://<pkg>/ to file://<abs path>/."""
    out = subprocess.check_output(['xacro', xacro_path], text=True)
    return out.replace(
        f'package://{MESH_PKG_NAME}/',
        f'file://{mesh_pkg_root}/',
    )


def _spawn(context, *_):
    xacro_path    = LaunchConfiguration('xacro_path').perform(context)
    mesh_pkg_root = LaunchConfiguration('mesh_pkg_root').perform(context)
    rviz_config   = LaunchConfiguration('rviz_config').perform(context)
    use_jsp_gui   = LaunchConfiguration('use_jsp_gui').perform(context).lower() == 'true'
    run_odom      = LaunchConfiguration('run_odom').perform(context).lower() == 'true'

    robot_description = _expand_urdf(xacro_path, mesh_pkg_root)

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package=('joint_state_publisher_gui' if use_jsp_gui
                     else 'joint_state_publisher'),
            executable=('joint_state_publisher_gui' if use_jsp_gui
                        else 'joint_state_publisher'),
            name='joint_state_publisher',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ]

    if run_odom:
        odom_params = {
            'wheelbase':       float(LaunchConfiguration('wheelbase').perform(context)),
            'track':           float(LaunchConfiguration('track').perform(context)),
            'wheel_radius':    float(LaunchConfiguration('wheel_radius').perform(context)),
            'yaw_source':      LaunchConfiguration('yaw_source').perform(context),
            'publish_tf':      True,
            'odom_frame':      LaunchConfiguration('odom_frame').perform(context),
            'base_frame':      LaunchConfiguration('base_frame').perform(context),
            'odom_topic':      LaunchConfiguration('odom_topic').perform(context),
            'chassis_topic':   LaunchConfiguration('chassis_topic').perform(context),
            'imu_topic':       LaunchConfiguration('imu_topic').perform(context),
        }
        nodes.append(Node(
            package='wheel_odometry',
            executable='wheel_only_node',
            name='wheel_only_odom',
            output='screen',
            parameters=[odom_params],
        ))

    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory('wheel_odometry')
    default_rviz = os.path.join(pkg_share, 'config', 'verify_mapping.rviz')

    # Measured chassis geometry — keep in sync with the xacro file.
    return LaunchDescription([
        DeclareLaunchArgument('xacro_path',    default_value=DEFAULT_XACRO_PATH),
        DeclareLaunchArgument('mesh_pkg_root', default_value=DEFAULT_MESH_PKG_ROOT),
        DeclareLaunchArgument('rviz_config',   default_value=default_rviz),
        DeclareLaunchArgument('use_jsp_gui',   default_value='true'),
        DeclareLaunchArgument('run_odom',      default_value='true'),

        # wheel_only_node geometry (from measurements 2026-05-13: 0.435 / 0.400)
        DeclareLaunchArgument('wheelbase',     default_value='0.435'),
        DeclareLaunchArgument('track',         default_value='0.400'),
        DeclareLaunchArgument('wheel_radius',  default_value='0.121',
            description='multiplier on ChassisState.speed (rad/s → m/s)'),
        DeclareLaunchArgument('yaw_source',    default_value='gyro'),

        # Frames: publish odom → base_footprint so wheel odometry plugs into the
        # URDF tree without a bridging static transform.
        DeclareLaunchArgument('odom_frame',    default_value='odom'),
        DeclareLaunchArgument('base_frame',    default_value='base_footprint'),
        DeclareLaunchArgument('odom_topic',    default_value='/wheel_odometry'),
        DeclareLaunchArgument('chassis_topic', default_value='/robot/wheel_status'),
        DeclareLaunchArgument('imu_topic',     default_value='/rslidar_imu_data'),

        OpaqueFunction(function=_spawn),
    ])
