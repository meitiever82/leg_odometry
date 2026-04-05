#!/usr/bin/env python3
"""映射验证 launch: remapper + robot_state_publisher。

用法:
  ros2 launch leg_odometry verify_mapping.launch.py
  # 另一终端:
  ros2 bag play <bag_path> --clock -r 0.5
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find_pkg_dir():
    """找到包的源码目录。"""
    # 从 launch 文件位置往上找
    this_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.dirname(this_dir)  # leg_odometry/
    return pkg_dir


def _find_urdf():
    """找到 URDF 文件。"""
    # 硬编码源码路径 + 相对路径备选
    candidates = [
        '/home/steve/casbot_ws/src/finder_lidar_mapping/glim_ros2/urdf/casbot02_7dof_shell.urdf',
    ]
    # 也从 launch 文件位置相对查找
    pkg_dir = _find_pkg_dir()
    for rel in [
        os.path.join(pkg_dir, '..', '..', '..', '..', 'glim_ros2', 'urdf', 'casbot02_7dof_shell.urdf'),
        os.path.join(pkg_dir, '..', 'finder_lidar_mapping', 'glim_ros2', 'urdf', 'casbot02_7dof_shell.urdf'),
    ]:
        candidates.append(os.path.abspath(rel))

    for p in candidates:
        if os.path.exists(p):
            return p
    raise FileNotFoundError(f'URDF not found, tried: {candidates}')


def generate_launch_description():
    pkg_dir = _find_pkg_dir()
    config_path = os.path.join(pkg_dir, 'config', 'joint_mapping.yaml')
    if not os.path.exists(config_path):
        # symlink-install 时，config 在源码目录
        config_path = '/home/steve/casbot_ws/src/leg_odometry/config/joint_mapping.yaml'
    urdf_path = _find_urdf()

    # 读取 URDF 并修复 mesh 路径
    with open(urdf_path) as f:
        urdf_content = f.read()
    meshes_dir = os.path.abspath(os.path.join(os.path.dirname(urdf_path),
                                              '..', 'meshes'))
    urdf_content = urdf_content.replace(
        'filename="../meshes/',
        f'filename="file://{meshes_dir}/')

    return LaunchDescription([
        # Joint state remapper
        Node(
            package='leg_odometry',
            executable='joint_state_remapper',
            name='joint_state_remapper',
            output='screen',
            parameters=[{'config': config_path}],
        ),

        # Robot state publisher (读 remapped joint states)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_content}],
            remappings=[('/joint_states', '/joint_states_remapped')],
        ),
    ])
