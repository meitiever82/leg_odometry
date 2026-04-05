#!/usr/bin/env python3
"""腿式里程计 launch: remapper + robot_state_publisher + EKF 节点。

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
    urdf_candidates = [
        os.path.join(pkg_dir, '..', '..', '..', '..', 'glim_ros2', 'urdf',
                     'casbot02_7dof_shell.urdf'),
    ]
    for p in urdf_candidates:
        p = os.path.abspath(p)
        if os.path.exists(p):
            return p
    raise FileNotFoundError('URDF not found')


def generate_launch_description():
    pkg_dir = _find_pkg_dir()
    mapping_config = os.path.join(pkg_dir, 'config', 'joint_mapping.yaml')
    ekf_config = os.path.join(pkg_dir, 'config', 'ekf_params.yaml')
    urdf_path = _find_urdf()

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
            parameters=[{'config': mapping_config}],
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_content}],
            remappings=[('/joint_states', '/joint_states_remapped')],
        ),

        # Leg odometry EKF
        Node(
            package='leg_odometry',
            executable='leg_odometry_node',
            name='leg_odometry_node',
            output='screen',
            parameters=[{
                'mapping_config': mapping_config,
                'ekf_config': ekf_config,
                'robot_description': urdf_content,
            }],
        ),
    ])
