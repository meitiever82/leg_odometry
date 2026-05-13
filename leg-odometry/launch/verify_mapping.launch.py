'''
Author: meitiever
Date: 2026-04-09 15:21:14
LastEditors: meitiever
LastEditTime: 2026-05-13 14:45:26
Description: content
'''
#!/usr/bin/env python3
"""映射验证 launch: remapper + robot_state_publisher + RViz。

用法:
  ros2 launch leg_odometry verify_mapping.launch.py
  ros2 launch leg_odometry verify_mapping.launch.py urdf_path:=/abs/path/to.urdf
  # 另一终端:
  ros2 bag play <bag_path> --clock -r 0.5

URDF 默认从 ~/rtabmap_ws/casbot_02/urdf/casbot02_7dof_shell.urdf 读，
也可用 urdf_path:= 或 CASBOT_URDF 环境变量覆盖。
视觉 sanity check: bag 里的 LJ0/LJPITCH
这些名字到 URDF joint 的映射对不对? 方向反没反, RViz 里看机器人摆动正不正常。
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

DEFAULT_URDF = os.environ.get(
    'CASBOT_URDF',
    os.path.expanduser('~/rtabmap_ws/casbot_02/urdf/casbot02_7dof_shell.urdf'),
)

def _spawn_robot_state_publisher(context, *args, **kwargs):
    """读 URDF + 修 mesh 路径 + 起 robot_state_publisher。

    用 OpaqueFunction 是因为 robot_state_publisher 需要把 URDF 内容
    作为字符串参数传入 (而非文件路径)，必须在 launch 时打开文件读出来。
    """
    urdf_path = LaunchConfiguration('urdf_path').perform(context)
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(
            f'URDF not found: {urdf_path}\n'
            f'  use launch arg urdf_path:=... or env CASBOT_URDF=...'
        )

    with open(urdf_path) as f:
        content = f.read()

    meshes_dir = os.path.abspath(
        os.path.join(os.path.dirname(urdf_path), '..', 'meshes')
    )
    content = content.replace(
        'filename="../meshes/',
        f'filename="file://{meshes_dir}/',
    )

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': content}],
            remappings=[('/joint_states', '/joint_states_remapped')],
        ),
    ]


def generate_launch_description():
    pkg = FindPackageShare('leg_odometry')
    config_path = PathJoinSubstitution([pkg, 'config', 'joint_mapping.yaml'])
    rviz_config = PathJoinSubstitution([pkg, 'rviz', 'leg.rviz'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf_path',
            default_value=DEFAULT_URDF,
            description='Absolute path to the robot URDF',
        ),

        # Joint state remapper (rename bag joints -> URDF joints)
        Node(
            package='leg_odometry',
            executable='joint_state_remapper',
            name='joint_state_remapper',
            output='screen',
            parameters=[{'config': config_path}],
        ),

        # robot_state_publisher (loaded via OpaqueFunction so we can mesh-fix the URDF)
        OpaqueFunction(function=_spawn_robot_state_publisher),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
