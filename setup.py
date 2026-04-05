from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'leg_odometry'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'joint_state_remapper = scripts.joint_state_remapper:main',
            'leg_odometry_node = scripts.leg_odometry_node:main',
        ],
    },
)
