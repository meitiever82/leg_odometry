from setuptools import find_packages, setup

package_name = 'w2_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/perturbation_default.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='steve',
    maintainer_email='guoruonan@ztpilot.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'degradation_node = w2_sim.degradation_node:main',
            'lidar_pc2_publisher = w2_sim.lidar_pc2_publisher:main',
        ],
    },
)
