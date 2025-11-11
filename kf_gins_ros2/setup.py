from setuptools import setup
from glob import glob
import os
package_name = 'kf_gins_ros2'
setup(
    name=package_name, version='0.1.3',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/rviz', ['rviz/kf_gins_nav.rviz']),
        ('share/' + package_name + '/config', ['config/frames.yaml'])
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='KF-GINS ROS2', maintainer_email='you@example.com',
    description='ROS2 Foxy wrapper to visualize KF-GINS results with TF/Odometry/Path; includes dataset player.',
    license='GPL-3.0-or-later', tests_require=['pytest'],
    entry_points={'console_scripts': [
        'nav_result_publisher = kf_gins_ros2.nav_result_publisher:main',
        'dataset_player = kf_gins_ros2.dataset_player:main',
        'error_monitor = kf_gins_ros2.error_monitor:main',
        'odom_qos_bridge = kf_gins_ros2.odom_qos_bridge:main',
    ]},
)
