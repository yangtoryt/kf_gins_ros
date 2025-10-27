from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    nav_file = LaunchConfiguration('nav_file')
    time_scale = LaunchConfiguration('time_scale')
    pose_decimation = LaunchConfiguration('pose_decimation')
    max_path_points = LaunchConfiguration('max_path_points')
    rviz_path = os.path.join(get_package_share_directory('kf_gins_ros2'), 'rviz', 'kf_gins_nav.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('nav_file', default_value='~/KF-GINS/result/nav_result.txt'),
        DeclareLaunchArgument('time_scale', default_value='1.0'),
        DeclareLaunchArgument('pose_decimation', default_value='10'),
        DeclareLaunchArgument('max_path_points', default_value='20000'),
        Node(package='kf_gins_ros2', executable='nav_result_publisher', name='nav_result_publisher',
            parameters=[{'nav_file': nav_file},{'time_scale': time_scale},{'broadcast_tf': True},{'publish_odom': True},{'publish_path': True},{'pose_decimation': pose_decimation},{'max_path_points': max_path_points},{'map_frame': 'map'},{'odom_frame': 'odom'},{'base_frame': 'base_link'}],
            output='screen'),
        Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_path], output='screen')
    ])
