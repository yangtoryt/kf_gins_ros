from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
def generate_launch_description():
    nav_file = LaunchConfiguration('nav_file')
    time_scale = LaunchConfiguration('time_scale')
    pose_decimation = LaunchConfiguration('pose_decimation')
    max_path_points = LaunchConfiguration('max_path_points')
    return LaunchDescription([
        DeclareLaunchArgument('nav_file', default_value='~/KF-GINS/result/nav_result.txt'),
        DeclareLaunchArgument('time_scale', default_value='1.0'),
        DeclareLaunchArgument('pose_decimation', default_value='10'),
        DeclareLaunchArgument('max_path_points', default_value='20000'),
        Node(package='kf_gins_ros2', executable='nav_result_publisher', name='nav_result_publisher',
            parameters=[{'nav_file': nav_file},{'time_scale': time_scale},{'broadcast_tf': True},{'publish_odom': True},{'publish_path': True},{'pose_decimation': pose_decimation},{'max_path_points': max_path_points},{'map_frame': 'map'},{'odom_frame': 'odom'},{'base_frame': 'base_link'}],
            output='screen')
    ])
