from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
def generate_launch_description():
    imu_file = LaunchConfiguration('imu_file')
    gnss_file = LaunchConfiguration('gnss_file')
    rate_scale = LaunchConfiguration('rate_scale')
    return LaunchDescription([
        DeclareLaunchArgument('imu_file', default_value='~/KF-GINS/dataset/Leador-A15.txt'),
        DeclareLaunchArgument('gnss_file', default_value='~/KF-GINS/dataset/gnss.txt'),
        DeclareLaunchArgument('rate_scale', default_value='1.0'),
        Node(package='kf_gins_ros2', executable='dataset_player', name='dataset_player', parameters=[{'imu_file': imu_file},{'gnss_file': gnss_file},{'rate_scale': rate_scale},{'loop': False}], output='screen')
    ])
