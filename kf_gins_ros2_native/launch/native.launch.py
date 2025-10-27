from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
def generate_launch_description():
    config = LaunchConfiguration('config_file')
    imu_topic = LaunchConfiguration('imu_topic')
    gnss_topic = LaunchConfiguration('gnss_topic')
    imu_is_delta = LaunchConfiguration('imu_is_delta')
    pose_decimation = LaunchConfiguration('pose_decimation')
    max_path_points = LaunchConfiguration('max_path_points')
    return LaunchDescription([
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('QT_X11_NO_MITSHM', '1'),
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        SetEnvironmentVariable('XDG_SESSION_TYPE', 'x11'),
        DeclareLaunchArgument('config_file', default_value=''),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('gnss_topic', default_value='/gps/fix_cov'),
        DeclareLaunchArgument('imu_is_delta', default_value='False'),
        DeclareLaunchArgument('pose_decimation', default_value='10'),
        DeclareLaunchArgument('max_path_points', default_value='20000'),
        Node(package='kf_gins_ros2_native', executable='kf_gins_node', name='kf_gins_node',
             parameters=[{'config_file': config},{'imu_topic': imu_topic},{'gnss_topic': gnss_topic},
                         {'imu_is_delta': imu_is_delta},{'pose_decimation': pose_decimation},{'max_path_points': max_path_points}], output='screen')
    ])
