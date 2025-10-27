from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    config_file  = LaunchConfiguration('config_file')
    imu_topic    = LaunchConfiguration('imu_topic',   default='/imu/data')
    gnss_topic   = LaunchConfiguration('gnss_topic',  default='/gps/fix')
    # 顶部：声明 LaunchConfiguration
    # ★ 新增：把你想从命令行覆盖的参数都声明出来
    pose_decimation       = LaunchConfiguration('pose_decimation', default='10')
    min_dist_m            = LaunchConfiguration('min_dist_m',      default='0.30')
    path_publish_rate_hz  = LaunchConfiguration('path_publish_rate_hz', default='5.0')
    use_gnss_llh_for_pose = LaunchConfiguration('use_gnss_llh_for_pose', default='false')
    min_speed_mps         = LaunchConfiguration('min_speed_mps', default='0.30')  # 若你在代码里加了这个


    # Gazebo（Classic）官方启动
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
        'verbose': 'true',
        'world': os.path.expanduser('~/.gazebo/worlds/empty_enu.world')
    }.items()
    )


    # 从本地 SDF 直接生成一个实体
    sdf_path = os.path.expanduser('~/.gazebo/models/simple_quadrotor/model.sdf')
    spawn_quad = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_quad',
        output='screen',
        arguments=['-entity', 'quad1', '-file', sdf_path, '-x', '0', '-y', '0', '-z', '1.5'],
        
    )

    # KF-GINS 原生节点
    kfgins_node = Node(
        package='kf_gins_ros2_native',
        executable='kf_gins_node',
        name='kf_gins_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'imu_topic': imu_topic,
            'gnss_topic': gnss_topic,
            'imu_is_delta': False,
            # 'pose_decimation': 20,
            'max_path_points': 20000,
            'config_file': config_file,
            'imu_linear_no_gravity': False, 
            
            'pose_decimation':      ParameterValue(pose_decimation, value_type=int),
            'min_dist_m':           ParameterValue(min_dist_m, value_type=float),
            'path_publish_rate_hz': ParameterValue(path_publish_rate_hz, value_type=float),
            'use_gnss_llh_for_pose':ParameterValue(use_gnss_llh_for_pose, value_type=bool),
            'min_speed_mps':        ParameterValue(min_speed_mps, value_type=float),
        }]
    )

    # RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.expanduser('~/.gazebo/models')),

        DeclareLaunchArgument('config_file', description='KF-GINS yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('gnss_topic', default_value='/gps/fix'),

        # ★ 新增四/五个
        DeclareLaunchArgument('pose_decimation', default_value='10'),
        DeclareLaunchArgument('min_dist_m', default_value='0.30'),
        DeclareLaunchArgument('path_publish_rate_hz', default_value='5.0'),
        DeclareLaunchArgument('use_gnss_llh_for_pose', default_value='false'),
        DeclareLaunchArgument('min_speed_mps', default_value='0.30'),

        gazebo_launch,
        spawn_quad,
        kfgins_node,
        rviz2
    ])
