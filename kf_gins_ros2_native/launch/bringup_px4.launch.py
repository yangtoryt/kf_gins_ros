from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    start_rviz   = DeclareLaunchArgument('start_rviz', default_value='true')

    imu_topic_arg  = DeclareLaunchArgument('imu_topic',  default_value='/imu/data')
    gnss_topic_arg = DeclareLaunchArgument('gnss_topic', default_value='/mavros/global_position/raw/fix')

    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='world_to_map_broadcaster',
        arguments=['0','0','0','0','0','0','world','map'],
        output='screen'
    )

    imu_convert = Node(
        package='kf_gins_ros2_native', executable='imu_flu_to_frd.py', name='imu_flu_to_frd',
        output='screen',
        parameters=[{'input_topic':'/mavros/imu/data', 'output_topic':'/imu/data', 'flip_gravity': True}]
    )

    gnss_relay = Node(
        package='kf_gins_ros2_native', executable='gnss_relay.py', name='gnss_relay',
        output='screen',
        parameters=[{'in_topic':'/mavros/global_position/raw/fix', 'out_topic':'/gps/fix'}]
    )

    gt_path = Node(
        package='kf_gins_ros2_native', executable='odom_to_path.py', name='odom_to_path',
        output='screen',
        parameters=[{'odom_topic':'/kf_gins/odom','path_topic':'/kf_gins/path','buffer_length':1000}]
    )

    kf_gins = Node(
        package='kf_gins_ros2_native', executable='kf_gins_node', name='kf_gins_node', output='screen',
        parameters=[
            PathJoinSubstitution([FindPackageShare('kf_gins_ros2_native'),'config','kf_gins_core.yaml']),
            {'imu_topic': LaunchConfiguration('imu_topic'),
             'gnss_topic': LaunchConfiguration('gnss_topic'),
             'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # —— 只在本进程会话中补齐 RViz 所需环境（不会破坏 DISPLAY）
    set_env = [
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        SetEnvironmentVariable('QT_OPENGL', 'software'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('LIBGL_DRI3_DISABLE', '1'),
        SetEnvironmentVariable('OGRE_RTT_MODE', 'Copy'),
        # 追加而非覆盖 LD_LIBRARY_PATH
        SetEnvironmentVariable(
            'LD_LIBRARY_PATH',
            [EnvironmentVariable('LD_LIBRARY_PATH'),
             TextSubstitution(text=':/opt/ros/humble/opt/rviz_ogre_vendor/lib')]
        ),
    ]

    rviz_cfg = PathJoinSubstitution([FindPackageShare('kf_gins_ros2_native'),'rviz','kf_gins_nav_ok.rviz'])
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_rviz')),
    )

    return LaunchDescription([
        use_sim_time, start_rviz, imu_topic_arg, gnss_topic_arg,
        static_tf, imu_convert, gnss_relay, gt_path, kf_gins,
        # 先设置环境，再启动 rviz
        *set_env,
        rviz
    ])
