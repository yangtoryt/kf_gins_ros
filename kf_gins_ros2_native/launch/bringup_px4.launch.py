from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    start_rviz   = DeclareLaunchArgument('start_rviz', default_value='true')

    # KF-GINS 输入话题（经由 relay 后）
    imu_topic_arg  = DeclareLaunchArgument('imu_topic',  default_value='/imu/data')
    gnss_topic_arg = DeclareLaunchArgument('gnss_topic', default_value='/gps/fix')

    # MAVROS 原始输入（用于 relay / dropzones）
    mavros_imu_topic_arg  = DeclareLaunchArgument('mavros_imu_topic',  default_value='/mavros/imu/data_raw')
    mavros_gnss_topic_arg = DeclareLaunchArgument('mavros_gnss_topic', default_value='/mavros/global_position/raw/fix')

    enable_gps_dropzones = DeclareLaunchArgument('enable_gps_dropzones', default_value='false')

    # 找到 YAML 文件所在位置
    gps_yaml = PathJoinSubstitution([
        FindPackageShare('kf_gins_ros2_native'),
        'config',
        'gps_dropzones.yaml'
    ])

    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='world_to_map_broadcaster',
        arguments=['0','0','0','0','0','0','world','map'],
        output='screen'
    )

    imu_convert = Node(
        package='kf_gins_ros2_native', executable='imu_flu_to_frd.py', name='imu_flu_to_frd',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': LaunchConfiguration('mavros_imu_topic'),
            'output_topic': '/imu/data',
            'flip_gravity': True,
            'use_node_stamp': True
        }]
    )

    # 默认 GNSS relay：将 MAVROS GNSS 转为 /gps/fix
    gnss_relay = Node(
        package='kf_gins_ros2_native',
        executable='gnss_relay.py',
        name='gnss_relay',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'in_topic': LaunchConfiguration('mavros_gnss_topic'),
            'out_topic': LaunchConfiguration('gnss_topic'),
        }],
        condition=UnlessCondition(LaunchConfiguration('enable_gps_dropzones')),
    )

    # 创建 gps_relay_dropzones 节点：参数只传 YAML 文件
    dropzones = Node(
        package='kf_gins_ros2_native',
        executable='gps_relay_dropzones.py',
        name='gps_relay_dropzones',
        output='screen',
        parameters=[
            gps_yaml,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'in_fix_topic': LaunchConfiguration('mavros_gnss_topic'),
                'pose_topic': '/mavros/local_position/pose',
                'out_fix_topic': LaunchConfiguration('gnss_topic'),
            },
        ],
        condition=IfCondition(LaunchConfiguration('enable_gps_dropzones')),
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
             'use_sim_time': LaunchConfiguration('use_sim_time'),
             # Time base robustness:
             # - Use node time (/clock) only as an input time source
             # - Use IMU-dt integrated monotonic time for KF-GINS core
             'use_node_time_for_core': False,
             'use_integrated_time_for_core': True,
             'use_steady_time_for_imu_dt': True,
             'force_monotonic_time_for_core': True,
             'auto_reset_on_time_jump': True,
             'auto_reset_on_invalid_state': True},
            {'check_nan': True},  # Enable NaN checking
            {'nan_threshold': 1e6}  # Set NaN threshold
        ]
    )

    # —— 只在本进程会话中补齐 RViz 所需环境（不会破坏 DISPLAY）
    set_env = [
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        SetEnvironmentVariable('QT_X11_NO_MITSHM', '1'),
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
        use_sim_time,
        start_rviz,
        imu_topic_arg,
        gnss_topic_arg,
        mavros_imu_topic_arg,
        mavros_gnss_topic_arg,
        enable_gps_dropzones,
        static_tf,
        imu_convert,
        gnss_relay,
        dropzones,
        gt_path,
        kf_gins,
        # 先设置环境，再启动 rviz
        *set_env,
        rviz
    ])
