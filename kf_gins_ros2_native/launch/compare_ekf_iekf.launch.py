from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, ExecuteProcess
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable, TextSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from datetime import datetime

def generate_launch_description():
    """
    EKF2 (PX4) vs IEKF (KF-GINS) 并行对比测试 Launcher
    
    支持功能：
    - 同时运行 PX4 EKF2 和 KF-GINS IEKF
    - 自动时间同步
    - 实时误差计算
    - 可选的 rosbag 记录
    - 多场景支持
    """
    
    # ============ 参数声明 ============
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time (required for SITL)'
    )

    kf_gins_param_file = DeclareLaunchArgument(
        'kf_gins_param_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('kf_gins_ros2_native'),
            'config',
            'kfgins_sim_fixed.yaml'
        ]),
        description='KF-GINS ROS2 param file (sim recommended)'
    )
    kf_gins_core_config_file = DeclareLaunchArgument(
        'kf_gins_core_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('kf_gins_ros2_native'),
            'external',
            'KF-GINS',
            'config',
            'kf-gins.yaml'
        ]),
        description='Original KF-GINS core YAML loaded by the adapter'
    )
    use_gnss_llh_for_pose = DeclareLaunchArgument(
        'use_gnss_llh_for_pose', default_value='false',
        description='Use GNSS LLH directly for /kf_gins/odom pose (visualization)'
    )
    
    scenario = DeclareLaunchArgument(
        'scenario', default_value='open_field',
        description='Test scenario: open_field, urban_canyon, forest, indoor'
    )
    
    record_bag = DeclareLaunchArgument(
        'record_bag', default_value='true',
        description='Record rosbag for offline analysis'
    )
    
    start_rviz = DeclareLaunchArgument(
        'start_rviz', default_value='true',
        description='Start RViz for visualization'
    )
    
    enable_real_time_comparison = DeclareLaunchArgument(
        'enable_real_time_comparison', default_value='true',
        description='Enable real-time comparison node'
    )

    publish_ekf2_state = DeclareLaunchArgument(
        'publish_ekf2_state', default_value='true',
        description='Publish EKF2 state topics for PlotJuggler'
    )
    publish_iekf_state = DeclareLaunchArgument(
        'publish_iekf_state', default_value='true',
        description='Publish IEKF state topics for PlotJuggler'
    )
    publish_aligned_iekf_state = DeclareLaunchArgument(
        'publish_aligned_iekf_state', default_value='true',
        description='Publish aligned IEKF state topics for PlotJuggler'
    )
    aligned_fallback_raw = DeclareLaunchArgument(
        'aligned_fallback_raw', default_value='false',
        description='Publish raw IEKF on /iekf/state_aligned before alignment is ready'
    )
    ekf2_use_input_stamp = DeclareLaunchArgument(
        'ekf2_use_input_stamp', default_value='false',
        description='Use MAVROS stamp for EKF2 relay (false uses node time for sync)'
    )

    imu_use_node_stamp = DeclareLaunchArgument(
        'imu_use_node_stamp', default_value='false',
        description='Use /clock stamp for IMU output (false uses MAVROS stamp)'
    )
    enable_imu_converter = DeclareLaunchArgument(
        'enable_imu_converter', default_value='false',
        description='Run imu_flu_to_frd.py only for diagnostics; KF-GINS subscribes raw IMU directly'
    )

    # ============ MAVROS topics (可覆盖) ============
    # 注意：MAVROS 的 ROS 命名空间由其 launch 参数 `namespace` 控制（默认是 `/mavros`）。
    # 终端里看到的 “UAS Prefix: /uas1” 不是 ROS topic 前缀。
    # 如果你想让 MAVROS topics 变成 `/uas1/mavros/...`，需要这样启动 MAVROS：
    #   ros2 launch mavros px4.launch namespace:=uas1/mavros ...
    mavros_imu_topic = DeclareLaunchArgument(
        'mavros_imu_topic', default_value='/mavros/imu/data_raw',
        description='MAVROS IMU topic (default: /mavros/imu/data_raw)'
    )
    imu_source = DeclareLaunchArgument(
        'imu_source', default_value='mavros_raw',
        description='IMU source for KF-GINS: mavros_raw, px4_sensor_combined, or px4_vehicle_imu'
    )
    px4_sensor_combined_topic = DeclareLaunchArgument(
        'px4_sensor_combined_topic', default_value='/fmu/out/sensor_combined',
        description='PX4 DDS SensorCombined topic'
    )
    px4_vehicle_imu_topic = DeclareLaunchArgument(
        'px4_vehicle_imu_topic', default_value='/fmu/out/vehicle_imu',
        description='PX4 DDS VehicleImu topic'
    )
    mavros_gps_topic = DeclareLaunchArgument(
        'mavros_gps_topic', default_value='/mavros/global_position/raw/fix',
        description='MAVROS GNSS topic (default: /mavros/global_position/raw/fix)'
    )
    mavros_local_pose_topic = DeclareLaunchArgument(
        'mavros_local_pose_topic', default_value='/mavros/local_position/pose',
        description='MAVROS local pose topic (default: /mavros/local_position/pose)'
    )
    mavros_local_velocity_topic = DeclareLaunchArgument(
        'mavros_local_velocity_topic', default_value='/mavros/local_position/velocity_local',
        description='MAVROS local velocity topic (default: /mavros/local_position/velocity_local)'
    )
    mavros_state_topic = DeclareLaunchArgument(
        'mavros_state_topic', default_value='/mavros/state',
        description='MAVROS state topic (default: /mavros/state)'
    )
    mavros_hil_gps_topic = DeclareLaunchArgument(
        'mavros_hil_gps_topic', default_value='/mavros/hil/gps',
        description='MAVROS HIL GPS topic (default: /mavros/hil/gps)'
    )

    # ============ GNSS Dropzones (可选) ============
    # 用于模拟 GNSS 遮挡/丢星：当飞机进入配置的盒子区域时，将 /gps/fix 发布为 NO_FIX（或 stop/nan）
    enable_gps_dropzones = DeclareLaunchArgument(
        'enable_gps_dropzones', default_value='false',
        description='Enable GNSS dropzones relay (replace gnss_relay.py)'
    )
    inject_dropzone_gps_to_px4 = DeclareLaunchArgument(
        'inject_dropzone_gps_to_px4', default_value='true',
        description='Inject dropzoned /gps/fix into PX4 via MAVROS'
    )
    px4_gps_injection_mode = DeclareLaunchArgument(
        'px4_gps_injection_mode', default_value='hil_gps',
        description='PX4 GPS injection mode when dropzones enabled: hil_gps or gps_input'
    )
    px4_set_params = DeclareLaunchArgument(
        'px4_set_params', default_value='true',
        description='Auto-set PX4 params for GPS injection (e.g. MAV_USEHILGPS)'
    )
    px4_param_service_v2 = DeclareLaunchArgument(
        'px4_param_service_v2', default_value='/mavros/param/set_v2',
        description='MAVROS ParamSetV2 service'
    )
    px4_param_service = DeclareLaunchArgument(
        'px4_param_service', default_value='/mavros/param/set',
        description='MAVROS ParamSet service (fallback)'
    )

    # 【修复】仿真 GNSS 自适应参数声明
    use_sim_gnss_std = DeclareLaunchArgument(
        'use_sim_gnss_std', default_value='true',
        description='Enable simulation-specific GNSS std (use 0.1m instead of 5.0m)'
    )
    
    sim_gnss_std_h_m = DeclareLaunchArgument(
        'sim_gnss_std_h_m', default_value='0.1',
        description='Simulation GNSS horizontal std (meters)'
    )
    
    sim_gnss_std_u_m = DeclareLaunchArgument(
        'sim_gnss_std_u_m', default_value='0.2',
        description='Simulation GNSS vertical std (meters)'
    )

    aligned_path_require_armed = DeclareLaunchArgument(
        'aligned_path_require_armed', default_value='true',
        description='Require arming before publishing IEKF aligned path'
    )

    gps_dropzones_yaml = PathJoinSubstitution([
        FindPackageShare('kf_gins_ros2_native'),
        'config',
        'gps_dropzones.yaml'
    ])
    
    # ============ 静态 TF ============
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        output='screen'
    )

    # ============ 数据预处理 ============
    imu_convert = Node(
        package='kf_gins_ros2_native',
        executable='imu_flu_to_frd.py',
        name='imu_flu_to_frd',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': LaunchConfiguration('mavros_imu_topic'),
            'output_topic': '/imu/data',
            'flip_gravity': True,
            # 默认用 MAVROS stamp；若 /clock 稳定可切换为 use_node_stamp=true
            'use_node_stamp': LaunchConfiguration('imu_use_node_stamp'),
            # 当 /clock 不前进时回退到输入时间戳，保证单调
            'fallback_to_input_stamp': True,
            'force_monotonic_stamp': True,
        }],
        condition=IfCondition(LaunchConfiguration('enable_imu_converter'))
    )

    gnss_relay = Node(
        package='kf_gins_ros2_native',
        executable='gnss_relay.py',
        name='gnss_relay',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'in_topic': LaunchConfiguration('mavros_gps_topic'),
            'out_topic': '/gps/fix'
        }],
        condition=UnlessCondition(LaunchConfiguration('enable_gps_dropzones')),
    )

    gps_dropzones = Node(
        package='kf_gins_ros2_native',
        executable='gps_relay_dropzones.py',
        name='gps_relay_dropzones',
        output='screen',
        parameters=[
            gps_dropzones_yaml,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'in_fix_topic': LaunchConfiguration('mavros_gps_topic'),
                'pose_topic': LaunchConfiguration('mavros_local_pose_topic'),
                'mavros_state_topic': LaunchConfiguration('mavros_state_topic'),
                'out_fix_topic': '/gps/fix',
                # Disarmed 时不做遮挡，避免 IEKF 在地面纯惯导发散
                'require_armed_for_drop': True,
            },
        ],
        condition=IfCondition(LaunchConfiguration('enable_gps_dropzones')),
    )

    px4_param_setter = Node(
        package='kf_gins_ros2_native',
        executable='px4_param_setter.py',
        name='px4_param_setter',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'param_ints': ['MAV_USEHILGPS=1'],
            'service_set_v2': LaunchConfiguration('px4_param_service_v2'),
            'service_set': LaunchConfiguration('px4_param_service'),
            'wait_timeout_sec': 3.0,
            'retry_count': 8,
            'retry_sleep_sec': 1.0,
            'force_set': True,
        }],
        condition=IfCondition(PythonExpression([
            "'",
            LaunchConfiguration('enable_gps_dropzones'),
            "' == 'true' and '",
            LaunchConfiguration('inject_dropzone_gps_to_px4'),
            "' == 'true' and '",
            LaunchConfiguration('px4_set_params'),
            "' == 'true' and '",
            LaunchConfiguration('px4_gps_injection_mode'),
            "' == 'hil_gps'",
        ])),
    )

    hil_gps_relay = Node(
        package='kf_gins_ros2_native',
        executable='gps_fix_to_mavros_hil_gps.py',
        name='hil_gps_relay',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'in_fix_topic': '/gps/fix',
            'out_hil_gps_topic': LaunchConfiguration('mavros_hil_gps_topic'),
            'use_input_stamp': True,
        }],
        condition=IfCondition(PythonExpression([
            "'",
            LaunchConfiguration('enable_gps_dropzones'),
            "' == 'true' and '",
            LaunchConfiguration('inject_dropzone_gps_to_px4'),
            "' == 'true' and '",
            LaunchConfiguration('px4_gps_injection_mode'),
            "' == 'hil_gps'",
        ])),
    )

    gps_input_relay = Node(
        package='kf_gins_ros2_native',
        executable='gps_fix_to_mavros_gps_input.py',
        name='gps_input_relay',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'in_fix_topic': '/gps/fix',
            'out_gps_input_topic': '/mavros/gps_input',
            'use_input_stamp': True,
        }],
        condition=IfCondition(PythonExpression([
            "'",
            LaunchConfiguration('enable_gps_dropzones'),
            "' == 'true' and '",
            LaunchConfiguration('inject_dropzone_gps_to_px4'),
            "' == 'true' and '",
            LaunchConfiguration('px4_gps_injection_mode'),
            "' == 'gps_input'",
        ])),
    )

    # ============ 轨迹发布器 ============
    # EKF2 路径转换
    ekf2_path = Node(
        package='kf_gins_ros2_native',
        executable='ekf2_path_publisher.py',
        name='ekf2_path_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # 默认 1000 点大约只有几十秒，会出现“尾迹滚动/老轨迹消失”的错觉
            'max_path_length': 20000,
        }],
    )

    iekf_path = Node(
        package='kf_gins_ros2_native',
        executable='iekf_path_publisher.py',
        name='iekf_path_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': '/kf_gins/odom',
            'output_topic': '/kf_gins/path_iekf',
            'frame_id': 'map',
            'max_path_length': 20000,
            'min_distance': 0.05,
            'require_armed': True,
            'clear_on_arm_transition': False,
            'clear_on_reset_event': True,
            'mavros_state_topic': LaunchConfiguration('mavros_state_topic'),
            'startup_ignore_sec': 3.0,
            'max_abs_position_m': 10000.0,
            'max_abs_velocity_mps': 100.0,
            'max_jump_distance_m': 50.0,
        }],
    )

    # 真值轨迹发布器
    gt_path = Node(
        package='kf_gins_ros2_native',
        executable='odom_to_path.py',
        name='odom_to_path_ground_truth',
        output='screen',
        parameters=[{
            'odom_topic': '/kf_gins/odom',  # 可改为 Gazebo 真值
            'path_topic': '/kf_gins/path_ground_truth',
            'buffer_length': 1000
        }]
    )

    # ============ PX4 EKF2 (已通过 MAVROS 发布) ============
    # 注意: EKF2 状态通过 /mavros/local_position/pose 发布
    # 我们创建一个节点来重新发布为标准格式
    ekf2_relay = Node(
        package='kf_gins_ros2_native',
        executable='ekf2_state_relay.py',  # ⭐ 新脚本，下面会创建
        name='ekf2_state_relay',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': LaunchConfiguration('mavros_local_pose_topic'),
            'velocity_topic': LaunchConfiguration('mavros_local_velocity_topic'),
            'output_topic': '/ekf2/pose',
            # 默认用 node time，确保与 IEKF 同一时间基准
            'use_input_stamp': LaunchConfiguration('ekf2_use_input_stamp'),
            'use_covariance': False,
            'prefer_native_velocity': True,
            'max_native_velocity_age_sec': 0.5,
        }]
    )

    # ============ KF-GINS IEKF ============
    kf_gins_config = LaunchConfiguration('kf_gins_param_file')
    
    kf_gins = Node(
        package='kf_gins_ros2_native',
        executable='kf_gins_node',
        name='kf_gins_node',
        output='screen',
        parameters=[
            kf_gins_config,
            {
                'imu_source': LaunchConfiguration('imu_source'),
                'imu_topic': LaunchConfiguration('mavros_imu_topic'),
                'px4_sensor_combined_topic': LaunchConfiguration('px4_sensor_combined_topic'),
                'px4_vehicle_imu_topic': LaunchConfiguration('px4_vehicle_imu_topic'),
                'gnss_topic': '/gps/fix',
                'config_file': LaunchConfiguration('kf_gins_core_config_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'use_gnss_llh_for_pose': LaunchConfiguration('use_gnss_llh_for_pose'),
                'imu_input_is_flu': True,
                # Time base robustness: prefer values from YAML (kfgins_sim_fixed.yaml)
                # 【修复】仿真环境 GNSS 协方差自适应参数
                'use_sim_gnss_std': LaunchConfiguration('use_sim_gnss_std'),
                'sim_gnss_std_h_m': LaunchConfiguration('sim_gnss_std_h_m'),
                'sim_gnss_std_u_m': LaunchConfiguration('sim_gnss_std_u_m'),
                # RViz Path gating: disarmed 时不画 Path，避免起飞前"蜘蛛网"
                'mavros_state_topic': LaunchConfiguration('mavros_state_topic'),
                'mavros_local_velocity_topic': LaunchConfiguration('mavros_local_velocity_topic'),
                'path_require_armed': True,
                'clear_path_on_arm_transition': False,
            }
        ]
    )

    # ============ 实时对比分析 ============
    real_time_comparison = Node(
        package='kf_gins_ros2_native',
        executable='real_time_comparison.py',  # ⭐ 新脚本
        name='real_time_comparison',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'ekf2_pose_topic': '/ekf2/pose',
            'ekf2_odom_topic': '/ekf2/pose_odom',
            'iekf_topic': '/kf_gins/odom',
            'comparison_output': '/comparison/metrics',
            'metrics_publish_rate': 50,  # 50 Hz
            'sync_tolerance_ms': 50,
            'align_initial': True,
            'max_abs_position_m': 10000.0,
            'max_abs_velocity_mps': 100.0,
            'max_pose_jump_m': 50.0,
            'max_velocity_jump_mps': 100.0,
            'recompute_alignment_on_jump': True,
            'clear_error_history_on_realign': True,
            'aligned_fallback_raw': LaunchConfiguration('aligned_fallback_raw'),
            'publish_iekf_state': LaunchConfiguration('publish_iekf_state'),
            'publish_ekf2_state': LaunchConfiguration('publish_ekf2_state'),
            'publish_aligned_iekf_state': LaunchConfiguration('publish_aligned_iekf_state'),
        }],
        condition=IfCondition(LaunchConfiguration('enable_real_time_comparison'))
    )

    iekf_aligned_path = Node(
        package='kf_gins_ros2_native',
        executable='iekf_aligned_path_publisher.py',
        name='iekf_aligned_path_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': '/iekf/state_aligned/position',
            'output_topic': '/iekf/path_aligned',
            'frame_id': 'map',
            'max_path_length': 20000,
            'require_armed': LaunchConfiguration('aligned_path_require_armed'),
            'clear_on_arm_transition': False,
            'clear_on_reset_event': True,
            'mavros_state_topic': LaunchConfiguration('mavros_state_topic'),
        }],
        condition=IfCondition(LaunchConfiguration('enable_real_time_comparison'))
    )

    # ============ ROS Bag 记录器 ============
    # 使用命令行工具而非 ros2bag Node (更稳定)
    bag_dir = f"/tmp/kf_gins_comparison_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    
    # rosbag 会自动创建输出目录
    record_bag_node = ExecuteProcess(
        cmd=['ros2', 'bag', 'record',
             '-o', bag_dir,
             '/clock',
             '/tf',
             '/tf_static',
             '/imu/data',
             '/gps/fix',
             '/gps_dropzones/markers',
             '/mavros/gps_input',
             '/mavros/hil/gps',
             '/ekf2/pose',
             '/ekf2/pose_odom',
             '/ekf2/path',
             '/ekf2/state/position',
             '/ekf2/state/velocity',
             '/ekf2/state/rpy',
             '/kf_gins/odom',
             '/kf_gins/path',
             '/comparison/metrics',
             '/comparison/pos_err',
             '/comparison/pos_rmse',
             '/comparison/pos_mae',
             '/comparison/vel_err',
             '/comparison/att_err',
             '/comparison/yaw_err',
             '/comparison/pos_err_xyz',
             '/comparison/vel_err_xyz',
             '/comparison/att_err_rpy',
             '/comparison/sync_dt',
             '/comparison/ekf2_initialized',
             '/comparison/iekf_initialized',
             '/iekf/state/position',
             '/iekf/state/velocity',
             '/iekf/state/rpy',
             '/iekf/state_aligned/position',
             '/iekf/state_aligned/velocity',
             '/iekf/state_aligned/rpy',
             '/iekf/path_aligned',
             '/mavros/battery'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('record_bag'))
    )

    # ============ RViz 可视化 ============
    rviz_config = PathJoinSubstitution([
        FindPackageShare('kf_gins_ros2_native'),
        'rviz',
        'ekf2_vs_iekf.rviz'
    ])
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_comparison',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_rviz'))
    )

    # ============ 日志指导 ============
    log_info = LogInfo(
        msg="=" * 80 +
        "\n🚀 EKF2 vs IEKF 对比测试已启动\n" +
        "=" * 80 +
        "\n📊 监控信息:\n" +
        "  - EKF2 输出: /ekf2/pose\n" +
        "  - IEKF 输出: /kf_gins/odom\n" +
        "  - 对比指标(误差): /comparison/metrics (Float32MultiArray, legacy)\n" +
        "  - 对比指标(命名): /comparison/pos_err /comparison/yaw_err ...\n" +
        "  - 状态(PlotJuggler): /ekf2/state/*  /iekf/state/*  /iekf/state_aligned/*\n" +
        "  - GNSS 遮挡: enable_gps_dropzones:=true (见 /gps_dropzones/markers)\n" +
        "  - GNSS 注入: px4_gps_injection_mode:=hil_gps|gps_input\n" +
        "  - PX4 参数: MAV_USEHILGPS=1 (px4_set_params:=true)\n" +
        "\n🔍 查看实时数据:\n" +
        "  rqt_plot /comparison/metrics/data[0]  # position_error_norm\n" +
        "  rqt_plot /comparison/metrics/data[1]  # attitude_error_norm\n" +
        "  rqt_plot /comparison/metrics/data[2]  # velocity_error_norm\n" +
        "  rqt_plot /comparison/pos_err  /comparison/yaw_err  /comparison/vel_err\n" +
        "\n📈 离线分析 (测试后):\n" +
        "  python3 ~/kf_gins_ws/src/kf_gins_ros2_native/scripts/offline_analysis.py\n" +
        f"\n📁 rosbag 输出目录:\n  {bag_dir}\n" +
        "\n" + "=" * 80
    )

    # ============ RQT 配置 (可选) ============
    # 自动启动数据绘图工具（可选，可通过手动启动）
    # 使用方式: rqt_plot /comparison/metrics/position_error

    # ============ 配置环境变量 (避免 RViz 显示问题) ============
    set_env = [
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        # Qt/rviz 在虚拟机或 XWayland 下可能出现“闪烁/黑条”：
        # 禁用 MIT-SHM 共享内存通常能显著改善（尤其是 VirtualBox/VMware/远程桌面环境）。
        SetEnvironmentVariable('QT_X11_NO_MITSHM', '1'),
        # 注意：强制软件渲染会让 Gazebo/RViz 极度卡顿（尤其是复杂 world）。
        # 如果你的环境必须用软件渲染再手动 export：
        #   export LIBGL_ALWAYS_SOFTWARE=1 QT_OPENGL=software
        SetEnvironmentVariable('LIBGL_DRI3_DISABLE', '1'),
        SetEnvironmentVariable('OGRE_RTT_MODE', 'Copy'),
        SetEnvironmentVariable(
            'LD_LIBRARY_PATH',
            [EnvironmentVariable('LD_LIBRARY_PATH'),
             TextSubstitution(text=':/opt/ros/humble/opt/rviz_ogre_vendor/lib')]
        ),
    ]

    # ============ 组合所有节点 ============
    return LaunchDescription([
        # 参数声明
        use_sim_time,
        kf_gins_param_file,
        kf_gins_core_config_file,
        use_gnss_llh_for_pose,
        scenario,
        record_bag,
        start_rviz,
        enable_real_time_comparison,
        publish_ekf2_state,
        publish_iekf_state,
        publish_aligned_iekf_state,
        aligned_fallback_raw,
        ekf2_use_input_stamp,
        imu_use_node_stamp,
        enable_imu_converter,
        mavros_imu_topic,
        imu_source,
        px4_sensor_combined_topic,
        px4_vehicle_imu_topic,
        mavros_gps_topic,
        mavros_local_pose_topic,
        mavros_local_velocity_topic,
        mavros_state_topic,
        mavros_hil_gps_topic,
        enable_gps_dropzones,
        inject_dropzone_gps_to_px4,
        px4_gps_injection_mode,
        px4_set_params,
        px4_param_service_v2,
        px4_param_service,
        use_sim_gnss_std,
        sim_gnss_std_h_m,
        sim_gnss_std_u_m,
        aligned_path_require_armed,
        
        # 日志输出
        log_info,
        
        # 基础设施
        static_tf,
        imu_convert,
        gnss_relay,
        gps_dropzones,
        px4_param_setter,
        hil_gps_relay,
        gps_input_relay,
        
        # 轨迹转换
        ekf2_path,
        iekf_path,
        gt_path,
        ekf2_relay,
        
        # IEKF
        kf_gins,
        
        # 对比
        real_time_comparison,
        iekf_aligned_path,
        
        # 可视化和记录
        *set_env,
        record_bag_node,
        rviz,
        # rqt_launcher_script,  # 可选，可能需要单独处理
        
        # 输出提示
        LogInfo(msg="\n✅ 所有节点已启动，请检查 RViz 和终端输出\n")
    ])
