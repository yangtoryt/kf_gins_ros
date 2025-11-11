# kf_gins_ros2/launch/offline_demo.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def _common_env(extra=None):
    base = {
        'DISPLAY': EnvironmentVariable('DISPLAY', default_value=':0'),
        'HOME': EnvironmentVariable('HOME', default_value='/home/yang'),
        'ROS_HOME': EnvironmentVariable('ROS_HOME', default_value='/home/yang/.ros'),
        'XDG_RUNTIME_DIR': EnvironmentVariable('XDG_RUNTIME_DIR', default_value='/run/user/1000'),
        'QT_QPA_PLATFORM': 'xcb',
        'AMENT_PREFIX_PATH': EnvironmentVariable('AMENT_PREFIX_PATH', default_value='/opt/ros/humble:/home/yang/kf_gins_ws/install'),
        'CMAKE_PREFIX_PATH': EnvironmentVariable('CMAKE_PREFIX_PATH', default_value='/opt/ros/humble:/home/yang/kf_gins_ws/install'),
        'LD_LIBRARY_PATH': [
            EnvironmentVariable('LD_LIBRARY_PATH'),
            ':/opt/ros/humble/opt/rviz_ogre_vendor/lib',
            ':/opt/ros/humble/lib'
        ],
    }
    if extra:
        base.update(extra)
    return base

def _rviz_env():
    # 这组在虚拟机里最稳（滚轮/快捷键/渲染都兜底）
    return _common_env({
        'QT_IM_MODULE': 'xim',
        'XMODIFIERS': '@im=none',
        'GTK_IM_MODULE': 'xim',
        'LIBGL_ALWAYS_SOFTWARE': '1',
        'LIBGL_DRI3_DISABLE': '1',
        'MESA_LOADER_DRIVER_OVERRIDE': 'llvmpipe',
        'MESA_GL_VERSION_OVERRIDE': '3.3',
        'MESA_GLSL_VERSION_OVERRIDE': '330',
        'OGRE_RTT_MODE': 'Copy',
    })

def generate_launch_description():
    imu_file   = DeclareLaunchArgument('imu_file', default_value=PathJoinSubstitution([
        FindPackageShare('kf_gins_ros2_native'), 'external', 'KF-GINS', 'dataset', 'Leador-A15.txt'
    ]))
    gnss_file  = DeclareLaunchArgument('gnss_file', default_value=PathJoinSubstitution([
        FindPackageShare('kf_gins_ros2_native'), 'external', 'KF-GINS', 'dataset', 'GNSS-RTK.txt'
    ]))
    truth_nav  = DeclareLaunchArgument('truth_nav', default_value=PathJoinSubstitution([
        FindPackageShare('kf_gins_ros2_native'), 'external', 'KF-GINS', 'dataset', 'truth.nav'
    ]))
    rate_scale = DeclareLaunchArgument('rate_scale', default_value='1.0')
    start_rviz = DeclareLaunchArgument('start_rviz', default_value='true')
    start_plot = DeclareLaunchArgument('start_plot', default_value='true')

    player = Node(
        package='kf_gins_ros2', executable='dataset_player', name='dataset_player', output='screen',
        parameters=[{
            'imu_file':   LaunchConfiguration('imu_file'),
            'gnss_file':  LaunchConfiguration('gnss_file'),
            'rate_scale': LaunchConfiguration('rate_scale')
        }]
    )

    kf_node = Node(
        package='kf_gins_ros2_native', executable='kf_gins_node', name='kf_gins_node', output='screen',
        parameters=[PathJoinSubstitution([FindPackageShare('kf_gins_ros2_native'),'config','kf_gins_core.yaml'])]
    )

    qos_bridge = Node(
        package='kf_gins_ros2', executable='odom_qos_bridge', name='kf_odom_qos_bridge', output='screen',
        parameters=[{'input_odom': '/kf_gins/odom', 'output_odom': '/kf_gins/odom_reliable'}]
    )

    truth = Node(
        package='kf_gins_ros2', executable='nav_result_publisher', name='kf_nav_result_publisher', output='screen',
        parameters=[{
            'nav_file': LaunchConfiguration('truth_nav'),
            'frames': {'map':'map','odom':'odom','base_link':'base_link'},
            'decimation': 10, 'max_path_points': 20000
        }],
        remappings=[('/kf_gins/odom','/truth/odom'), ('/kf_gins/path','/truth/path')]
    )

    # 用 ros_arguments 强制参数覆盖，确保不再退回默认值
    err = Node(
        package='kf_gins_ros2', executable='error_monitor', name='kf_gins_error_monitor', output='screen',
        arguments=[
            '--ros-args',
            '-p', 'odom_topic:=/kf_gins/odom_reliable',
            '-p', 'truth_topic:=/truth/odom'
        ]
    )

    odom2path = Node(
        package='kf_gins_ros2_native', executable='odom_to_path.py', name='kf_odom_to_path', output='screen',
        parameters=[{'odom_topic': '/kf_gins/odom_reliable', 'path_topic': '/kf_gins/path'}]
    )

    rviz_cfg = PathJoinSubstitution([FindPackageShare('kf_gins_ros2'),'rviz','kf_gins_nav.rviz'])
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_cfg], output='screen',
        condition=IfCondition(LaunchConfiguration('start_rviz')),
        env=_rviz_env()
    )

    plot = Node(
        package='plotjuggler', executable='plotjuggler', name='plotjuggler',
        condition=IfCondition(LaunchConfiguration('start_plot')),
        output='screen',
        env=_common_env({
            'PLOTJUGGLER_PLUGIN_PATH': '/opt/ros/humble/lib/plotjuggler_ros:/opt/ros/humble/lib/plotjuggler',
        })
    )

    return LaunchDescription([
        imu_file, gnss_file, truth_nav, rate_scale, start_rviz, start_plot,
        player, kf_node, qos_bridge, truth, err, odom2path, rviz, plot
    ])
