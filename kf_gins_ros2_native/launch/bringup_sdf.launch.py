from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    sdf_path = LaunchConfiguration('sdf_path', default='/home/yang/.gazebo/models/simple_quadrotor/model.sdf')

    pkg_share = FindPackageShare('kf_gins_ros2_native')
    core_yaml = PathJoinSubstitution([pkg_share, 'config', 'kf_gins_core.yaml'])
    sim_yaml  = PathJoinSubstitution([pkg_share, 'config', 'kfgins_sim.yaml'])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('gazebo_ros'), '/launch/gazebo.launch.py'])
    )

    spawn = Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
                 arguments=['-entity', 'quad1', '-file', sdf_path])

    # FLU->FRD 转换（无任何“翻转重力”的处理）
    imu_conv = Node(
        package='kf_gins_ros2_native', executable='imu_flu_to_frd.py',
        name='imu_plugin_frd', output='screen',
        parameters=[{'in_topic': '/imu/data_flu',
                     'out_topic': '/imu/data',
                     'use_sim_time': use_sim_time}]
    )

    # 真值 Path（frame=map）
    gt_path = Node(
        package='kf_gins_ros2_native', executable='odom_to_path.py',
        name='gt_path_node', output='screen',
        parameters=[{'odom_topic': '/quad1/ground_truth',
                     'path_topic': '/quad1/ground_truth_path',
                     'frame_id': 'map', 'decimation': 5, 'min_dist_m': 0.05,
                     'use_sim_time': use_sim_time}]
    )

    kf_gins = Node(
        package='kf_gins_ros2_native', executable='kf_gins_node', output='screen',
        parameters=[core_yaml, sim_yaml, {'use_sim_time': use_sim_time}],
    )

    # world->map 静态 TF（避免 RViz 的 world 消息被丢弃）
    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    )

    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('rviz2'), 'default.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}], output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('sdf_path', default_value=sdf_path),
        gazebo_launch,
        spawn,
        static_tf,
        imu_conv,
        gt_path,
        kf_gins,
        rviz
    ])
