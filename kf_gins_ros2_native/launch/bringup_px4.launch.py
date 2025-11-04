from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg_share = FindPackageShare('kf_gins_ros2_native')
    core_yaml = PathJoinSubstitution([pkg_share, 'config', 'kf_gins_core.yaml'])
    sim_yaml  = PathJoinSubstitution([pkg_share, 'config', 'kfgins_sim.yaml'])

    # world->map 的静态 TF（避免 RViz 报 "map==map"）
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','world','map'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    imu_conv = Node(
        package='kf_gins_ros2_native',
        executable='imu_flu_to_frd.py',
        name='imu_plugin_frd',
        output='screen',
        parameters=[{'in_topic': '/imu/data_flu',
                     'out_topic': '/imu/data',
                     'flip_gravity_sign_to_frd': True,
                     'use_sim_time': use_sim_time}]
    )

    gt_path = Node(
        package='kf_gins_ros2_native',
        executable='odom_to_path.py',
        name='gt_path_node',
        output='screen',
        parameters=[{'odom_topic': '/quad1/ground_truth',  # 如无地面真值可改成 /kf_gins/odom
                     'path_topic': '/quad1/ground_truth_path',
                     'frame_id': 'map',
                     'decimation': 5,
                     'min_dist_m': 0.05,
                     'use_sim_time': use_sim_time}]
    )

    kf_gins = Node(
        package='kf_gins_ros2_native',
        executable='kf_gins_node',
        output='screen',
        parameters=[core_yaml, sim_yaml, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        static_tf,
        imu_conv,
        gt_path,
        kf_gins
    ])
