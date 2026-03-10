from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('kf_gins_ros2_native')

    entity_arg     = DeclareLaunchArgument('entity', default_value='replay_uav')
    odom_topic_arg = DeclareLaunchArgument('odom_topic', default_value='/kf_gins/odom_reliable')
    world_arg      = DeclareLaunchArgument('world', default_value='replay.world')

    env_xcb  = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')
    env_swgl = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')
    env_ogre = SetEnvironmentVariable('OGRE_RTT_MODE', 'Copy')

    world_path = PathJoinSubstitution([pkg, 'worlds', LaunchConfiguration('world')])


    gzserver = ExecuteProcess(
    cmd=[
            'gzserver', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_path,
        ],
        output='screen'
    )
    gzclient = ExecuteProcess(cmd=['gzclient'], output='screen')

    model_path = PathJoinSubstitution([pkg, 'models', 'replay_quadcopter.sdf'])

    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', LaunchConfiguration('entity'),
                   '-file', model_path, '-x', '0', '-y', '0', '-z', '0.3'],
        output='screen'
    )

    odom2gazebo = Node(
        package='kf_gins_ros2_native', executable='odom_to_gazebo.py',
        name='odom_to_gazebo', output='screen',
        parameters=[{
            'entity_name': LaunchConfiguration('entity'),
            'odom_topic':  LaunchConfiguration('odom_topic'),
            'decimate': 1, 
            'z_offset': 0.1, 
        }]
    )

    return LaunchDescription([
        env_xcb, env_swgl, env_ogre,
        entity_arg, odom_topic_arg, world_arg,
        gzserver, gzclient, spawn, odom2gazebo
    ])
