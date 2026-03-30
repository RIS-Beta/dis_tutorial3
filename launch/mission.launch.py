from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dis_tutorial3 = get_package_share_directory('dis_tutorial3')

    sim_nav_launch = PathJoinSubstitution(
        [pkg_dis_tutorial3, 'launch', 'sim_turtlebot_nav.launch.py']
    )

    arguments = [
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('world', default_value='bird_demo1'),
        DeclareLaunchArgument('model', default_value='standard', choices=['standard', 'lite']),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([pkg_dis_tutorial3, 'maps', 'bird_demo.yaml'])
        ),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('device', default_value=''),
        DeclareLaunchArgument('mission_start_delay', default_value='8.0'),
    ]

    simulation_and_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([sim_nav_launch]),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'rviz': LaunchConfiguration('rviz'),
            'world': LaunchConfiguration('world'),
            'model': LaunchConfiguration('model'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map': LaunchConfiguration('map'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )

    detect_people = Node(
        package='dis_tutorial3',
        executable='detect_people.py',
        name='detect_people',
        output='screen',
        parameters=[{'device': LaunchConfiguration('device')}],
    )

    detect_rings = Node(
        package='dis_tutorial3',
        executable='detect_rings.py',
        name='detect_rings',
        output='screen',
        parameters=[{'device': LaunchConfiguration('device')}],
    )

    voice_commander = Node(
        package='dis_tutorial3',
        executable='voice_commander.py',
        name='voice_commander',
        output='screen',
    )

    mission_controller = Node(
        package='dis_tutorial3',
        executable='mission_controler.py',
        name='mission_controler',
        output='screen',
    )

    delayed_mission_controller = TimerAction(
        period=LaunchConfiguration('mission_start_delay'),
        actions=[mission_controller],
    )

    ld = LaunchDescription(arguments)
    ld.add_action(simulation_and_nav)
    ld.add_action(detect_people)
    ld.add_action(detect_rings)
    ld.add_action(voice_commander)
    ld.add_action(delayed_mission_controller)
    return ld