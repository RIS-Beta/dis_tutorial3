
from ament_index_python.packages import get_package_share_directory
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


pkg_dis_tutorial3 = get_package_share_directory('dis_tutorial3')

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='', description='Robot namespace'),
    DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('world', default_value='bird_demo1', description='Simulation World'),
    DeclareLaunchArgument('model', default_value='standard', choices=['standard', 'lite'], description='Turtlebot4 Model'),
    DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false'], description='use_sim_time'),
    DeclareLaunchArgument('map', default_value=PathJoinSubstitution([pkg_dis_tutorial3, 'maps', 'bird_demo.yaml']), description='Full path to map yaml file to load'),

	DeclareLaunchArgument(
		'people_device',
        default_value='cpu',
		description='Compute device for YOLO in detect_people.py (e.g. cpu, cuda:0)',
	),
	DeclareLaunchArgument(
		'rings_device',
		default_value='',
		description='Optional compute device parameter for detect_rings.py',
	),
	DeclareLaunchArgument(
		'startup_delay',
		default_value='10.0',
		description='Seconds to wait before launching mission nodes',
	)
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(
        DeclareLaunchArgument(
            pose_element,
            default_value='0.0',
            description=f'{pose_element} component of the robot pose',
        )
    )


def generate_launch_description():    
    # Launch Files
    gazebo_launch = PathJoinSubstitution([pkg_dis_tutorial3, 'launch', 'sim.launch.py'])
    robot_spawn_launch = PathJoinSubstitution([pkg_dis_tutorial3, 'launch', 'turtlebot4_spawn.launch.py'])
    localization_launch = PathJoinSubstitution([pkg_dis_tutorial3, 'launch', 'localization.launch.py'])
    nav2_launch = PathJoinSubstitution([pkg_dis_tutorial3, 'launch', 'nav2.launch.py'])

    #Simulator and world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world')),
            ('use_sim_time', LaunchConfiguration('use_sim_time'))
        ]
    )

    #Spawn turtlebot in the world
    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('rviz', LaunchConfiguration('rviz')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw')),
            ('use_sim_time', LaunchConfiguration('use_sim_time'))
        ]
    )

    # Localization
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([localization_launch]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('map', LaunchConfiguration('map')),
        ]
    )

    # Nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('use_sim_time', LaunchConfiguration('use_sim_time'))
        ]
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    people_device = LaunchConfiguration('people_device')
    rings_device = LaunchConfiguration('rings_device')
    startup_delay = LaunchConfiguration('startup_delay')


    detect_people = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            [
                'source /opt/ultralytics/bin/activate && '
                'ros2 run dis_tutorial3 detect_people.py '
                '--ros-args -r __node:=detect_people '
                '-p use_sim_time:=',
                use_sim_time,
                ' -p device:=',
                people_device,
            ],
        ],
        output='screen',
    )

    detect_rings = Node(
        package='dis_tutorial3',
        executable='detect_rings.py',
        name='detect_rings',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'device': rings_device,
            }
        ],
    )

    cluster_people = Node(
        package='dis_tutorial3',
        executable='cluster_people.py',
        name='cluster_people',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}],
    )

    cluster_rings = Node(
        package='dis_tutorial3',
        executable='cluster_rings.py',
        name='cluster_rings',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}],
    )

    voice_commander = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            [
                'source /home/beta/kitten_env/bin/activate',
                ' && ros2 run dis_tutorial3 voice_commander.py '
                '--ros-args -r __node:=voice_commander '
                '-p use_sim_time:=',
                use_sim_time,
            ],
        ],
        output='screen',
    )

    mission_controller = Node(
        package='dis_tutorial3',
        executable='mission_controler.py',
        name='mission_controler',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}],
    )

    mission_nodes = TimerAction(
        period=startup_delay,
        actions=[
            detect_people,
            detect_rings,
            cluster_people,
            cluster_rings,
            voice_commander,
            mission_controller,
        ],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gazebo)
    ld.add_action(robot_spawn)
    ld.add_action(localization)
    ld.add_action(nav2)
    ld.add_action(mission_nodes)
    return ld
