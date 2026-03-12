from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    args = [
        DeclareLaunchArgument('v',                default_value='1.5'),
        DeclareLaunchArgument('dt',               default_value='0.1'),
        DeclareLaunchArgument('lookahead_steps',  default_value='20'),
        DeclareLaunchArgument('omega_max',        default_value='4.0'),
        DeclareLaunchArgument('omega_samples',    default_value='61'),
        DeclareLaunchArgument('goal_tolerance',   default_value='0.3'),
        DeclareLaunchArgument('obstacle_radius',  default_value='0.8'),
        DeclareLaunchArgument('repulsion_weight', default_value='3.0'),
        DeclareLaunchArgument('control_rate',     default_value='20.0'),
        DeclareLaunchArgument('obstacle_refresh', default_value='10.0'),
    ]

    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen',
    )

    spawn_turtles = TimerAction(period=1.0, actions=[
        Node(
            package='turtle_autonomy',
            executable='spawn_turtles',
            name='spawn_turtles',
            output='screen',
            parameters=[{'spawn_configs': ['turtle2,2.0,2.0,0.0']}],
        )
    ])

    odometry = Node(
        package='turtle_odometry',
        executable='turtle_odometry',
        name='turtle_odometry',
        output='screen',
    )

    obstacle_server = TimerAction(period=1.5, actions=[
        Node(
            package='turtle_autonomy',
            executable='obstacle_server',
            name='obstacle_server',
            output='screen',
            parameters=[{'robot_names': ['turtle1', 'turtle2']}],
        )
    ])

    autonomy = Node(
        package='turtle_autonomy',
        executable='turtle_autonomy',
        name='turtle_autonomy',
        output='screen',
        parameters=[{
            'robot_name':        'turtle1',
            'v':                 LaunchConfiguration('v'),
            'dt':                LaunchConfiguration('dt'),
            'lookahead_steps':   LaunchConfiguration('lookahead_steps'),
            'omega_max':         LaunchConfiguration('omega_max'),
            'omega_samples':     LaunchConfiguration('omega_samples'),
            'goal_tolerance':    LaunchConfiguration('goal_tolerance'),
            'obstacle_radius':   LaunchConfiguration('obstacle_radius'),
            'repulsion_weight':  LaunchConfiguration('repulsion_weight'),
            'control_rate':      LaunchConfiguration('control_rate'),
            'obstacle_refresh':  LaunchConfiguration('obstacle_refresh'),
        }],
    )

    return LaunchDescription(args + [
        turtlesim, spawn_turtles, odometry, obstacle_server, autonomy,
    ])
