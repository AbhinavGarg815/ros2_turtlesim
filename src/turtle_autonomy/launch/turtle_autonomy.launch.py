from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    args = [
        DeclareLaunchArgument('v', default_value='1.5', description='Linear speed (m/s)'),
        DeclareLaunchArgument('dt', default_value='0.1', description='Prediction time step (s)'),
        DeclareLaunchArgument('lookahead_steps', default_value='20', description='Steps per candidate trajectory'),
        DeclareLaunchArgument('omega_max', default_value='4.0', description='Max yaw rate candidate (rad/s)'),
        DeclareLaunchArgument('omega_samples', default_value='61', description='Number of yaw rate samples'),
        DeclareLaunchArgument('goal_tolerance', default_value='0.3', description='Goal acceptance radius (m)'),
        DeclareLaunchArgument('control_rate', default_value='20.0', description='Controller frequency (Hz)'),
    ]

    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen',
    )

    odometry = Node(
        package='turtle_odometry',
        executable='turtle_odometry',
        name='turtle_odometry',
        output='screen',
    )

    autonomy = Node(
        package='turtle_autonomy',
        executable='turtle_autonomy',
        name='turtle_autonomy',
        output='screen',
        parameters=[{
            'v':               LaunchConfiguration('v'),
            'dt':              LaunchConfiguration('dt'),
            'lookahead_steps': LaunchConfiguration('lookahead_steps'),
            'omega_max':       LaunchConfiguration('omega_max'),
            'omega_samples':   LaunchConfiguration('omega_samples'),
            'goal_tolerance':  LaunchConfiguration('goal_tolerance'),
            'control_rate':    LaunchConfiguration('control_rate'),
        }],
    )

    return LaunchDescription(args + [turtlesim, odometry, autonomy])
