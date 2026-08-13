"""Launch the minimal, fail-safe FSDS local autonomy pipeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enabled = LaunchConfiguration('enabled')
    return LaunchDescription([
        DeclareLaunchArgument(
            'enabled',
            default_value='false',
            description='Allow the controller to move the vehicle'),
        Node(
            package='autonomy',
            executable='cone_detector',
            output='screen'),
        Node(
            package='autonomy',
            executable='local_planner',
            output='screen'),
        Node(
            package='autonomy',
            executable='pure_pursuit',
            output='screen',
            parameters=[{'enabled': enabled}]),
    ])
