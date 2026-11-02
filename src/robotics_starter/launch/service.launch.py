"""Start the AddTwoInts server and call it once from the client."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('a', default_value='2'),
        DeclareLaunchArgument('b', default_value='3'),
        Node(package='robotics_starter', executable='add_service', output='screen'),
        Node(package='robotics_starter', executable='add_client', output='screen',
             parameters=[{'a': LaunchConfiguration('a'), 'b': LaunchConfiguration('b')}]),
    ])
