"""Start the talker and the listener together."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    message = LaunchConfiguration('message')
    return LaunchDescription([
        DeclareLaunchArgument('message', default_value='Hello robotics'),
        Node(package='robotics_starter', executable='talker', output='screen',
             parameters=[{'message': message}]),
        Node(package='robotics_starter', executable='listener', output='screen'),
    ])
