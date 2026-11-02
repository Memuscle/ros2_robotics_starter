"""Run the shape follower against turtlesim (ships with ROS 2) to see it move.

    ros2 launch robotics_starter shape_turtlesim.launch.py shape:=circle size:=2.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robotics_starter'), 'config', 'shape_follower.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('shape', default_value='square'),
        DeclareLaunchArgument('size', default_value='2.0'),
        Node(package='turtlesim', executable='turtlesim_node', output='screen'),
        Node(
            package='robotics_starter', executable='shape_follower', output='screen',
            parameters=[config, {
                'shape': LaunchConfiguration('shape'),
                'size': LaunchConfiguration('size'),
            }],
            remappings=[('cmd_vel', '/turtle1/cmd_vel')],
        ),
    ])
