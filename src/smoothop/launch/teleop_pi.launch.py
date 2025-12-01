#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the config file
    config_file = os.path.join(
        get_package_share_directory('smoothop'),
        'config',
        'controller.config.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[config_file],
            arguments=['--ros-args', '--log-level', 'WARN']  # Add this line
        ),
        Node(
            package='smoothop',
            executable='joy_turbo_filter',
            name='joy_turbo_filter',
            parameters=[config_file],
            output='screen'
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[config_file],
            remappings=[('joy', 'joy_filtered')]
        )
    ])
