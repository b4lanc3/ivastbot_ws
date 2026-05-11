#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'joystick_index', default_value='-1',
            description='Index tay cầm (-1=tự động tìm Flydigi, 0=đầu tiên, 1=thứ hai, ...)'
        ),
        Node(
            package='flydigi',
            executable='flydigi_node',
            name='flydigi_controller_node',
            output='screen',
            parameters=[{
                'joystick_index': LaunchConfiguration('joystick_index'),
            }]
        )
    ])
