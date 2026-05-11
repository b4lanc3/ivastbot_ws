#!/usr/bin/env python3
"""
Launch file for IDS830 robot bringup with ros2_control.
Uses:
  - controller_manager (loads IDS830HW plugin)
  - diff_drive_controller (kinematics + odometry + TF)
  - joint_state_broadcaster
"""

import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Paths
    pkg_dir = get_package_share_directory('ids830_hw')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'robot.urdf.xacro')
    controllers_file = os.path.join(pkg_dir, 'config', 'controllers.yaml')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]), value_type=str)

    # ============================================================
    # 1. Robot State Publisher — URDF model + static TF
    # ============================================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # ============================================================
    # 2. Controller Manager — loads C++ hardware plugin
    # ============================================================
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_file,
        ],
        output='screen',
        remappings=[
            ('/drivetrain_controller/cmd_vel_unstamped', '/cmd_vel'),
        ],
    )

    # ============================================================
    # 3. Spawn controllers (after controller_manager starts)
    # ============================================================
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    drivetrain_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['drivetrain_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Delay spawners to ensure controller_manager is ready
    delayed_spawners = TimerAction(
        period=2.0,
        actions=[
            joint_state_broadcaster_spawner,
            drivetrain_controller_spawner,
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        delayed_spawners,
    ])
