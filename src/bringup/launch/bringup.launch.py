#!/usr/bin/env python3
"""
IvastBot Bringup Launch File.
Launches:
  1. Robot State Publisher (URDF model + static TF)
  2. Controller Manager (ros2_control with IDS830 plugin)
  3. Joint State Broadcaster + Diff Drive Controller (spawned after delay)
  4. RPLidar A2M8 (sllidar_ros2, /scan topic)

Structure based on tbm_ws motor.launch.py.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Paths
    pkg_dir = get_package_share_directory('bringup')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'ivastbot.urdf.xacro')
    controllers_file = os.path.join(pkg_dir, 'config', 'controllers.yaml')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]), value_type=str)

    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'lidar_serial_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for RPLidar A2M8'
    )


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
    # 2. Controller Manager — loads IDS830 hardware plugin
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

    # ============================================================
    # 4. RPLidar A2M8 (serial -> /scan)
    # ============================================================
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('sllidar_ros2'),
                'launch',
                'sllidar_a2m8_launch.py'
            ])
        ),
        launch_arguments={
            'serial_port': LaunchConfiguration('lidar_serial_port'),
            'frame_id': 'laser_frame',
            'serial_baudrate': '115200',
            'angle_compensate': 'true',
            'scan_mode': 'Sensitivity',
        }.items()
    )

    return LaunchDescription([
        serial_port_arg,
        robot_state_publisher,
        controller_manager,
        delayed_spawners,
        rplidar_launch,
    ])
