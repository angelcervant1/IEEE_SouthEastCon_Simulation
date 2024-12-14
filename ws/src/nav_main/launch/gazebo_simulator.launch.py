#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Resolve the Gazebo launch file path
    gazebo_launch_file = PathJoinSubstitution([
        FindPackageShare('gazebo_tools'),
        'launch',
        'gazebo.launch.py'
    ])
    
    # Path to the holonomic velocity controller configuration file
    controller_config_file = PathJoinSubstitution([
        FindPackageShare('nav_main'),
        'config',
        'holonomic_velocity_controller.yaml'
    ])
    
    return LaunchDescription([
        # Include the Gazebo launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file),
            launch_arguments={'output': 'screen'}.items()
        ),

        # Launch the holonomic controller node
        Node(
            package='nav_main',
            executable='holonomic_controller',
            name='controller',
            output='screen',
        ),

        # Load ROS2 Control Manager and pass configuration to the controller
        Node(
            package='controller_manager',
            executable='spawner',
            name='controller_spawner',
            parameters=[controller_config_file]
        ),

        # Load the Holonomic Velocity Controller
        Node(
            package='ros2_control',
            executable='control',
            name='load_holonomic_controller',
            parameters=[
                {'controller_name': 'holonomic_velocity_controller'},
                {'set_state': 'start'}
            ]
        ),

        # Launch the teleop node to send velocity commands
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            remappings=[('/cmd_vel', '/cmd_vel')]
        ),
    ])
