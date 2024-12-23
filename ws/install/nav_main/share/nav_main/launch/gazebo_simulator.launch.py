#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare 
from launch.substitutions import PathJoinSubstitution

# robot_description = LaunchConfiguration('robot_description')
# controller_config_file = PathJoinSubstitution([
#     FindPackageShare('nav_main'),
#     'config',
#     'holonomic_velocity_controller.yaml'
# ])


# # not working 
# control_node = Node(
#             package='controller_manager',
#             executable='ros2_control_node',
#             name='ros2_control_node',
#             parameters=[controller_config_file],
#             output='screen',
#         )

def generate_launch_description():    

    gazebo_launch_file = PathJoinSubstitution([
        FindPackageShare('gazebo_tools'),
        'launch',
        'gazebo.launch.py'
    ])
    
    return LaunchDescription([
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file),
            launch_arguments={'output': 'screen'}.items()
        ),
        Node(
            package='nav_main',
            executable='holonomic_controller',
            name='base_controller',
            output='screen',
        ),        
    ])
