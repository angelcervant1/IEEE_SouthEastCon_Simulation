

# !/usr/bin/env python3

# import os
import launch_ros.descriptions
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from pathlib import Path


pkg_path = get_package_share_directory("gazebo_tools")# Get the gazebo_tools package folder

# Convert Path to string using str() to ensure correct handling
robot_description_path = str(Path(pkg_path) / "description" / "my_robot.xacro")
rviz_config_file_path = str(Path(pkg_path) / "rviz" / "stage.rviz")
world_path = str(Path(pkg_path) / "worlds" / "stage.sdf")

robot_description = {"robot_description" : xacro.process_file(robot_description_path).toxml()}


sim_cmd = ExecuteProcess(
    cmd=["ign", "gazebo", "-r", world_path],
    output="screen"
)

open_rviz = Node(
    package="rviz2",
    executable="rviz2",
    arguments=["-d", rviz_config_file_path]
)


robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="both",
    parameters=[robot_description]
)

def generate_launch_description():
    # use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    
    return LaunchDescription([
        sim_cmd,
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/r1/gazebo/command/twist@geometry_msgs/msg/Twist@ignition.msgs.Twist",
                "/model/r1/odometry@nav_msgs/msg//Odometry@ignition.msgs.Odometry"
            ],
            remappings=[
                ("/r1/gazebo/command/twist", "r1/cmd/vel"),
                ("/model/r1/odometry", "/r1/odom"),
            ],
            output="screen"
        ),
        
        open_rviz,
        robot_state_publisher,
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=sim_cmd,
                on_exit=[EmitEvent(event=Shutdown)]
            )
        )
        
    ])