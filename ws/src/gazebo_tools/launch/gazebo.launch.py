#!/usr/bin/env python3


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from pathlib import Path
from launch.substitutions import LaunchConfiguration, Command
import launch_ros.descriptions
import os

pkg_path = get_package_share_directory("gazebo_tools")
robot_xacro_name = "BaseMecanumURDF_old.urdf.xacro"

robot_description_path = str(Path(pkg_path) / "description" / robot_xacro_name)
rviz_config_file_path = str(Path(pkg_path) / "rviz" / "stage.rviz")
world_path = str(Path(pkg_path) / "worlds" / "stage.sdf")
models_path = str(Path(pkg_path) / "meshes")


xacro_file = robot_description_path
use_sim_time = LaunchConfiguration('use_sim_time', default='true')


# For .xacro
robot_description_config = Command(
    ["xacro ", xacro_file, " sim_mode:=", use_sim_time]
)

# additional_paths = [models_path, world_path]
# existing_paths = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
# os.environ["GZ_SIM_RESOURCE_PATH"] = ":".join(filter(None, [existing_paths] + additional_paths))

# For .urdf
# robot_description_path = str(Path(pkg_path) / "description" / "BaseMecanumURDF_old.urdf")
# robot_description_config = open(robot_description_path).read()

# # stage_description_path = str(Path(pkg_path) / "description" / "MiningMayhemField.urdf")
# # stage_description_config = open(stage_description_path).read()

# controller_config = str(Path("nav_main") / "config" / "holonomic_velocity_controller.yaml")



# topc to robot_desc
robot_description_params = {
        "robot_description": launch_ros.descriptions.ParameterValue(
            robot_description_config, value_type=str
        ),
        "use_sim_time": use_sim_time,
    }


# launch Gazebo
sim_cmd = ExecuteProcess(
    cmd=["ign", "gazebo", "-r", world_path],
    output="screen"
)

# open RViz
open_rviz = Node(
    package="rviz2",
    executable="rviz2",
    arguments=["-d", rviz_config_file_path]
)

# launch robot_state_pub
robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="screen",
    parameters=[robot_description_params]
)



# spawm robot
robot_gazebo_bridge = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "robot",
            "-topic",
            "/robot_description",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "2",
        ],
)


# launch jont_state_pub
joint_state_publisher = Node(
    package="joint_state_publisher",
    executable="joint_state_publisher",
    output="screen",
)

def generate_launch_description():
    return LaunchDescription([
        
        robot_state_publisher,
        joint_state_publisher,
        robot_gazebo_bridge,
        open_rviz,
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/r1/gazebo/command/twist@geometry_msgs/msg/Twist@ignition.msgs.Twist",
                "/model/r1/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry"
            ],
            remappings=[
                ("/r1/gazebo/command/twist", "/r1/cmd_vel"),
                ("/model/r1/odometry", "/r1/odom"),
            ],
            output="screen"
        ),
        sim_cmd, 
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=sim_cmd,
        #         on_exit=[EmitEvent(event=Shutdown)]
        #     )
        # )
    ])
