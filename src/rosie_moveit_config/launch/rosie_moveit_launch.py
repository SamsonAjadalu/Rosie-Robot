import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Define paths to your package and configuration files
    package_name = "rosie_moveit_config"
    package_share_dir = get_package_share_directory(package_name)
    config_dir = os.path.join(package_share_dir, "config")

    # Load robot description from URDF/XACRO
    xacro_file = os.path.join(config_dir, "rosie_V1.urdf.xacro")
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {"robot_description": robot_description_config.toxml()}

    # Load SRDF file
    robot_description_semantic_path = os.path.join(config_dir, "rosie_V1.srdf")
    with open(robot_description_semantic_path, "r") as file:
        robot_description_semantic = {"robot_description_semantic": file.read()}

    # Define MoveIt configurations
    moveit_config = (
        MoveItConfigsBuilder(robot_name="rosie_V1", package_name=package_name)
        .robot_description(file_path=xacro_file)
        .robot_description_semantic(file_path=robot_description_semantic_path)
        .trajectory_execution(file_path=os.path.join(config_dir, "moveit_controllers.yaml"))
        .robot_description_kinematics(file_path=os.path.join(config_dir, "kinematics.yaml"))  # Added kinematics.yaml
        .moveit_cpp(file_path=os.path.join(config_dir, "controller_setting.yaml"))
        .moveit_cpp(file_path=os.path.join(config_dir, "planning_python_api.yaml"))
        .to_moveit_configs()
    )


    # Nodes to launch
    nodes = [
   

        # Simple MoveIt Interface
        Node(
            package="rosie_moveit_config",
            executable="arm_control_from_UI.py",  # Replace with your executable
            name="moveit_arm_controller",  # Assign an explicit name
            parameters=[moveit_config.to_dict(), {"use_sim_time": False}],
            arguments=["--ros-args", "--log-level", "tf2_ros:=error"],

        ),

                # RViz2 Node for visualization
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", os.path.join(config_dir, "motion_planning.rviz")],
            parameters=[
                robot_description,
                robot_description_semantic,
            ],
        ),
    ]



    return LaunchDescription(nodes)
