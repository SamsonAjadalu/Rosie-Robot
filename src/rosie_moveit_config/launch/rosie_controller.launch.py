import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Paths
    package_name = "rosie_moveit_config"
    config_dir = os.path.join(get_package_share_directory(package_name), "config")
    ros2_controllers_path = os.path.join(config_dir, "ros2_controllers.yaml")

    # Load robot description from XACRO
    xacro_file = os.path.join(config_dir, "rosie_V1.urdf.xacro")
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {"robot_description": robot_description_config.toxml()}

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[robot_description, {"use_sim_time": False}],
    )

    # Start Controller Manager
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
        ros2_controllers_path,
        robot_description  # Include the robot description parameter here
    ],
    )

    # Spawn right arm controller
    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Spawn right gripper controller
    right_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager_node,
        right_arm_controller_spawner,
        right_gripper_controller_spawner,
    ])
