"""
Launch file for Rosie Robot Navigation Stack
Brings up:
1. Hardware drivers (mecanumbot_bringup)
2. SLAM Toolbox in online_async mode
3. Nav2 navigation stack
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # Use simulation time set to False for real hardware
    use_sim_time = LaunchConfiguration("use_sim_time", default="False")

    # Get package directories
    rosie_navigation_share = get_package_share_directory("rosie_navigation")
    mecanumbot_bringup_share = get_package_share_directory("mecanumbot_bringup")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    # Configure paths
    nav2_params_file = os.path.join(rosie_navigation_share, "config", "nav2_params.yaml")
    mecanumbot_bringup_launch = os.path.join(mecanumbot_bringup_share, "launch", "mecanumbot_hardware.py")

    # SLAM Toolbox configuration
    slam_params_file = os.path.join(
        get_package_share_directory("slam_toolbox"),
        "config",
        "mapper_params_online_async.yaml",
    )

    return LaunchDescription(
        [
            # Arguments
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=slam_params_file,
                description="Full path to SLAM params file",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=nav2_params_file,
                description="Full path to Nav2 params file",
            ),
            # ===== Hardware Bringup (Wheels + Lidar) =====
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mecanumbot_bringup_launch),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
            # ===== SLAM Toolbox (Online Async Mapping) =====
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[
                    LaunchConfiguration("slam_params_file"),
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[
                    ("scan", "/scan"),
                    ("tf", "tf"),
                    ("tf_static", "tf_static"),
                ],
            ),
            # ===== Nav2 Bringup =====
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_share, "launch", "bringup_launch.py")
                ),
                launch_arguments={
                    "slam": "False",
                    "use_sim_time": use_sim_time,
                    "params_file": LaunchConfiguration("params_file"),
                }.items(),
            ),
        ]
    )
