from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(get_package_share_directory('rosie_grasp'), 'config', 'grasp_pose.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('checkpoint_path', default_value=''),
        DeclareLaunchArgument('camera_frame', default_value='camera_color_optical_frame'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        Node(package='rosie_grasp', executable='grasp_pose_node.py', name='rosie_grasp',
             parameters=[config, {'checkpoint_path': LaunchConfiguration('checkpoint_path'),
                                  'camera_frame': LaunchConfiguration('camera_frame'),
                                  'base_frame': LaunchConfiguration('base_frame')}], output='screen')
    ])
