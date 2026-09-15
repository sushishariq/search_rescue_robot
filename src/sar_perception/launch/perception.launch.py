"""ArUco detector with detection.yaml and the robot's camera calibration file.

Usage:
  ros2 launch sar_perception perception.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    perception_dir = get_package_share_directory('sar_perception')
    bringup_dir = get_package_share_directory('sar_bringup')

    camera_info_file = LaunchConfiguration('camera_info_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    detector = Node(
        package='sar_perception',
        executable='aruco_detector',
        name='aruco_detector',
        output='screen',
        parameters=[
            os.path.join(perception_dir, 'config', 'detection.yaml'),
            {'camera_info_file': camera_info_file, 'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_info_file',
            default_value=os.path.join(bringup_dir, 'config', 'camera_info.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        detector,
    ])
