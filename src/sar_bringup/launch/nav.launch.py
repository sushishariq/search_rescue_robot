"""Localization + navigation on the saved map: Gazebo + map_server + AMCL + Nav2 + RViz.

Usage:
  ros2 launch sar_bringup nav.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('sar_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    gui = LaunchConfiguration('gui')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'sim.launch.py')),
        launch_arguments={'gui': gui}.items(),
    )

    # map_server + amcl + planner/controller/bt_navigator/behaviors, all lifecycle-managed
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'params_file': params_file,
            'use_sim_time': 'true',
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(bringup_dir, 'maps', 'building.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config', default_value=os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')),
        DeclareLaunchArgument('gui', default_value='true'),
        sim,
        nav2,
        rviz_node,
    ])
