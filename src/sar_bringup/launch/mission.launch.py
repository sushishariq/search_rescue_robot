"""Full SAR mission: Gazebo + map_server + AMCL + Nav2 + ArUco perception + victim registry + patrol.

Usage:
  ros2 launch sar_bringup mission.launch.py
  ros2 launch sar_bringup mission.launch.py gui:=false   # no Gazebo window
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('sar_bringup')
    perception_dir = get_package_share_directory('sar_perception')
    mission_dir = get_package_share_directory('sar_mission')

    gui = LaunchConfiguration('gui')
    results_file = LaunchConfiguration('results_file')
    waypoints_file = LaunchConfiguration('waypoints_file')

    # Gazebo + robot + map_server + AMCL + Nav2 + RViz
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'nav.launch.py')),
        launch_arguments={
            'gui': gui,
            'rviz_config': os.path.join(bringup_dir, 'rviz', 'mission.rviz'),
        }.items(),
    )

    # ArUco detection + camera -> map projection
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(perception_dir, 'launch', 'perception.launch.py')),
    )

    registry = Node(
        package='sar_mission',
        executable='victim_registry',
        name='victim_registry',
        output='screen',
        parameters=[
            os.path.join(mission_dir, 'config', 'registry.yaml'),
            {'use_sim_time': True, 'results_file': results_file},
        ],
    )

    # Started a few seconds later so Gazebo has spawned the robot; the node itself
    # also waits until AMCL and bt_navigator are active.
    patrol = TimerAction(period=10.0, actions=[Node(
        package='sar_mission',
        executable='patrol',
        name='patrol',
        output='screen',
        parameters=[{'use_sim_time': True, 'waypoints_file': waypoints_file}],
    )])

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Start the Gazebo window'),
        DeclareLaunchArgument('results_file', default_value='/sar_ws/src/sar_mission/results/victims.json'),
        DeclareLaunchArgument('waypoints_file', default_value=os.path.join(mission_dir, 'config', 'waypoints.yaml')),
        navigation,
        perception,
        registry,
        patrol,
    ])
