"""Gazebo + robot_state_publisher + spawn the SAR Waffle.

Usage:
  ros2 launch sar_bringup sim.launch.py
  ros2 launch sar_bringup sim.launch.py world:=/path/to.world x:=1.0 y:=0.5 yaw:=1.57 gui:=false
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
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    
    urdf_path = os.path.join(bringup_dir, 'urdf', 'sar_waffle.urdf')
    sdf_path = os.path.join(bringup_dir, 'models', 'sar_waffle', 'model.sdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    declare_args = [
               DeclareLaunchArgument(
            'world',
            default_value=os.path.join(bringup_dir, 'worlds', 'building.world'),
            description='Full path to the Gazebo world file'),
        DeclareLaunchArgument('gui', default_value='true', description='Start gzclient'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x', default_value='-3.0', description='Spawn x [m]'),
        DeclareLaunchArgument('y', default_value='2.5', description='Spawn y [m]'),
        DeclareLaunchArgument('yaw', default_value='1.5708', description='Spawn yaw [rad]'),
    ]

    # Physics + sensors. Also loads the factory plugin (spawn service) and
    # builds GAZEBO_MODEL_PATH from <gazebo_ros> exports in package.xml files.
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items(),
    )

    # The 3D window only. Optional: the simulation runs without it.
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui),
    )

    # Publishes the fixed TF frames from the URDF (base_link -> camera_rgb_optical_frame, ...)
    # and the wheel frames from /joint_states.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    # Inserts the SDF robot (with sensor + drive plugins) into the running simulation.
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'sar_waffle',
            '-file', sdf_path,
            '-x', x, '-y', y, '-z', '0.01', '-Y', yaw,
        ],
    )

    return LaunchDescription(declare_args + [gzserver, gzclient, robot_state_publisher, spawn_robot])
