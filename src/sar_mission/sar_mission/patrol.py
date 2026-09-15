#!/usr/bin/env python3
"""Waypoint patrol using the Nav2 Simple Commander API.

Loads config/waypoints.yaml, sets the AMCL initial pose, runs FollowWaypoints
(the waypoint_follower pauses at each pose so the camera gets a steady look),
then calls /get_victims and prints the registry.
"""
import math

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from sar_interfaces.srv import GetVictims


def make_pose(navigator, frame_id, x, y, yaw_deg):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    yaw = math.radians(float(yaw_deg))
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def print_victims(navigator):
    client = navigator.create_client(GetVictims, 'get_victims')
    if not client.wait_for_service(timeout_sec=5.0):
        navigator.error('get_victims service not available')
        return
    future = client.call_async(GetVictims.Request())
    rclpy.spin_until_future_complete(navigator, future, timeout_sec=5.0)
    if future.result() is None:
        navigator.error('get_victims call failed')
        return
    victims = future.result().victims
    navigator.info(f'Victim registry: {len(victims)} victims')
    for v in victims:
        navigator.info(f'  id {v.id:2d}  x={v.position.x:+.2f}  y={v.position.y:+.2f}  '
                       f'observations={v.observations}')


def main():
    rclpy.init()
    navigator = BasicNavigator(node_name='patrol')
    navigator.declare_parameter('waypoints_file', '')
    path = navigator.get_parameter('waypoints_file').value
    with open(path, 'r') as f:
        cfg = yaml.safe_load(f)

    frame_id = cfg.get('frame_id', 'map')
    loops = int(cfg.get('loops', 1))
    waypoints = cfg['waypoints']

    # BasicNavigator publishes an initial pose while waiting for AMCL; give it the real start
    # pose, otherwise it would publish (0, 0, 0) and throw the localization off.
    start = cfg['initial_pose']
    navigator.setInitialPose(make_pose(navigator, frame_id, start['x'], start['y'], start['yaw']))
    navigator.waitUntilNav2Active()

    for loop in range(loops):
        poses = [make_pose(navigator, frame_id, w['x'], w['y'], w['yaw']) for w in waypoints]
        navigator.info(f'Patrol loop {loop + 1}/{loops}: {len(poses)} waypoints')
        navigator.followWaypoints(poses)

        last_index = -1
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback and feedback.current_waypoint != last_index:
                last_index = feedback.current_waypoint
                w = waypoints[last_index]
                navigator.info(f'  -> waypoint {last_index + 1}/{len(poses)} "{w["name"]}" '
                               f'({w["x"]}, {w["y"]}, {w["yaw"]} deg)')

        result = navigator.getResult()
        missed = list(navigator.result_future.result().result.missed_waypoints)
        if result == TaskResult.SUCCEEDED and not missed:
            navigator.info('Patrol loop complete')
        else:
            navigator.warn(f'Patrol loop ended with {result}, missed waypoints: {missed}')
        print_victims(navigator)

    navigator.info('Patrol finished. Registry keeps running; results are in sar_mission/results/')
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
