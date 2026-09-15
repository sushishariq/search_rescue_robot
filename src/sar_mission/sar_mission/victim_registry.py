#!/usr/bin/env python3
"""Victim registry node.

  /victim_detections (sar_interfaces/Victim, one per detected marker per frame)
    -> VictimRegistry dedup (see registry_core.py)
    -> /get_victims service (sar_interfaces/srv/GetVictims): confirmed victims with map x, y
    -> /victim_registry_markers (RViz), results JSON + CSV written on change and at shutdown
"""
import csv
import json
import os
from datetime import datetime, timezone

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from visualization_msgs.msg import Marker, MarkerArray

from sar_interfaces.msg import Victim
from sar_interfaces.srv import GetVictims
from sar_mission.registry_core import VictimRegistry


class VictimRegistryNode(Node):
    def __init__(self):
        super().__init__('victim_registry')

        self.declare_parameter('detections_topic', 'victim_detections')
        self.declare_parameter('merge_distance', 0.5)
        self.declare_parameter('outlier_distance', 1.0)
        self.declare_parameter('min_observations', 3)
        self.declare_parameter('results_file', '/sar_ws/src/sar_mission/results/victims.json')
        self.declare_parameter('save_period', 2.0)

        gp = self.get_parameter
        self.registry = VictimRegistry(
            merge_distance=gp('merge_distance').value,
            outlier_distance=gp('outlier_distance').value,
            min_observations=gp('min_observations').value,
        )
        self.results_file = gp('results_file').value
        self.dirty = False
        self.rejected = 0

        self.create_subscription(Victim, gp('detections_topic').value, self.on_detection, 50)
        self.create_service(GetVictims, 'get_victims', self.on_get_victims)
        self.marker_pub = self.create_publisher(MarkerArray, 'victim_registry_markers', 10)
        self.create_timer(gp('save_period').value, self.on_timer)

        self.get_logger().info(
            f'Victim registry ready: merge_distance={self.registry.merge_distance} m, '
            f'min_observations={self.registry.min_observations}, results_file={self.results_file}')

    def on_detection(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        was_confirmed = {id(t) for t in self.registry.confirmed()}
        track, event = self.registry.add_detection(
            msg.id, msg.position.x, msg.position.y, msg.position.z, msg.range, stamp)

        if event == VictimRegistry.REJECTED:
            self.rejected += 1
            self.get_logger().warn(
                f'Rejected outlier for id {msg.id}: ({msg.position.x:.2f}, {msg.position.y:.2f}) is '
                f'{track.distance_xy(msg.position.x, msg.position.y):.2f} m from ({track.x:.2f}, {track.y:.2f})')
            return
        if event == VictimRegistry.MERGED_NEAR and msg.id != track.marker_id:
            self.get_logger().warn(f'Detection id {msg.id} merged into nearby victim id {track.marker_id}')

        if self.registry.is_confirmed(track):
            self.dirty = True
            if id(track) not in was_confirmed:
                self.get_logger().info(
                    f'NEW VICTIM id {track.marker_id} at map ({track.x:.2f}, {track.y:.2f}) '
                    f'[{len(self.registry.confirmed())} total]')

    def on_get_victims(self, request, response):
        for t in self.registry.confirmed():
            v = Victim()
            v.header.frame_id = 'map'
            v.header.stamp = Time(sec=int(t.last_seen), nanosec=int((t.last_seen % 1) * 1e9))
            v.id = t.marker_id
            v.position.x, v.position.y, v.position.z = t.x, t.y, t.z
            v.range = float(t.best_range)
            v.observations = t.observations
            response.victims.append(v)
        return response

    def on_timer(self):
        self.publish_markers()
        if self.dirty:
            self.save()
            self.dirty = False

    def publish_markers(self):
        markers = MarkerArray()
        for t in self.registry.confirmed():
            body = Marker()
            body.header.frame_id = 'map'
            body.ns = 'victims'
            body.id = t.marker_id
            body.type = Marker.CYLINDER
            body.action = Marker.ADD
            body.pose.position.x, body.pose.position.y, body.pose.position.z = t.x, t.y, 0.25
            body.pose.orientation.w = 1.0
            body.scale.x = body.scale.y = 0.35
            body.scale.z = 0.5
            body.color.r, body.color.g, body.color.b, body.color.a = 0.1, 0.9, 0.1, 0.7

            label = Marker()
            label.header.frame_id = 'map'
            label.ns = 'victim_labels'
            label.id = t.marker_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x, label.pose.position.y, label.pose.position.z = t.x, t.y, 0.9
            label.pose.orientation.w = 1.0
            label.scale.z = 0.25
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f'Victim {t.marker_id}\n({t.x:.2f}, {t.y:.2f})\nn={t.observations}'
            markers.markers.extend([body, label])
        if markers.markers:
            self.marker_pub.publish(markers)

    def save(self):
        data = self.registry.to_dict()
        data['saved_at_utc'] = datetime.now(timezone.utc).isoformat(timespec='seconds')
        data['rejected_outliers'] = self.rejected
        os.makedirs(os.path.dirname(self.results_file) or '.', exist_ok=True)

        tmp = self.results_file + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(data, f, indent=2)
        os.replace(tmp, self.results_file)  # atomic: a crash never leaves a half-written file

        csv_file = os.path.splitext(self.results_file)[0] + '.csv'
        with open(csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['id', 'x', 'y', 'observations'])
            for v in data['victims']:
                writer.writerow([v['id'], v['x'], v['y'], v['observations']])


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = VictimRegistryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.get_logger().info(f'Saved {len(node.registry.confirmed())} victims to {node.results_file}')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
