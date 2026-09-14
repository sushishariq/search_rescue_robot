#!/usr/bin/env python3
"""One-shot sanity check: grab one camera frame, detect ArUco markers, print id + tvec.

tvec is the marker centre in the camera OPTICAL frame (x right, y down, z forward).
"""
import argparse

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class ArucoCheck(Node):
    def __init__(self, image_topic, info_topic, marker_length):
        super().__init__('aruco_check')
        self.marker_length = marker_length
        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters_create()
        self.K = None
        self.D = None
        self.done = False
        self.create_subscription(CameraInfo, info_topic, self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)

    def on_info(self, msg):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64)

    def on_image(self, msg):
        if self.K is None or self.done:
            return
        gray = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)
        print(f'image {msg.width}x{msg.height}, frame_id={msg.header.frame_id}')
        if ids is None:
            print('No markers detected')
        else:
            _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.K, self.D)
            for marker_id, tvec in zip(ids.flatten(), tvecs):
                x, y, z = tvec.flatten()
                print(f'id={marker_id}  tvec x={x:+.3f}  y={y:+.3f}  z={z:+.3f}  [m]')
        self.done = True


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--image', default='/camera/image_raw')
    p.add_argument('--info', default='/camera/camera_info')
    p.add_argument('--marker-length', type=float, default=0.20)
    args = p.parse_args()

    rclpy.init()
    node = ArucoCheck(args.image, args.info, args.marker_length)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
