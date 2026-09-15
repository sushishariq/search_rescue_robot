#!/usr/bin/env python3
"""ArUco victim detector.

  /camera/image_raw + calibration
    -> cv2.aruco.detectMarkers
    -> cv2.aruco.estimatePoseSingleMarkers   (pose in camera_rgb_optical_frame)
    -> tf2: camera_rgb_optical_frame -> base_footprint -> odom -> map, at the image timestamp
    -> /victim_detections (sar_interfaces/Victim), /victim_markers (RViz), /aruco/debug_image

Uses the OpenCV 4.5.x aruco API (Dictionary_get / DetectorParameters_create).
"""
import math

import cv2
import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401  registers PoseStamped with tf2_ros.Buffer.transform
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from sar_interfaces.msg import Victim


def rotation_matrix_to_quaternion(R):
    """3x3 rotation matrix -> quaternion (x, y, z, w)."""
    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    x = math.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0
    y = math.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2.0
    z = math.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2.0
    x = math.copysign(x, R[2, 1] - R[1, 2])
    y = math.copysign(y, R[0, 2] - R[2, 0])
    z = math.copysign(z, R[1, 0] - R[0, 1])
    return x, y, z, w


def load_calibration(path):
    """Read K and D from a ROS camera_calibration YAML file."""
    with open(path, 'r') as f:
        calib = yaml.safe_load(f)
    K = np.array(calib['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    D = np.array(calib['distortion_coefficients']['data'], dtype=np.float64)
    return K, D


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('camera_info_file', '')
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('marker_length', 0.20)
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('tf_timeout', 0.1)
        self.declare_parameter('max_range', 3.5)
        self.declare_parameter('min_marker_pixels', 20.0)
        self.declare_parameter('publish_debug_image', True)

        gp = self.get_parameter
        self.marker_length = gp('marker_length').value
        self.target_frame = gp('target_frame').value
        self.tf_timeout = Duration(seconds=gp('tf_timeout').value)
        self.max_range = gp('max_range').value
        self.min_marker_pixels = float(gp('min_marker_pixels').value)
        self.publish_debug = gp('publish_debug_image').value

        self.dictionary = cv2.aruco.Dictionary_get(getattr(cv2.aruco, gp('dictionary').value))
        self.detector_params = cv2.aruco.DetectorParameters_create()
        self.detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        # Calibration: from file if given (checked against the topic), else from the topic.
        self.K = None
        self.D = None
        self.calib_checked = False
        calib_file = gp('camera_info_file').value
        if calib_file:
            self.K, self.D = load_calibration(calib_file)
            self.get_logger().info(f'Loaded calibration from {calib_file}')
        self.create_subscription(CameraInfo, gp('camera_info_topic').value,
                                 self.on_camera_info, qos_profile_sensor_data)

        # TF subscriptions use their own reentrant callback group; with a MultiThreadedExecutor
        # they keep filling the buffer while an image callback waits for a transform.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.bridge = CvBridge()
        self.victim_pub = self.create_publisher(Victim, 'victim_detections', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'victim_markers', 10)
        self.debug_pub = self.create_publisher(Image, 'aruco/debug_image', 1)
        # One image at a time: its own mutually exclusive group, separate from TF.
        self.create_subscription(Image, gp('image_topic').value, self.on_image, qos_profile_sensor_data,
                                 callback_group=MutuallyExclusiveCallbackGroup())

        self.get_logger().info(
            f'ArUco detector ready: marker_length={self.marker_length} m, '
            f'target_frame={self.target_frame}, max_range={self.max_range} m')

    def on_camera_info(self, msg):
        topic_K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        if self.K is None:
            self.K = topic_K
            self.D = np.array(msg.d, dtype=np.float64)
            self.get_logger().info('Using calibration from camera_info topic')
        elif not self.calib_checked:
            if not np.allclose(self.K, topic_K, atol=0.5):
                self.get_logger().warn(
                    f'camera_info_file K differs from camera_info topic K:\n{self.K}\nvs\n{topic_K}')
            self.calib_checked = True

    def on_image(self, msg):
        if self.K is None:
            self.get_logger().warn('Waiting for camera calibration', throttle_duration_sec=5.0)
            return

        gray = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.detector_params)

        debug = None
        if self.publish_debug and self.debug_pub.get_subscription_count() > 0:
            debug = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(debug, corners, ids)

        markers = MarkerArray()
        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.K, self.D)

            for i, marker_id in enumerate(ids.flatten()):
                rvec, tvec, pts = rvecs[i][0], tvecs[i][0], corners[i][0]
                side_px = float(np.mean(np.linalg.norm(pts - np.roll(pts, 1, axis=0), axis=1)))
                rng = float(np.linalg.norm(tvec))
                if side_px < self.min_marker_pixels or rng > self.max_range:
                    continue

                # Marker pose in the camera OPTICAL frame (z forward, x right, y down),
                # stamped with the image time so TF uses where the robot was when the photo was taken.
                pose_cam = PoseStamped()
                pose_cam.header = msg.header
                pose_cam.pose.position.x = float(tvec[0])
                pose_cam.pose.position.y = float(tvec[1])
                pose_cam.pose.position.z = float(tvec[2])
                R, _ = cv2.Rodrigues(rvec)
                q = rotation_matrix_to_quaternion(R)
                pose_cam.pose.orientation.x, pose_cam.pose.orientation.y = q[0], q[1]
                pose_cam.pose.orientation.z, pose_cam.pose.orientation.w = q[2], q[3]

                try:
                    pose_map = self.tf_buffer.transform(pose_cam, self.target_frame, timeout=self.tf_timeout)
                except TransformException as e:
                    self.get_logger().warn(f'TF {msg.header.frame_id} -> {self.target_frame} failed: {e}',
                                           throttle_duration_sec=2.0)
                    continue

                victim = Victim()
                victim.header = pose_map.header
                victim.id = int(marker_id)
                victim.position = pose_map.pose.position
                victim.range = rng
                victim.observations = 1
                self.victim_pub.publish(victim)
                markers.markers.extend(self.make_rviz_markers(victim, pose_map))

                if debug is not None:
                    cv2.aruco.drawAxis(debug, self.K, self.D, rvec, tvec, self.marker_length * 0.5)
                    cv2.putText(debug, f'id{marker_id} {rng:.2f}m', tuple(pts[0].astype(int)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                self.get_logger().debug(
                    f'id={marker_id} map=({victim.position.x:.2f}, {victim.position.y:.2f}) range={rng:.2f}')

        if markers.markers:
            self.marker_pub.publish(markers)
        if debug is not None:
            out = self.bridge.cv2_to_imgmsg(debug, 'bgr8')
            out.header = msg.header
            self.debug_pub.publish(out)

    def make_rviz_markers(self, victim, pose_map):
        cube = Marker()
        cube.header.frame_id = self.target_frame
        cube.ns = 'detections'
        cube.id = victim.id
        cube.type = Marker.CUBE
        cube.action = Marker.ADD
        cube.pose = pose_map.pose
        cube.scale.x = cube.scale.y = cube.scale.z = 0.15
        cube.color.r, cube.color.g, cube.color.b, cube.color.a = 1.0, 0.5, 0.0, 0.8
        cube.lifetime = Duration(seconds=1.0).to_msg()

        text = Marker()
        text.header.frame_id = self.target_frame
        text.ns = 'detection_labels'
        text.id = victim.id
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = victim.position.x
        text.pose.position.y = victim.position.y
        text.pose.position.z = victim.position.z + 0.3
        text.pose.orientation.w = 1.0
        text.scale.z = 0.2
        text.color.r, text.color.g, text.color.b, text.color.a = 1.0, 0.5, 0.0, 1.0
        text.text = f'seen id {victim.id}'
        text.lifetime = Duration(seconds=1.0).to_msg()
        return [cube, text]


def main():
    # Handle Ctrl+C ourselves so the executor is not shut down underneath its worker threads.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ArucoDetector()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
