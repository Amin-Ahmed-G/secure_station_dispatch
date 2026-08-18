#!/usr/bin/env python3
"""
mock_image_publisher.py
────────────────────────
Publishes synthetic ArUco marker images to /camera/image_raw so the
marker_detector_node can be tested without a real camera.

Each image contains a single 4x4_50 ArUco marker (ID cycles through 0–3).
The marker is rendered large enough that the confidence heuristic in
marker_detector_node passes the min_confidence threshold.

Usage (standalone):
  python3 test/mock_image_publisher.py [--marker_id 0] [--rate 0.5]

Usage (via ROS 2):
  ros2 run secure_station_dispatch mock_image_publisher

Topics published:
  /camera/image_raw  (sensor_msgs/Image, encoding=bgr8)
"""

import sys
import time
import argparse

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# ─── ArUco generation helper ──────────────────────────────────────────────────
def _get_aruco_dict():
    dict_id = cv2.aruco.DICT_4X4_50
    try:
        return cv2.aruco.getPredefinedDictionary(dict_id)   # OpenCV ≥ 4.7
    except AttributeError:
        return cv2.aruco.Dictionary_get(dict_id)            # OpenCV < 4.7


def generate_marker_image(marker_id: int, image_size: int = 640) -> np.ndarray:
    """
    Create a BGR image with a large ArUco marker centred on a white background.
    The marker is drawn at 300×300 pixels so the confidence heuristic scores well.
    """
    aruco_dict = _get_aruco_dict()
    marker_size_px = 300
    marker_img = np.zeros((marker_size_px, marker_size_px), dtype=np.uint8)

    try:
        # OpenCV ≥ 4.7
        cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size_px, marker_img)
    except AttributeError:
        # OpenCV < 4.7
        marker_img = cv2.aruco.drawMarker(aruco_dict, marker_id, marker_size_px)

    # Embed marker centred in a white canvas
    canvas = np.ones((image_size, image_size), dtype=np.uint8) * 255
    offset = (image_size - marker_size_px) // 2
    canvas[offset:offset + marker_size_px, offset:offset + marker_size_px] = marker_img

    return cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)


# ─── ROS 2 Node ───────────────────────────────────────────────────────────────
class MockImagePublisher(Node):

    def __init__(self, marker_ids: list[int], rate_hz: float):
        super().__init__('mock_image_publisher')
        self.declare_parameter('rate_hz', rate_hz)
        self.declare_parameter('cycle_markers', True)

        self._bridge    = CvBridge()
        self._marker_ids = marker_ids
        self._idx       = 0
        rate_hz         = self.get_parameter('rate_hz').value
        period          = 1.0 / max(rate_hz, 0.01)

        self._pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self._timer = self.create_timer(period, self._publish)

        self.get_logger().info(
            f'MockImagePublisher: publishing marker IDs {marker_ids} '
            f'at {rate_hz:.1f} Hz on /camera/image_raw'
        )

    def _publish(self):
        marker_id = self._marker_ids[self._idx % len(self._marker_ids)]
        self._idx += 1

        img = generate_marker_image(marker_id)
        msg = self._bridge.cv2_to_imgmsg(img, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        self._pub.publish(msg)

        self.get_logger().info(f'[MOCK CAM] Published marker {marker_id} image')


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Publish synthetic ArUco marker images for testing'
    )
    parser.add_argument('--marker_ids', nargs='+', type=int, default=[0, 1, 2, 3],
                        help='ArUco marker IDs to cycle through')
    parser.add_argument('--rate', type=float, default=0.33,
                        help='Publish rate in Hz (default: 0.33 = one image every 3 s)')
    cli_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args or args)
    node = MockImagePublisher(
        marker_ids=cli_args.marker_ids,
        rate_hz=cli_args.rate,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
