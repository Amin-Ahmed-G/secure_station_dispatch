#!/usr/bin/env python3
"""
mock_image_publisher.py
────────────────────────
Publishes synthetic ArUco marker images to /camera/image_raw so the
marker_detector_node can be tested without a real camera.

Phase 1 mode (no pose source): cycles through all marker IDs at 1 Hz.
Phase 3 mode (with /mock_pose): subscribes to /mock_pose and, using the
station registry, publishes ONLY the marker whose expected_pose is within
tolerance of the current robot position.  This prevents LOCATION_MISMATCH
spam from wrong markers being detected while parked at a different station.

Usage (standalone):
  python3 test/mock_image_publisher.py [--marker_id 0] [--rate 0.5]

Usage (via ROS 2):
  ros2 run secure_station_dispatch mock_image_publisher

Topics published:
  /camera/image_raw  (sensor_msgs/Image, encoding=bgr8)

Topics subscribed (optional):
  /mock_pose  (geometry_msgs/PoseStamped)
"""

import math
import argparse
import os

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped


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


def _build_image_msg(node: Node, img: np.ndarray) -> Image:
    """
    Manually construct sensor_msgs/Image to avoid cv_bridge KeyError on
    multi-channel OpenCV types (CV_8UC3 = 16 absent from cvtype_to_name
    in some ROS 2 Jazzy / cv_bridge builds).
    """
    img = np.ascontiguousarray(img, dtype=np.uint8)
    h, w, c = img.shape
    msg = Image()
    msg.header.stamp    = node.get_clock().now().to_msg()
    msg.header.frame_id = 'camera_frame'
    msg.height          = h
    msg.width           = w
    msg.encoding        = 'bgr8'
    msg.is_bigendian    = 0
    msg.step            = w * c
    msg.data            = img.tobytes()
    return msg


# ─── ROS 2 Node ───────────────────────────────────────────────────────────────
class MockImagePublisher(Node):

    def __init__(self, marker_ids: list[int], rate_hz: float):
        super().__init__('mock_image_publisher')
        self.declare_parameter('rate_hz', rate_hz)
        self.declare_parameter('cycle_markers', True)
        self.declare_parameter('registry_path', '')

        self._marker_ids  = marker_ids
        self._idx         = 0
        self._robot_pose  = None   # [x, y, z] or None if not yet received
        rate_hz           = self.get_parameter('rate_hz').value
        period            = 1.0 / max(rate_hz, 0.01)

        # Load station registry for pose-aware mode (Phase 3)
        self._registry: dict = {}
        reg_path: str = self.get_parameter('registry_path').value
        if not reg_path:
            # Try installed share path
            try:
                import ament_index_python.packages as ament
                share = ament.get_package_share_directory('secure_station_dispatch')
                reg_path = os.path.join(share, 'config', 'station_registry.yaml')
            except Exception:
                pass
        if reg_path and os.path.exists(reg_path):
            try:
                with open(reg_path) as f:
                    data = yaml.safe_load(f)
                self._registry = {
                    int(k): v for k, v in data.get('stations', {}).items()
                }
                self.get_logger().info(
                    f'[MOCK CAM] Loaded registry ({len(self._registry)} stations) '
                    f'— pose-aware mode active'
                )
            except Exception as e:
                self.get_logger().warn(f'[MOCK CAM] Could not load registry: {e}')

        self._pub = self.create_publisher(Image, '/camera/image_raw', 10)

        # Subscribe to /mock_pose for pose-aware marker selection (Phase 3)
        self._pose_sub = self.create_subscription(
            PoseStamped, '/mock_pose', self._pose_callback, 10
        )

        self._timer = self.create_timer(period, self._publish)

        self.get_logger().info(
            f'MockImagePublisher: publishing marker IDs {marker_ids} '
            f'at {rate_hz:.1f} Hz on /camera/image_raw'
        )

    def _pose_callback(self, msg: PoseStamped):
        p = msg.pose.position
        self._robot_pose = [p.x, p.y, p.z]

    def _marker_for_pose(self) -> int | None:
        """
        Return the marker ID whose expected_pose is closest to the current
        robot pose and within tolerance.  Returns None if no match.
        """
        if self._robot_pose is None or not self._registry:
            return None

        best_id   = None
        best_dist = float('inf')

        for marker_id, station in self._registry.items():
            expected = station.get('expected_pose')
            if expected is None:
                continue
            tolerance = station.get('pose_tolerance_m', 0.6)
            dist = math.sqrt(sum(
                (a - b) ** 2 for a, b in zip(self._robot_pose, expected[:3])
            ))
            if dist <= tolerance and dist < best_dist:
                best_dist = dist
                best_id   = marker_id

        return best_id

    def _publish(self):
        # ── Pose-aware mode: only publish the marker that matches current pose ──
        pose_marker = self._marker_for_pose()
        if pose_marker is not None:
            marker_id = pose_marker
        else:
            # Fallback: round-robin cycling (Phase 1 / standalone mode)
            marker_id = self._marker_ids[self._idx % len(self._marker_ids)]
            self._idx += 1

        img = generate_marker_image(marker_id)          # returns BGR uint8
        if len(img.shape) == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        msg = _build_image_msg(self, img)
        self._pub.publish(msg)

        mode = 'pose-match' if pose_marker is not None else 'cycle'
        self.get_logger().info(
            f'[MOCK CAM] Published marker {marker_id} image ({mode})'
        )


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Publish synthetic ArUco marker images for testing'
    )
    parser.add_argument('--marker_ids', nargs='+', type=int, default=[0, 1, 2, 3],
                        help='ArUco marker IDs to cycle through')
    parser.add_argument('--rate', type=float, default=1.0,
                        help='Publish rate in Hz (default: 1.0)')
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
