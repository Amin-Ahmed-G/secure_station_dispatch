#!/usr/bin/env python3
"""
attacks/spoof_marker_publish.py
───────────────────────────────
Phase 2 Security Attack Demonstration: Marker Spoofing / Unsigned Publish.

Simulates an adversary publishing a forged or unsigned StationMarker message
directly onto the /station_marker ROS 2 topic (bypassing marker detector).

When dispatcher_node is run with enable_security:=true:
  - HMAC verification detects signature mismatch (or missing signature).
  - Dispatcher rejects the message and logs 'AUTH_FAIL' in logs/dispatch_events.jsonl.
  - No action goal is sent to task_simulator_node.

Usage:
  python3 attacks/spoof_marker_publish.py [--marker_id 0] [--forged_sig "INVALID_SIG"]
  OR via ROS 2:
  ros2 run secure_station_dispatch spoof_marker_publish
"""

import sys
import time
import argparse

import rclpy
from rclpy.node import Node
from secure_station_dispatch.msg import StationMarker


class SpoofPublisher(Node):

    def __init__(self, marker_id: int, station_name: str, signature: str):
        super().__init__('spoof_marker_publisher')
        self._pub = self.create_publisher(StationMarker, '/station_marker', 10)
        self._marker_id = marker_id
        self._station_name = station_name
        self._signature = signature

        self.get_logger().info('SpoofMarkerPublisher node initialized.')

    def publish_spoofed_marker(self):
        msg = StationMarker()
        msg.marker_id = self._marker_id
        msg.station_name = self._station_name
        # Use valid pose for station 0 (Inspection Station: [1.0, 2.0, 0.0])
        msg.robot_pose = [1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        msg.confidence = 0.99
        msg.stamp = self.get_clock().now().to_msg()
        msg.hmac_signature = self._signature   # Forged / empty signature!

        self.get_logger().warn(
            f'[ATTACK] Publishing forged marker {msg.marker_id} ({msg.station_name}) '
            f'with signature="{msg.hmac_signature}"...'
        )
        self._pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(description='Publish spoofed/unsigned StationMarker')
    parser.add_argument('--marker_id', type=int, default=0)
    parser.add_argument('--station_name', type=str, default='Inspection Station')
    parser.add_argument('--signature', type=str, default='INVALID_FORGED_HMAC_SIG_999')
    cli_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args if ros_args else None)
    node = SpoofPublisher(cli_args.marker_id, cli_args.station_name, cli_args.signature)

    # Allow subscribers time to connect
    time.sleep(1.5)
    node.publish_spoofed_marker()
    time.sleep(1.0)

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
