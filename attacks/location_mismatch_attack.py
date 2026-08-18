#!/usr/bin/env python3
"""
attacks/location_mismatch_attack.py
───────────────────────────────────
Phase 2 Security Attack Demonstration: Pose / Location Mismatch.

Simulates a scenario where a valid HMAC signature is present (e.g., replayed or
legitimately signed by a compromised detector), BUT the reported robot position
is impossible / outside the station's pose tolerance radius.

When dispatcher_node is run with enable_security:=true:
  - HMAC verification PASSES (signature is valid).
  - Pose consistency check FAILS (distance from expected station pose > tolerance).
  - Dispatcher rejects message and logs 'LOCATION_MISMATCH' in logs/dispatch_events.jsonl.
  - No action goal is sent to task_simulator_node.

Usage:
  python3 attacks/location_mismatch_attack.py [--marker_id 0]
  OR via ROS 2:
  ros2 run secure_station_dispatch location_mismatch_attack
"""

import sys
import os
import time
import argparse
import hmac
import hashlib
import yaml

import rclpy
from rclpy.node import Node
from secure_station_dispatch.msg import StationMarker


class LocationMismatchPublisher(Node):

    def __init__(self, marker_id: int, fake_pose: list[float], registry_path: str):
        super().__init__('location_mismatch_publisher')
        self._pub = self.create_publisher(StationMarker, '/station_marker', 10)
        self._marker_id = marker_id
        self._fake_pose = fake_pose
        self._registry = self._load_registry(registry_path)

        self.get_logger().info('LocationMismatchPublisher node initialized.')

    def _load_registry(self, path: str) -> dict:
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            return {int(k): v for k, v in data.get('stations', {}).items()}
        except Exception:
            # Fallback default for station 0
            return {0: {'station_name': 'Inspection Station', 'hmac_key': 'inspect-secret-key-do-not-share'}}

    def publish_attack_marker(self):
        station = self._registry.get(self._marker_id, {'station_name': 'Inspection Station', 'hmac_key': 'inspect-secret-key-do-not-share'})
        station_name = station['station_name']
        hmac_key = station['hmac_key']

        # Compute VALID HMAC signature for payload (marker_id + station_name)
        key = hmac_key.encode('utf-8')
        payload = f"{self._marker_id}{station_name}".encode('utf-8')
        valid_signature = hmac.new(key, payload, hashlib.sha256).hexdigest()

        msg = StationMarker()
        msg.marker_id = self._marker_id
        msg.station_name = station_name
        # Pose is IMPOSSIBLE / out-of-tolerance! (e.g. [99.0, 99.0, 0.0])
        msg.robot_pose = list(self._fake_pose)
        msg.confidence = 0.95
        msg.stamp = self.get_clock().now().to_msg()
        msg.hmac_signature = valid_signature   # VALID signature!

        self.get_logger().warn(
            f'[ATTACK] Publishing valid-signed marker {msg.marker_id} ({msg.station_name}) '
            f'with impossible pose={msg.robot_pose[:3]}...'
        )
        self._pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(description='Publish valid-signed StationMarker with mismatched pose')
    parser.add_argument('--marker_id', type=int, default=0)
    parser.add_argument('--registry_path', type=str, default='')
    cli_args, ros_args = parser.parse_known_args()

    registry_path = cli_args.registry_path
    if not registry_path:
        registry_path = os.path.join(
            os.path.dirname(__file__), '..', 'config', 'station_registry.yaml'
        )

    rclpy.init(args=ros_args if ros_args else None)
    # Impossible robot pose: [99.0, 99.0, 0.0, 0.0, 0.0, 0.0, 1.0] (Expected for St. 0 is [1.0, 2.0, 0.0])
    fake_pose = [99.0, 99.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    node = LocationMismatchPublisher(cli_args.marker_id, fake_pose, registry_path)

    time.sleep(1.5)
    node.publish_attack_marker()
    time.sleep(1.0)

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
