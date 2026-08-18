#!/usr/bin/env python3
"""
test_security_checks.py
────────────────────────
Phase 2 Security Integration Test Suite.

Verifies:
  1. Valid signed marker with valid pose → DISPATCHED & COMPLETED
  2. Unsigned/spoofed marker (invalid HMAC) → AUTH_FAIL
  3. Valid signed marker with invalid pose → LOCATION_MISMATCH

Run AFTER bringing up task_simulator and dispatcher with enable_security:=true:
  bash test/run_security_test.sh
"""

import sys
import os
import time
import json
import hmac
import hashlib
import threading
import argparse

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from secure_station_dispatch.msg import StationMarker


class SecurityTestPublisher(Node):

    def __init__(self, done_event: threading.Event):
        super().__init__('security_test_publisher')
        self._pub = self.create_publisher(StationMarker, '/station_marker', 10)
        self._done = done_event

        self._thread = threading.Thread(target=self._run_tests, daemon=True)
        self._thread.start()

    def _run_tests(self):
        time.sleep(2.0)  # Wait for subscribers

        # Test 1: Valid signed marker for Station 0 (Inspection Station)
        self.get_logger().info('\n=== Security Test 1: Valid HMAC + Valid Pose ===')
        msg1 = StationMarker()
        msg1.marker_id = 0
        msg1.station_name = 'Inspection Station'
        msg1.robot_pose = [1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0]  # Valid pose (expected [1.0, 2.0, 0.0])
        msg1.confidence = 0.95
        msg1.stamp = self.get_clock().now().to_msg()
        # Compute valid signature
        key = b'inspect-secret-key-do-not-share'
        payload = f"{msg1.marker_id}{msg1.station_name}".encode('utf-8')
        msg1.hmac_signature = hmac.new(key, payload, hashlib.sha256).hexdigest()
        self._pub.publish(msg1)
        time.sleep(6.0)

        # Test 2: Spoofed / Invalid HMAC signature for Station 1 (Assembly Station)
        self.get_logger().info('\n=== Security Test 2: Invalid HMAC (Spoof Attack) ===')
        msg2 = StationMarker()
        msg2.marker_id = 1
        msg2.station_name = 'Assembly Station'
        msg2.robot_pose = [4.5, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]  # Valid pose
        msg2.confidence = 0.90
        msg2.stamp = self.get_clock().now().to_msg()
        msg2.hmac_signature = 'INVALID_FORGED_SIGNATURE_12345'
        self._pub.publish(msg2)
        time.sleep(3.0)

        # Test 3: Valid HMAC signature + Impossible Pose for Station 2 (Storage Station)
        self.get_logger().info('\n=== Security Test 3: Location Mismatch Attack ===')
        msg3 = StationMarker()
        msg3.marker_id = 2
        msg3.station_name = 'Storage Station'
        msg3.robot_pose = [99.0, 99.0, 0.0, 0.0, 0.0, 0.0, 1.0]  # Out of tolerance! (Expected [4.5, 5.0, 0.0])
        msg3.confidence = 0.92
        msg3.stamp = self.get_clock().now().to_msg()
        key2 = b'storage-secret-key-do-not-share'
        payload2 = f"{msg3.marker_id}{msg3.station_name}".encode('utf-8')
        msg3.hmac_signature = hmac.new(key2, payload2, hashlib.sha256).hexdigest()
        self._pub.publish(msg3)
        time.sleep(3.0)

        self.get_logger().info('Security test sequence complete.')
        self._done.set()


def _parse_log(log_path: str) -> list[dict]:
    if not os.path.exists(log_path):
        return []
    records = []
    with open(log_path) as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    records.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
    return records


def _print_security_summary(records: list[dict]) -> bool:
    print('\n' + '=' * 60)
    print('  PHASE 2 SECURITY TEST SUMMARY')
    print('=' * 60)

    auth_fails = [r for r in records if r.get('outcome') == 'AUTH_FAIL']
    location_mismatches = [r for r in records if r.get('outcome') == 'LOCATION_MISMATCH']
    dispatched = [r for r in records if r.get('outcome') == 'DISPATCHED']
    completed = [r for r in records if r.get('outcome') == 'COMPLETED']

    print(f'  Total security events logged: {len(records)}')
    print(f'  Valid dispatches           : {len(dispatched)}')
    print(f'  Valid completions          : {len(completed)}')
    print(f'  AUTH_FAIL rejections       : {len(auth_fails)}')
    print(f'  LOCATION_MISMATCH rejections: {len(location_mismatches)}')
    print()

    # Check 1: Valid marker 0 dispatched & completed
    valid_ok = any(r['marker_id'] == 0 and r['outcome'] == 'COMPLETED' for r in records)
    print(f'  Test 1 (Valid signed marker):       {"✓ PASS" if valid_ok else "✗ FAIL"}')

    # Check 2: Invalid HMAC marker 1 rejected with AUTH_FAIL
    auth_ok = any(r['marker_id'] == 1 and r['outcome'] == 'AUTH_FAIL' for r in records)
    print(f'  Test 2 (Spoofed HMAC rejected):     {"✓ PASS" if auth_ok else "✗ FAIL"}')

    # Check 3: Mismatched pose marker 2 rejected with LOCATION_MISMATCH
    loc_ok = any(r['marker_id'] == 2 and r['outcome'] == 'LOCATION_MISMATCH' for r in records)
    print(f'  Test 3 (Location mismatch rejected): {"✓ PASS" if loc_ok else "✗ FAIL"}')

    all_pass = valid_ok and auth_ok and loc_ok
    print()
    verdict = '✓ PASS — All Phase 2 security protections working' if all_pass else '✗ FAIL — Check logs'
    print(f'  OVERALL: {verdict}')
    print('=' * 60 + '\n')
    return all_pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--log_path', default='logs/dispatch_events.jsonl')
    cli_args, ros_args = parser.parse_known_args()

    existing_count = len(_parse_log(cli_args.log_path))

    rclpy.init(args=ros_args if ros_args else None)

    done_event = threading.Event()
    node = SecurityTestPublisher(done_event)

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    done_event.wait()
    executor.shutdown()
    node.destroy_node()
    rclpy.try_shutdown()

    time.sleep(1.0)

    all_records = _parse_log(cli_args.log_path)
    new_records = all_records[existing_count:]

    ok = _print_security_summary(new_records)
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
