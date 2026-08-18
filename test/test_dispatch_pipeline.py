#!/usr/bin/env python3
"""
test_dispatch_pipeline.py
──────────────────────────
End-to-end Phase 1 integration test.

What it does
────────────
1. Starts its own rclpy context.
2. Publishes one StationMarker per station (IDs 0-3) directly to /station_marker,
   bypassing the camera/detector pipeline.  This verifies dispatcher + task simulator
   work correctly without needing ArUco detection or a camera.
3. Collects result log from logs/dispatch_events.jsonl and prints a pass/fail summary.

Run after bringing up the core nodes in another terminal:
  Terminal 1:  ros2 run secure_station_dispatch task_simulator
  Terminal 2:  ros2 run secure_station_dispatch dispatcher
  Terminal 3:  python3 test/test_dispatch_pipeline.py

Or, if using the launch file:
  Terminal 1:  ros2 launch secure_station_dispatch station_dispatch.launch.py
  Terminal 2:  python3 test/test_dispatch_pipeline.py
"""

import sys
import os
import time
import json
import argparse

import rclpy
from rclpy.node import Node

from secure_station_dispatch.msg import StationMarker


# ─── Test data: one marker per station ───────────────────────────────────────
TEST_MARKERS = [
    {'marker_id': 0, 'station_name': 'Inspection Station', 'confidence': 0.95},
    {'marker_id': 1, 'station_name': 'Assembly Station',   'confidence': 0.92},
    {'marker_id': 2, 'station_name': 'Storage Station',    'confidence': 0.88},
    {'marker_id': 3, 'station_name': 'Charging Station',   'confidence': 0.91},
]

# Seconds to wait between marker publications (must be > task sim duration ~3-4 s)
INTER_MARKER_DELAY = 8.0


class TestPublisher(Node):

    def __init__(self, markers: list[dict], delay: float):
        super().__init__('test_pipeline_publisher')
        self._pub    = self.create_publisher(StationMarker, '/station_marker', 10)
        self._markers = markers
        self._delay   = delay
        self._sent    = 0

        # Give subscribers time to connect before firing the first message
        self._timer = self.create_timer(2.0, self._fire)
        self.get_logger().info(
            f'TestPublisher ready — will send {len(markers)} markers '
            f'with {delay:.0f}s gap between each'
        )

    def _fire(self):
        self._timer.cancel()   # one-shot; reschedule manually

        for i, m in enumerate(self._markers):
            self.get_logger().info(
                f'\n{"="*60}\n'
                f'  TEST [{i+1}/{len(self._markers)}]'
                f'  Marker {m["marker_id"]} → {m["station_name"]}\n'
                f'{"="*60}'
            )

            msg = StationMarker()
            msg.marker_id    = m['marker_id']
            msg.station_name = m['station_name']
            msg.robot_pose   = [1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0]
            msg.confidence   = m['confidence']
            msg.stamp        = self.get_clock().now().to_msg()
            msg.hmac_signature = ''  # Phase 1: unsigned

            self._pub.publish(msg)
            self._sent += 1

            if i < len(self._markers) - 1:
                self.get_logger().info(
                    f'  Waiting {self._delay:.0f}s for task to complete…'
                )
                # rclpy-friendly sleep: spin for the delay period
                end = time.time() + self._delay
                while time.time() < end:
                    rclpy.spin_once(self, timeout_sec=0.1)


def _parse_log(log_path: str) -> list[dict]:
    """Read dispatch_events.jsonl and return parsed records."""
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


def _print_summary(records: list[dict], expected_ids: list[int]):
    """Print coloured pass/fail summary."""
    print('\n' + '='*60)
    print('  PIPELINE TEST SUMMARY')
    print('='*60)

    completed = [r for r in records if r.get('outcome') == 'COMPLETED']
    dispatched = [r for r in records if r.get('outcome') == 'DISPATCHED']

    print(f'  Events logged    : {len(records)}')
    print(f'  DISPATCHED events: {len(dispatched)}')
    print(f'  COMPLETED events : {len(completed)}')
    print()

    all_pass = True
    for mid in expected_ids:
        disp = any(r['marker_id'] == mid and r['outcome'] == 'DISPATCHED'
                   for r in records)
        comp = any(r['marker_id'] == mid and r['outcome'] == 'COMPLETED'
                   for r in records)
        status = '✓ PASS' if (disp and comp) else '✗ FAIL'
        if not (disp and comp):
            all_pass = False
        print(f'  Marker {mid}: dispatched={disp} completed={comp}  {status}')

    print()
    print('  OVERALL:', '✓ PASS — Phase 1 pipeline working' if all_pass
          else '✗ FAIL — check node logs above')
    print('='*60 + '\n')
    return all_pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--log_path', default='logs/dispatch_events.jsonl')
    parser.add_argument('--delay', type=float, default=INTER_MARKER_DELAY,
                        help='Seconds to wait between marker publications')
    cli_args, ros_args = parser.parse_known_args()

    # Snapshot existing log length so we only look at new records
    existing = _parse_log(cli_args.log_path)
    existing_count = len(existing)

    rclpy.init(args=ros_args if ros_args else None)
    node = TestPublisher(TEST_MARKERS, cli_args.delay)

    # Run until all markers sent + last task completes
    total_time = 2.0 + len(TEST_MARKERS) * (cli_args.delay + 1.0)
    start = time.time()
    while time.time() - start < total_time:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info('Test complete — parsing log…')
    node.destroy_node()
    rclpy.try_shutdown()

    # Allow a moment for log flush
    time.sleep(1.0)

    all_records = _parse_log(cli_args.log_path)
    new_records  = all_records[existing_count:]

    ok = _print_summary(new_records, [m['marker_id'] for m in TEST_MARKERS])
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
