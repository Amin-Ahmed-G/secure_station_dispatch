#!/usr/bin/env python3
"""
test_dispatch_pipeline.py
──────────────────────────
End-to-end Phase 1 integration test.

What it does
────────────
1. Publishes one StationMarker per station (IDs 0-3) directly to /station_marker,
   bypassing the camera/detector pipeline.  This verifies dispatcher + task simulator
   work correctly without needing ArUco detection or a camera.
2. Reads logs/dispatch_events.jsonl and prints a pass/fail summary.

Run AFTER bringing up the core nodes:
  Terminal 1:  ros2 run secure_station_dispatch task_simulator
  Terminal 2:  ros2 run secure_station_dispatch dispatcher
  Terminal 3:  ros2 run secure_station_dispatch test_dispatch_pipeline

Or use the helper script:
  bash test/run_test.sh
"""

import sys
import os
import time
import json
import threading
import argparse

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from secure_station_dispatch.msg import StationMarker


# ─── Test data: one marker per station ───────────────────────────────────────
TEST_MARKERS = [
    {'marker_id': 0, 'station_name': 'Inspection Station', 'confidence': 0.95},
    {'marker_id': 1, 'station_name': 'Assembly Station',   'confidence': 0.92},
    {'marker_id': 2, 'station_name': 'Storage Station',    'confidence': 0.88},
    {'marker_id': 3, 'station_name': 'Charging Station',   'confidence': 0.91},
]

# Seconds to wait between markers (must exceed longest task sim duration ~3.5 s)
INTER_MARKER_DELAY = 8.0


class TestPublisher(Node):

    def __init__(self, markers: list[dict], delay: float, done_event: threading.Event):
        super().__init__('test_pipeline_publisher')
        self._pub     = self.create_publisher(StationMarker, '/station_marker', 10)
        self._markers = markers
        self._delay   = delay
        self._done    = done_event

        # Wait 2 s for subscribers to connect, then fire in a background thread
        # so we don't block the executor.
        self._thread  = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f'TestPublisher ready — will send {len(markers)} markers '
            f'with {delay:.0f}s gap between each'
        )

    def _run(self):
        time.sleep(2.0)   # let subscribers connect
        for i, m in enumerate(self._markers):
            self.get_logger().info(
                f'\n{"="*60}\n'
                f'  TEST [{i+1}/{len(self._markers)}]  '
                f'Marker {m["marker_id"]} → {m["station_name"]}\n'
                f'{"="*60}'
            )

            msg = StationMarker()
            msg.marker_id      = m['marker_id']
            msg.station_name   = m['station_name']
            msg.robot_pose     = [1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0]
            msg.confidence     = m['confidence']
            msg.stamp          = self.get_clock().now().to_msg()
            msg.hmac_signature = ''   # Phase 1: unsigned

            self._pub.publish(msg)

            if i < len(self._markers) - 1:
                self.get_logger().info(
                    f'  Waiting {self._delay:.0f}s for task to complete…'
                )
                time.sleep(self._delay)   # plain sleep — safe inside a thread

        # Final pause for last task to complete
        self.get_logger().info('All markers sent — waiting for last task to finish…')
        time.sleep(self._delay)
        self.get_logger().info('Test sequence complete.')
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


def _print_summary(records: list[dict], expected_ids: list[int]) -> bool:
    print('\n' + '='*60)
    print('  PIPELINE TEST SUMMARY')
    print('='*60)

    dispatched = [r for r in records if r.get('outcome') == 'DISPATCHED']
    completed  = [r for r in records if r.get('outcome') == 'COMPLETED']

    print(f'  Events logged    : {len(records)}')
    print(f'  DISPATCHED events: {len(dispatched)}')
    print(f'  COMPLETED events : {len(completed)}')
    print()

    all_pass = True
    for mid in expected_ids:
        disp = any(r['marker_id'] == mid and r['outcome'] == 'DISPATCHED' for r in records)
        comp = any(r['marker_id'] == mid and r['outcome'] == 'COMPLETED'  for r in records)
        status = '✓ PASS' if (disp and comp) else '✗ FAIL'
        if not (disp and comp):
            all_pass = False
        print(f'  Marker {mid}: dispatched={disp}  completed={comp}  {status}')

    print()
    verdict = '✓ PASS — Phase 1 pipeline working' if all_pass else '✗ FAIL — check node logs'
    print(f'  OVERALL: {verdict}')
    print('='*60 + '\n')
    return all_pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--log_path', default='logs/dispatch_events.jsonl')
    parser.add_argument('--delay', type=float, default=INTER_MARKER_DELAY)
    cli_args, ros_args = parser.parse_known_args()

    # Snapshot existing log length so we only count NEW records from this run
    existing_count = len(_parse_log(cli_args.log_path))

    rclpy.init(args=ros_args if ros_args else None)

    done_event = threading.Event()
    node = TestPublisher(TEST_MARKERS, cli_args.delay, done_event)

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Spin until the test thread signals completion
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    done_event.wait()   # blocks until all markers sent + last delay elapsed
    executor.shutdown()
    node.destroy_node()
    rclpy.try_shutdown()

    time.sleep(1.0)   # allow final log writes to flush

    all_records = _parse_log(cli_args.log_path)
    new_records  = all_records[existing_count:]

    ok = _print_summary(new_records, [m['marker_id'] for m in TEST_MARKERS])
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
