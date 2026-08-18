#!/usr/bin/env python3
"""
dispatcher_node.py
──────────────────
Phase 1 (assignment): State-machine dispatcher that receives StationMarker
detections and drives the StationTask action server.

State machine (satisfies "state machine" assignment requirement):
  IDLE             – waiting for a valid marker detection
  TASK_IN_PROGRESS – an action goal is active; new detections are ignored

Subscribes:
  /station_marker  (secure_station_dispatch/StationMarker)

Action client:
  /station_task    (secure_station_dispatch/StationTask)

Logs every event to:
  logs/dispatch_events.jsonl   (path configurable via ROS parameter)

Phase 2 security hooks are imported from security/checks.py but are no-ops in
Phase 1 — this keeps Phase 1 logic intact and Phase 2 additive only.
"""

import os
import json
import time
import yaml
from enum import Enum, auto
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from secure_station_dispatch.msg import StationMarker
from secure_station_dispatch.action import StationTask

# Phase 2 security hook (no-op in Phase 1; real impl lives in security/checks.py)
from secure_station_dispatch.security.checks import (
    verify_hmac,       # returns True (always) in Phase 1
    check_pose,        # returns True (always) in Phase 1
)


# ─── State Machine ────────────────────────────────────────────────────────────
class DispatchState(Enum):
    IDLE             = auto()
    TASK_IN_PROGRESS = auto()


# ─── Node ─────────────────────────────────────────────────────────────────────
class DispatcherNode(Node):

    def __init__(self):
        super().__init__('dispatcher_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('log_path', 'logs/dispatch_events.jsonl')
        self.declare_parameter('registry_path', '')
        self.declare_parameter('enable_security', False)  # Phase 2 toggle

        log_path_param = self.get_parameter('log_path').value
        self._log_path = os.path.abspath(log_path_param)
        os.makedirs(os.path.dirname(self._log_path), exist_ok=True)

        self._enable_security: bool = self.get_parameter('enable_security').value

        registry_path = self.get_parameter('registry_path').value
        if not registry_path or not os.path.exists(registry_path):
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg_share = get_package_share_directory('secure_station_dispatch')
                registry_path = os.path.join(pkg_share, 'config', 'station_registry.yaml')
            except Exception:
                dev_path = os.path.join(
                    os.path.dirname(__file__), '..', 'config', 'station_registry.yaml'
                )
                if os.path.exists(dev_path):
                    registry_path = dev_path

        self.registry = self._load_registry(registry_path)

        # ── State machine ─────────────────────────────────────────────────────
        self._state = DispatchState.IDLE
        self._active_marker_id: int | None = None
        self._active_station: str | None = None

        # ── Callback group (allow action client callbacks alongside subs) ──────
        self._cb_group = ReentrantCallbackGroup()

        # ── Action client ─────────────────────────────────────────────────────
        self._action_client = ActionClient(
            self,
            StationTask,
            '/station_task',
            callback_group=self._cb_group,
        )

        # ── Subscription ──────────────────────────────────────────────────────
        self._sub = self.create_subscription(
            StationMarker,
            '/station_marker',
            self._marker_callback,
            10,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            f'DispatcherNode ready | state=IDLE | log={self._log_path} '
            f'| security_mode={self._enable_security}'
        )

    def _load_registry(self, path: str) -> dict:
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            stations = {int(k): v for k, v in data.get('stations', {}).items()}
            self.get_logger().info(
                f'Loaded station registry from {path}: '
                f'{list(stations.keys())} markers'
            )
            return stations
        except Exception as e:
            self.get_logger().error(f'Failed to load registry from {path}: {e}')
            return {}

    # ── Marker callback ───────────────────────────────────────────────────────
    def _marker_callback(self, msg: StationMarker):
        # ── State guard: ignore detections while a task is running ────────────
        if self._state == DispatchState.TASK_IN_PROGRESS:
            self.get_logger().info(
                f'[IGNORE] Marker {msg.marker_id} received while '
                f'TASK_IN_PROGRESS (busy at {self._active_station})'
            )
            return

        self.get_logger().info(
            f'[DETECT] Received marker {msg.marker_id} → {msg.station_name} '
            f'(conf={msg.confidence:.2f})'
        )

        # ── Phase 2 security checks ───────────────────────────────────────────
        if self._enable_security:
            if not verify_hmac(msg, self.registry):
                self._log_event(msg, outcome='AUTH_FAIL')
                self.get_logger().warn(
                    f'[SECURITY] AUTH_FAIL for marker {msg.marker_id} '
                    f'at {msg.station_name}'
                )
                return

            if not check_pose(msg, self.registry):
                self._log_event(msg, outcome='LOCATION_MISMATCH')
                self.get_logger().warn(
                    f'[SECURITY] LOCATION_MISMATCH for marker {msg.marker_id} '
                    f'at {msg.station_name}'
                )
                return

        # ── Dispatch ──────────────────────────────────────────────────────────
        self._dispatch(msg)

    def _dispatch(self, msg: StationMarker):
        """Transition to TASK_IN_PROGRESS and send action goal."""
        self._state = DispatchState.TASK_IN_PROGRESS
        self._active_marker_id = msg.marker_id
        self._active_station = msg.station_name

        self.get_logger().info(
            f'[DISPATCH] {msg.station_name} | state → TASK_IN_PROGRESS'
        )
        self._log_event(msg, outcome='DISPATCHED')

        # Wait for action server (non-blocking with timeout)
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                '[DISPATCH] Action server /station_task not available!'
            )
            self._reset_to_idle(msg, outcome='SERVER_UNAVAILABLE')
            return

        goal = StationTask.Goal()
        goal.task_type    = msg.station_name  # registry resolves this in task sim
        goal.station_name = msg.station_name

        future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )
        future.add_done_callback(self._goal_response_callback)

    # ── Action callbacks ──────────────────────────────────────────────────────
    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('[ACTION] Goal rejected by task server!')
            self._reset_to_idle(outcome='GOAL_REJECTED')
            return

        self.get_logger().info('[ACTION] Goal accepted — task running…')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'[FEEDBACK] {self._active_station} | {fb.status} '
            f'({fb.progress:.0f}%)'
        )

    def _result_callback(self, future):
        result = future.result().result
        outcome = 'COMPLETED' if result.success else 'FAILED'
        self.get_logger().info(
            f'[RESULT] {self._active_station} → {outcome} | {result.summary}'
        )
        self._log_event_raw(
            marker_id=self._active_marker_id,
            station=self._active_station,
            outcome=outcome,
            summary=result.summary,
        )
        self._reset_to_idle()

    # ── State helpers ─────────────────────────────────────────────────────────
    def _reset_to_idle(self, msg: StationMarker | None = None, outcome: str = ''):
        self._state = DispatchState.IDLE
        self._active_marker_id = None
        self._active_station = None
        self.get_logger().info('[STATE] → IDLE')
        if outcome:
            self._log_event(msg, outcome=outcome)

    # ── Logging ───────────────────────────────────────────────────────────────
    def _log_event(self, msg: StationMarker | None, outcome: str):
        """Log a dispatch event from a StationMarker message."""
        if msg is None:
            self._log_event_raw(
                marker_id=self._active_marker_id,
                station=self._active_station or '',
                outcome=outcome,
            )
            return
        self._log_event_raw(
            marker_id=msg.marker_id,
            station=msg.station_name,
            outcome=outcome,
        )

    def _log_event_raw(
        self,
        marker_id: int | None,
        station: str,
        outcome: str,
        summary: str = '',
    ):
        record = {
            'timestamp': datetime.now(timezone.utc).isoformat(),
            'ros_time':  self.get_clock().now().nanoseconds,
            'marker_id': marker_id,
            'station':   station,
            'outcome':   outcome,
            'summary':   summary,
        }
        try:
            with open(self._log_path, 'a') as f:
                f.write(json.dumps(record) + '\n')
        except Exception as e:
            self.get_logger().error(f'Failed to write log: {e}')

        self.get_logger().info(
            f'[LOG] {json.dumps({k: record[k] for k in ("marker_id","station","outcome")})}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = DispatcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
