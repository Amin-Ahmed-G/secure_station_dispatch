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

Phase 3 additions
─────────────────
1. *Re-dispatch cooldown.*  A berthed robot keeps the marker in frame for
   several seconds after the ~3 s task finishes.  Without a cooldown the state
   machine returns to IDLE, the very next detection frame re-dispatches the same
   station, and the event log double-counts every bay.  Completed markers are now
   held down for ``dispatcher.redispatch_cooldown_s`` from the registry.

2. */dispatch_status.*  Every logged outcome is mirrored to a ``std_msgs/String``
   JSON topic so ``waypoint_tour_node`` can wait for a station to actually finish
   instead of guessing with a fixed dwell.
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

from std_msgs.msg import String

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
        # Phase 3: negative means "take the value from the registry".
        self.declare_parameter('redispatch_cooldown_s', -1.0)
        self.declare_parameter('status_topic', '/dispatch_status')

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

        # ── Re-dispatch cooldown ──────────────────────────────────────────────
        cooldown = self.get_parameter('redispatch_cooldown_s').value
        if cooldown is None or cooldown < 0.0:
            cooldown = float(self._registry_section('dispatcher')
                             .get('redispatch_cooldown_s', 20.0))
        self._cooldown_s: float = float(cooldown)
        self._completed_at: dict[int, float] = {}
        self._suppressed_since: dict[int, float] = {}

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

        # ── Status mirror (Phase 3) ───────────────────────────────────────────
        # Created before the subscription: _marker_callback logs through
        # _publish_status, so the publisher must already exist by the time any
        # detection can arrive.
        self._status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, 20
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
            f'| security_mode={self._enable_security} '
            f'| redispatch_cooldown={self._cooldown_s:.0f}s'
        )

    def _registry_section(self, key: str) -> dict:
        """Return a top-level registry section, tolerating a missing file."""
        section = getattr(self, '_registry_raw', {}).get(key, {})
        return section if isinstance(section, dict) else {}

    def _load_registry(self, path: str) -> dict:
        self._registry_raw = {}
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f) or {}
            self._registry_raw = data
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
            self._log_throttled(
                msg.marker_id,
                f'[IGNORE] Marker {msg.marker_id} received while '
                f'TASK_IN_PROGRESS (busy at {self._active_station})'
            )
            return

        # ── Re-dispatch cooldown (Phase 3) ────────────────────────────────────
        # The camera keeps the board framed for the whole berth dwell, so a
        # completed station would otherwise re-fire on the next frame.
        remaining = self._cooldown_remaining(msg.marker_id)
        if remaining > 0.0:
            self._log_throttled(
                msg.marker_id,
                f'[COOLDOWN] Marker {msg.marker_id} ({msg.station_name}) already '
                f'completed — suppressed for another {remaining:.1f}s'
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

    # ── Cooldown helpers ──────────────────────────────────────────────────────
    def _cooldown_remaining(self, marker_id: int) -> float:
        """Seconds left before *marker_id* may be dispatched again."""
        finished = self._completed_at.get(marker_id)
        if finished is None or self._cooldown_s <= 0.0:
            return 0.0
        return max(0.0, self._cooldown_s - (time.monotonic() - finished))

    def _log_throttled(self, marker_id: int, message: str,
                       interval_s: float = 5.0) -> None:
        """Log at most once per *interval_s* per marker.

        Detections arrive at camera rate; without this the console fills with
        thousands of identical IGNORE/COOLDOWN lines during a single berth dwell.
        """
        now = time.monotonic()
        last = self._suppressed_since.get(marker_id, 0.0)
        if now - last >= interval_s:
            self._suppressed_since[marker_id] = now
            self.get_logger().info(message)

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
        if self._active_marker_id is not None:
            self._completed_at[self._active_marker_id] = time.monotonic()
            self._suppressed_since.pop(self._active_marker_id, None)
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

        # Mirror to /dispatch_status so waypoint_tour_node can wait on a real
        # completion instead of holding a fixed, hopeful dwell at each berth.
        self._publish_status(record)

        self.get_logger().info(
            f'[LOG] {json.dumps({k: record[k] for k in ("marker_id","station","outcome")})}'
        )


    def _publish_status(self, record: dict) -> None:
        """Mirror a dispatch event onto the status topic as compact JSON."""
        payload = dict(record)
        payload['state'] = self._state.name
        payload['cooldown_s'] = round(self._cooldown_remaining(record.get('marker_id') or -1), 2)
        message = String()
        message.data = json.dumps(payload)
        try:
            self._status_pub.publish(message)
        except Exception as e:  # publisher torn down during shutdown
            self.get_logger().debug(f'status publish skipped: {e}')


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
