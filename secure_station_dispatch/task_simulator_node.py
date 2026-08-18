#!/usr/bin/env python3
"""
task_simulator_node.py
──────────────────────
Phase 1 (assignment): Action server for StationTask.
Satisfies the "ROS 2 services/actions" assignment requirement.

Each task type (Inspection, Assembly, Storage, Charging) is simulated as a
multi-step timed process. The node publishes incremental feedback (progress %)
and returns a result (success + summary) on completion.

Action server:
  /station_task  (secure_station_dispatch/StationTask)

No real actuation — all output is logged/printed (stand-in for real hardware).
"""

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from secure_station_dispatch.action import StationTask


# ─── Task definitions ─────────────────────────────────────────────────────────
# Each task is a list of (step_label, duration_sec) tuples.
# Total duration = sum of step durations (roughly 2-5 seconds per task type).
TASK_STEPS: dict[str, list[tuple[str, float]]] = {
    'Inspection': [
        ('Positioning sensor array',     0.6),
        ('Scanning surface zone A',      0.8),
        ('Scanning surface zone B',      0.7),
        ('Analysing defect signatures',  0.5),
        ('Generating inspection report', 0.4),
    ],
    'Assembly': [
        ('Retrieving component from tray', 0.7),
        ('Aligning component',             0.8),
        ('Pressing / fastening',           1.0),
        ('Torque verification',            0.5),
        ('Quality snap-check',             0.5),
    ],
    'Storage': [
        ('Opening bay door',               0.4),
        ('Extending manipulator',          0.6),
        ('Placing item on shelf',          0.8),
        ('Retracting manipulator',         0.5),
        ('Closing bay door',               0.4),
    ],
    'Charging': [
        ('Aligning to charging pad',       0.5),
        ('Connecting charging plug',       0.6),
        ('Initiating charge handshake',    0.4),
        ('Charging (simulated fast-fill)', 1.5),
        ('Disconnecting and logging SoC',  0.5),
    ],
}

# Fallback for unknown task types
DEFAULT_STEPS: list[tuple[str, float]] = [
    ('Executing generic task', 1.0),
    ('Verifying outcome',      0.5),
]


class TaskSimulatorNode(Node):

    def __init__(self):
        super().__init__('task_simulator_node')

        self._cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            StationTask,
            '/station_task',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )

        self.get_logger().info('TaskSimulatorNode ready — action server /station_task active.')

    # ── Goal / Cancel callbacks ───────────────────────────────────────────────
    def _goal_callback(self, goal_request):
        task_type = goal_request.task_type
        self.get_logger().info(
            f'[GOAL] Received goal: task_type="{task_type}" '
            f'station="{goal_request.station_name}"'
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('[CANCEL] Task cancellation requested.')
        return CancelResponse.ACCEPT

    # ── Execute callback ──────────────────────────────────────────────────────
    def _execute_callback(self, goal_handle):
        """
        Simulate a multi-step task with incremental feedback.
        Runs synchronously in the action server thread.
        """
        task_type    = goal_handle.request.task_type
        station_name = goal_handle.request.station_name

        # Resolve task steps (strip "Station" suffix for lookup flexibility)
        lookup_key = task_type.replace(' Station', '').strip()
        steps = TASK_STEPS.get(lookup_key, DEFAULT_STEPS)

        total_duration = sum(d for _, d in steps)
        elapsed = 0.0

        self.get_logger().info(
            f'[TASK START] {task_type} at {station_name} '
            f'({len(steps)} steps, ~{total_duration:.1f}s)'
        )

        for step_idx, (step_label, duration) in enumerate(steps):
            # Check for cancellation at each step boundary
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('[TASK CANCELLED]')
                result = StationTask.Result()
                result.success = False
                result.summary = f'Task cancelled during step {step_idx + 1}/{len(steps)}'
                return result

            # ── Publish feedback ──────────────────────────────────────────────
            progress = (elapsed / total_duration * 100.0) if total_duration > 0 else 0.0
            feedback_msg = StationTask.Feedback()
            feedback_msg.status   = f'Step {step_idx + 1}/{len(steps)}: {step_label}'
            feedback_msg.progress = float(progress)
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(
                f'  [{step_idx + 1}/{len(steps)}] {step_label} '
                f'({progress:.0f}%)'
            )

            # ── Simulate step duration ────────────────────────────────────────
            time.sleep(duration)
            elapsed += duration

        # ── Final feedback (100 %) ────────────────────────────────────────────
        feedback_msg = StationTask.Feedback()
        feedback_msg.status   = 'Task complete'
        feedback_msg.progress = 100.0
        goal_handle.publish_feedback(feedback_msg)

        # ── Succeed ───────────────────────────────────────────────────────────
        goal_handle.succeed()
        result = StationTask.Result()
        result.success = True
        result.summary = (
            f'{task_type} complete at {station_name} '
            f'in {elapsed:.1f} s ({len(steps)} steps)'
        )

        self.get_logger().info(f'[TASK COMPLETE] {result.summary}')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = TaskSimulatorNode()
    try:
        # ReentrantCallbackGroup needs a multi-threaded executor
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
