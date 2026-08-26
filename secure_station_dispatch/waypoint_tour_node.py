#!/usr/bin/env python3
"""
waypoint_tour_node.py
─────────────────────
Phase 3: obstacle-aware scripted station tour.

What changed and why
────────────────────
The first version of this node drove a straight line between berths and trusted
that the floor was empty.  It is not: every station carries a marker board on a
solid bay frame.  The leg from the Assembly berth [4.5, 1.0] to the Storage
berth [4.5, 5.0] ran straight through the Assembly bay, the robot wedged against
it, and because nothing watched for lack of progress the controller kept pushing
until the leg timed out.  Every later leg then started from a corrupted pose, no
further marker was ever framed, and the tour silently reported success.

Three things were wrong, and all three are fixed here:

  1. *No obstacle model.*  Legs are now planned by
     ``secure_station_dispatch.navigation.PathPlanner`` over the obstacle set
     declared in ``station_registry.yaml``, so a blocked straight line becomes a
     detour instead of a collision.
  2. *No stall detection.*  A watchdog aborts a leg when the robot is commanded
     to move but does not, and a back-up-and-turn recovery re-plans from the
     freed pose.
  3. *No arrival verification.*  A leg that times out used to ``break`` and fall
     through as if it had arrived.  Arrival is now checked against the registry
     tolerances and a failed berth is reported, not swallowed.

The node also waits for the dispatcher to report task completion instead of
holding a fixed dwell, and writes a per-station tour report to
``logs/tour_report.json``.

Odometry sources
────────────────
With Gazebo running, poses come from ``/odom``.  When the node is run headless
(``test/run_gazebo_tour_test.sh``) no odometry ever arrives, so after
``navigation.odom_wait_s`` the node integrates its own commanded velocities
instead.  Both modes drive the identical controller, so the headless run
exercises the same planning, stall and arrival logic as the simulated one.
"""

from __future__ import annotations

import json
import math
import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from secure_station_dispatch.navigation import PathPlanner, obstacles_from_registry
from secure_station_dispatch.navigation.controller import (
    ControlParams, DiffDriveController, StallWatchdog, normalize_angle,
)

# ── Leg outcomes ──────────────────────────────────────────────────────────────
ARRIVED = 'arrived'
STALLED = 'stalled'
TIMEOUT = 'timeout'

DEFAULT_NAV = {
    'robot_radius_m': 0.25,
    'safety_margin_m': 0.15,
    'goal_tolerance_m': 0.12,
    'yaw_tolerance_rad': 0.06,
    'max_linear_speed': 0.45,
    'min_linear_speed': 0.05,
    'max_angular_speed': 0.90,
    'min_angular_speed': 0.12,
    'linear_gain': 0.80,
    'angular_gain': 1.40,
    'turn_in_place_rad': 0.55,
    'stall_window_s': 3.0,
    'stall_min_progress_m': 0.04,
    'recovery_backup_m': 0.35,
    'recovery_backup_speed': 0.15,
    'recovery_turn_rad': 0.70,
    'max_recoveries_per_leg': 3,
    'segment_timeout_s': 45.0,
    'rotate_timeout_s': 15.0,
    'odom_wait_s': 20.0,
    'task_wait_timeout_s': 25.0,
    'berth_settle_s': 1.5,
    'return_home': True,
}

CONTROL_PERIOD = 0.05      # s, 20 Hz control + pose publication


# ── Small helpers ─────────────────────────────────────────────────────────────
def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# The control law, its tuning and the stall watchdog live in
# navigation.controller so they can be replayed offline by the test suite.


# ═══════════════════════════════════════════════════════════════════════════════
class WaypointTourNode(Node):

    def __init__(self):
        super().__init__('waypoint_tour_node')

        self.declare_parameter('registry_path', '')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('pose_topic', '/mock_pose')
        self.declare_parameter('report_path', 'logs/tour_report.json')
        self.declare_parameter('return_home', True)

        registry_path = self._resolve_registry(self.get_parameter('registry_path').value)
        self.registry = self._load_registry(registry_path)

        self.nav = dict(DEFAULT_NAV)
        self.nav.update(self.registry.get('navigation', {}) or {})
        if not self.get_parameter('return_home').value:
            self.nav['return_home'] = False

        world = self.registry.get('world', {}) or {}
        home = list(world.get('home_pose') or [0.0, 0.0, 0.0])
        home += [0.0] * (3 - len(home))
        self.home_pose = tuple(home[:3])
        self.frame_id = world.get('frame_id', 'map')

        obstacles = obstacles_from_registry(self.registry)
        self.planner = PathPlanner(
            obstacles=obstacles,
            inflation=self.nav['robot_radius_m'] + self.nav['safety_margin_m'],
            hard_inflation=self.nav['robot_radius_m'],
            bounds=world.get('bounds'),
        )
        self.controller = DiffDriveController(
            ControlParams.from_registry(self.registry.get('navigation')))
        self.watchdog = StallWatchdog(self.nav['stall_window_s'],
                                      self.nav['stall_min_progress_m'])

        self.stations = self._build_itinerary()

        # ── Pose state ────────────────────────────────────────────────────────
        self.x, self.y, self.yaw = self.home_pose[0], self.home_pose[1], self.home_pose[2]
        self.odom_received = False
        self.simulated_odometry = False
        self._last_cmd = (0.0, 0.0)
        self._last_integration_t = time.time()

        # ── Dispatcher feedback ───────────────────────────────────────────────
        self._dispatch_events: list[dict] = []

        # ── ROS interfaces ────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.pose_pub = self.create_publisher(PoseStamped, self.get_parameter('pose_topic').value, 10)
        self.status_pub = self.create_publisher(String, '/tour_status', 10)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                 self._odom_cb, 10)
        self.create_subscription(String, '/dispatch_status', self._dispatch_status_cb, 20)
        self.create_timer(CONTROL_PERIOD, self._publish_pose)

        self.get_logger().info(
            f'WaypointTourNode ready | {len(self.stations)} stations '
            f'| {len(obstacles)} obstacles | inflation '
            f'{self.planner.inflation:.2f} m | registry {registry_path}'
        )

    # ── Setup helpers ─────────────────────────────────────────────────────────
    def _resolve_registry(self, override: str) -> str:
        if override and os.path.exists(override):
            return override
        try:
            return os.path.join(get_package_share_directory('secure_station_dispatch'),
                                'config', 'station_registry.yaml')
        except Exception:
            return os.path.abspath(os.path.join(os.path.dirname(__file__), '..',
                                                'config', 'station_registry.yaml'))

    def _load_registry(self, path: str) -> dict:
        try:
            with open(path, 'r') as handle:
                return yaml.safe_load(handle) or {}
        except Exception as exc:
            self.get_logger().error(f'Failed to load registry {path}: {exc}')
            return {}

    def _build_itinerary(self) -> list[dict]:
        itinerary = []
        for marker_id, station in sorted((self.registry.get('stations', {}) or {}).items()):
            pose = station.get('expected_pose') or [0.0, 0.0, 0.0]
            itinerary.append({
                'marker_id': int(marker_id),
                'name': station.get('station_name', f'Station {marker_id}'),
                'berth': (float(pose[0]), float(pose[1])),
                'approach_yaw': float(station.get('approach_yaw', 0.0)),
                'tolerance': float(station.get('pose_tolerance_m', 0.5)),
            })
        return itinerary

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.x, self.y = position.x, position.y
        self.yaw = yaw_from_quaternion(orientation.x, orientation.y,
                                       orientation.z, orientation.w)
        if not self.odom_received:
            self.get_logger().info('/odom stream acquired — using simulator odometry.')
        self.odom_received = True
        if self.simulated_odometry:
            # Gazebo came up late.  Real odometry always wins; keeping the
            # integrator running alongside it would corrupt the pose between
            # /odom messages.
            self.simulated_odometry = False
            self.get_logger().warn('/odom arrived after the fallback engaged — '
                                   'switching back to simulator odometry.')

    def _dispatch_status_cb(self, msg: String) -> None:
        try:
            self._dispatch_events.append(json.loads(msg.data))
        except (ValueError, TypeError):
            self.get_logger().warn(f'Unparseable /dispatch_status payload: {msg.data!r}')

    # ── Pose I/O ──────────────────────────────────────────────────────────────
    @property
    def xy(self) -> tuple[float, float]:
        return (self.x, self.y)

    def _publish_pose(self) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.orientation.z = math.sin(self.yaw * 0.5)
        msg.pose.orientation.w = math.cos(self.yaw * 0.5)
        self.pose_pub.publish(msg)

    def _publish_cmd(self, linear: float = 0.0, angular: float = 0.0) -> None:
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.cmd_pub.publish(twist)
        self._last_cmd = (float(linear), float(angular))

    def stop(self) -> None:
        self._publish_cmd(0.0, 0.0)
        self._tick(0.05)

    def _tick(self, timeout: float = CONTROL_PERIOD) -> None:
        """One control iteration: spin callbacks, then integrate if headless."""
        rclpy.spin_once(self, timeout_sec=timeout)
        now = time.time()
        dt = now - self._last_integration_t
        self._last_integration_t = now
        if self.simulated_odometry and dt > 0.0:
            linear, angular = self._last_cmd
            self.yaw = normalize_angle(self.yaw + angular * dt)
            self.x += linear * math.cos(self.yaw) * dt
            self.y += linear * math.sin(self.yaw) * dt

    def _emit(self, event: str, **fields) -> None:
        payload = {'event': event, **fields}
        message = String()
        message.data = json.dumps(payload)
        self.status_pub.publish(message)

    # ═════════════════════════════════════════════════════════════════════════
    # Motion primitives
    # ═════════════════════════════════════════════════════════════════════════
    def rotate_to(self, target_yaw: float) -> bool:
        """Turn on the spot until the heading error is inside tolerance."""
        deadline = time.time() + self.nav['rotate_timeout_s']
        while time.time() < deadline:
            self._tick()
            command = self.controller.rotate_command(self.yaw, target_yaw)
            if command.arrived:
                self.stop()
                return True
            self._publish_cmd(command.linear, command.angular)
        self.stop()
        self.get_logger().warn(f'rotate_to({target_yaw:+.2f}) timed out '
                               f'at yaw {self.yaw:+.2f}')
        return False

    def drive_to_point(self, target: tuple[float, float]) -> str:
        """Closed-loop drive to *target*, watched for stalls.

        Returns ``ARRIVED``, ``STALLED`` or ``TIMEOUT``.  The watchdog is the
        piece that was missing when the robot wedged against the Assembly bay:
        if we are asking for motion and not getting any, we stop asking.
        """
        deadline = time.time() + self.nav['segment_timeout_s']
        self.watchdog.reset(time.time(), self.xy)

        while time.time() < deadline:
            self._tick()

            command = self.controller.command(self.x, self.y, self.yaw, target)
            if command.arrived:
                self.stop()
                return ARRIVED
            self._publish_cmd(command.linear, command.angular)

            if self.watchdog.update(time.time(), self.xy, command.is_moving):
                self.stop()
                self.get_logger().error(
                    f'STALL: commanded motion but moved only '
                    f'{self.watchdog.travelled * 100:.1f} cm in '
                    f'{self.nav["stall_window_s"]:.1f} s at '
                    f'[{self.x:.2f}, {self.y:.2f}] — aborting segment towards '
                    f'[{target[0]:.2f}, {target[1]:.2f}]')
                return STALLED

        self.stop()
        self.get_logger().error(
            f'TIMEOUT after {self.nav["segment_timeout_s"]:.0f} s driving to '
            f'[{target[0]:.2f}, {target[1]:.2f}] from [{self.x:.2f}, {self.y:.2f}]')
        return TIMEOUT

    def recover(self, reason: str) -> None:
        """Back away from whatever we are touching, then turn to break symmetry."""
        self.get_logger().warn(f'RECOVERY ({reason}): backing off '
                               f'{self.nav["recovery_backup_m"]:.2f} m and turning.')
        self._emit('recovery', reason=reason, pose=[round(self.x, 3), round(self.y, 3)])

        start = self.xy
        speed = self.nav['recovery_backup_speed']
        deadline = time.time() + self.nav['recovery_backup_m'] / max(speed, 0.01) + 4.0
        while time.time() < deadline:
            self._tick()
            self._publish_cmd(-speed, 0.0)
            if math.dist(self.xy, start) >= self.nav['recovery_backup_m']:
                break
        self.stop()
        self.rotate_to(normalize_angle(self.yaw + self.nav['recovery_turn_rad']))
        self.stop()

    # ═════════════════════════════════════════════════════════════════════════
    # Leg execution
    # ═════════════════════════════════════════════════════════════════════════
    def execute_leg(self, goal: tuple[float, float], label: str) -> dict:
        """Plan and drive to *goal*, re-planning after each recovery."""
        attempts = int(self.nav['max_recoveries_per_leg']) + 1
        recoveries = 0
        plan_note = ''

        for attempt in range(1, attempts + 1):
            plan = self.planner.plan(self.xy, goal)
            plan_note = plan.describe(self.xy)

            if plan.fallback:
                self.get_logger().error(
                    f'{label}: no collision-free route found — '
                    f'blocked by {", ".join(plan.blocking)}. Driving direct and '
                    f'relying on the stall watchdog.')
            elif plan.direct:
                self.get_logger().info(f'{label}: straight-line path is clear.')
            else:
                self.get_logger().warn(f'{label}: {plan_note}')

            status = ARRIVED
            for waypoint in plan.waypoints:
                status = self.drive_to_point(waypoint)
                if status != ARRIVED:
                    break

            if status == ARRIVED:
                return {'reached': True, 'recoveries': recoveries,
                        'plan': plan_note, 'attempts': attempt}

            if attempt == attempts:
                break
            recoveries += 1
            self.recover(status)

        self.get_logger().error(f'{label}: giving up after {recoveries} recoveries.')
        return {'reached': False, 'recoveries': recoveries,
                'plan': plan_note, 'attempts': attempts}

    def wait_for_task(self, marker_id: int) -> dict:
        """Hold the berth until the dispatcher reports this station finished."""
        self._dispatch_events.clear()
        settle_until = time.time() + self.nav['berth_settle_s']
        while time.time() < settle_until:
            self._tick()

        deadline = time.time() + self.nav['task_wait_timeout_s']
        while time.time() < deadline:
            self._tick()
            for event in self._dispatch_events:
                if event.get('marker_id') != marker_id:
                    continue
                outcome = event.get('outcome', '')
                if outcome in ('COMPLETED', 'FAILED', 'AUTH_FAIL',
                               'LOCATION_MISMATCH', 'SERVER_UNAVAILABLE'):
                    return {'outcome': outcome, 'summary': event.get('summary', ''),
                            'waited_s': round(self.nav['task_wait_timeout_s']
                                              - (deadline - time.time()), 2)}
        return {'outcome': 'NO_DISPATCH',
                'summary': 'dispatcher reported nothing for this station',
                'waited_s': round(self.nav['task_wait_timeout_s'], 2)}

    # ═════════════════════════════════════════════════════════════════════════
    # Tour
    # ═════════════════════════════════════════════════════════════════════════
    def _await_odometry(self) -> None:
        self.get_logger().info('Waiting for /odom…')
        deadline = time.time() + self.nav['odom_wait_s']
        while time.time() < deadline and not self.odom_received:
            self._tick(0.1)
        if not self.odom_received:
            self.simulated_odometry = True
            self._last_integration_t = time.time()
            self.get_logger().warn(
                f'No /odom within {self.nav["odom_wait_s"]:.0f} s — running with '
                f'integrated command odometry (headless mode). The controller, '
                f'planner and watchdog behave identically.')

    def _preflight(self) -> bool:
        """Refuse to start a tour whose berths are not reachable on paper."""
        ok = True
        occupied = self.planner.occupied_by(self.home_pose[:2])
        if occupied:
            self.get_logger().warn(
                f'Home pose {self.home_pose[:2]} lies inside the inflated footprint of '
                f'{", ".join(occupied)}; departure will use reduced clearance.')

        cursor = (self.home_pose[0], self.home_pose[1])
        for station in self.stations:
            berth = station['berth']
            blocked = self.planner.occupied_by(berth)
            if blocked:
                self.get_logger().error(
                    f'Berth for {station["name"]} at {berth} is inside obstacle(s) '
                    f'{", ".join(blocked)} — check station_registry.yaml.')
                ok = False
            plan = self.planner.plan(cursor, berth)
            if plan.fallback:
                self.get_logger().error(
                    f'No route from {cursor} to {station["name"]} at {berth}.')
                ok = False
            cursor = berth
        return ok

    def run_tour(self) -> bool:
        self._await_odometry()
        self._emit('tour_start', stations=[s['name'] for s in self.stations])

        if not self._preflight():
            self.get_logger().error('Pre-flight route check FAILED — aborting tour.')
            return False

        results = []
        tour_started = time.time()

        for index, station in enumerate(self.stations, start=1):
            berth = station['berth']
            label = f'LEG {index}/{len(self.stations)} → {station["name"]}'
            self.get_logger().info(
                '\n' + '═' * 68 + f'\n  {label}\n'
                f'  berth [{berth[0]:.2f}, {berth[1]:.2f}]  '
                f'approach yaw {math.degrees(station["approach_yaw"]):+.0f}°\n' + '═' * 68)

            leg_started = time.time()
            leg = self.execute_leg(berth, label)

            error = math.dist(self.xy, berth)
            aligned = self.rotate_to(station['approach_yaw']) if leg['reached'] else False
            yaw_error = abs(normalize_angle(station['approach_yaw'] - self.yaw))

            within_tolerance = error <= station['tolerance']
            berthed = leg['reached'] and within_tolerance

            if berthed:
                self.get_logger().info(
                    f'  BERTHED at {station["name"]} — pose [{self.x:.2f}, {self.y:.2f}] '
                    f'({error * 100:.1f} cm off, yaw {math.degrees(yaw_error):+.1f}°). '
                    f'Awaiting dispatch…')
                task = self.wait_for_task(station['marker_id'])
            else:
                self.get_logger().error(
                    f'  FAILED to berth at {station["name"]} — stopped at '
                    f'[{self.x:.2f}, {self.y:.2f}], {error:.2f} m from target '
                    f'(tolerance {station["tolerance"]:.2f} m).')
                task = {'outcome': 'NOT_ATTEMPTED', 'summary': 'berth not reached',
                        'waited_s': 0.0}

            results.append({
                'marker_id': station['marker_id'],
                'station': station['name'],
                'berth': [round(berth[0], 3), round(berth[1], 3)],
                'final_pose': [round(self.x, 3), round(self.y, 3), round(self.yaw, 4)],
                'position_error_m': round(error, 3),
                'yaw_error_deg': round(math.degrees(yaw_error), 2),
                'within_tolerance': within_tolerance,
                'aligned': aligned,
                'reached': leg['reached'],
                'recoveries': leg['recoveries'],
                'plan': leg['plan'],
                'dispatch_outcome': task['outcome'],
                'dispatch_summary': task['summary'],
                'leg_duration_s': round(time.time() - leg_started, 2),
            })
            self._emit('station_done', **results[-1])

        returned_home = None
        if self.nav.get('return_home', True):
            self.get_logger().info('\n' + '═' * 68 +
                                   '\n  RETURN LEG → staging dock\n' + '═' * 68)
            home_leg = self.execute_leg((self.home_pose[0], self.home_pose[1]),
                                        'RETURN → staging dock')
            self.rotate_to(self.home_pose[2])
            returned_home = home_leg['reached']

        report = self._build_report(results, time.time() - tour_started, returned_home)
        self._write_report(report)
        self._print_report(report)
        self._emit('tour_complete', passed=report['passed'],
                   stations_ok=report['stations_berthed'])
        return report['passed']

    # ── Reporting ─────────────────────────────────────────────────────────────
    def _build_report(self, results: list[dict], duration: float,
                      returned_home: bool | None) -> dict:
        berthed = sum(1 for r in results if r['within_tolerance'] and r['reached'])
        completed = sum(1 for r in results if r['dispatch_outcome'] == 'COMPLETED')
        errors = [r['position_error_m'] for r in results if r['reached']]
        return {
            'generated_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
            'odometry_source': 'integrated_command' if self.simulated_odometry else 'gazebo_odom',
            'stations_total': len(results),
            'stations_berthed': berthed,
            'stations_completed': completed,
            'total_recoveries': sum(r['recoveries'] for r in results),
            'worst_position_error_m': round(max(errors), 3) if errors else None,
            'mean_position_error_m': round(sum(errors) / len(errors), 3) if errors else None,
            'returned_home': returned_home,
            'duration_s': round(duration, 2),
            'passed': berthed == len(results) and completed == len(results),
            'stations': results,
        }

    def _write_report(self, report: dict) -> None:
        path = os.path.abspath(self.get_parameter('report_path').value)
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, 'w') as handle:
                json.dump(report, handle, indent=2)
            self.get_logger().info(f'Tour report written to {path}')
        except OSError as exc:
            self.get_logger().error(f'Could not write tour report to {path}: {exc}')

    def _print_report(self, report: dict) -> None:
        rows = [
            '',
            '═' * 96,
            '  WAYPOINT TOUR REPORT',
            '═' * 96,
            f'  {"ID":<3}{"Station":<22}{"Berthed":<9}{"Err (cm)":>9}'
            f'{"Yaw (°)":>9}{"Recov":>7}  {"Dispatch":<18}Path',
            '  ' + '─' * 92,
        ]
        for row in report['stations']:
            rows.append(
                f'  {row["marker_id"]:<3}{row["station"]:<22}'
                f'{("yes" if row["within_tolerance"] and row["reached"] else "NO"):<9}'
                f'{row["position_error_m"] * 100:>9.1f}{row["yaw_error_deg"]:>9.1f}'
                f'{row["recoveries"]:>7}  {row["dispatch_outcome"]:<18}'
                f'{row["plan"][:28]}')
        rows += [
            '  ' + '─' * 92,
            f'  berthed {report["stations_berthed"]}/{report["stations_total"]}   '
            f'dispatch COMPLETED {report["stations_completed"]}/{report["stations_total"]}   '
            f'recoveries {report["total_recoveries"]}   '
            f'worst error {report["worst_position_error_m"]} m   '
            f'{report["duration_s"]} s',
            f'  odometry: {report["odometry_source"]}   '
            f'returned home: {report["returned_home"]}',
            f'  OVERALL: {"PASS" if report["passed"] else "FAIL"}',
            '═' * 96,
            '',
        ]
        self.get_logger().info('\n'.join(rows))


def main(args=None):
    rclpy.init(args=args)
    node = WaypointTourNode()
    passed = False
    try:
        passed = node.run_tour()
    except KeyboardInterrupt:
        node.get_logger().warn('Tour interrupted — stopping robot.')
    finally:
        try:
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()
