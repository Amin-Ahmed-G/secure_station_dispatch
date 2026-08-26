#!/usr/bin/env python3
"""
test_tour_simulation.py
───────────────────────
Replays the entire station tour offline — planner, control law, stall watchdog
and a unicycle model of the AMR — with no ROS and no Gazebo::

    python3 test/test_tour_simulation.py
    pytest test/test_tour_simulation.py

The simulation ticks at the node's real 20 Hz control rate and checks the robot
footprint against the *hard* obstacle inflation on every single tick, so a
regression that would drive into a station bay fails here rather than in the
demo.  It also injects a synthetic obstruction to prove the stall watchdog and
the back-up recovery still get the robot out of a wedge — the failure mode that
originally ended the tour at the second marker.
"""

from __future__ import annotations

import math
import os
import sys

import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

from secure_station_dispatch.navigation import (          # noqa: E402
    ControlParams, DiffDriveController, PathPlanner, Rect, StallWatchdog,
    normalize_angle, obstacles_from_registry,
)

REGISTRY_PATH = os.path.join(REPO, 'config', 'station_registry.yaml')
DT = 0.05                     # matches CONTROL_PERIOD in waypoint_tour_node
MAX_TICKS_PER_SEGMENT = 1600  # 80 s of simulated time


def load_registry() -> dict:
    with open(REGISTRY_PATH) as handle:
        return yaml.safe_load(handle)


class UnicycleRobot:
    """Ideal differential-drive integrator, optionally blocked by a barrier.

    *barrier* stands in for a real collision: while the robot's next pose would
    enter it, translation is refused but rotation is not — which is exactly how
    a robot behaves when it is pressed against a panel with its wheels turning.
    """

    def __init__(self, x: float, y: float, yaw: float, barrier: Rect | None = None):
        self.x, self.y, self.yaw = x, y, yaw
        self.barrier = barrier
        self.path: list[tuple[float, float]] = [(x, y)]

    @property
    def xy(self) -> tuple[float, float]:
        return (self.x, self.y)

    def step(self, linear: float, angular: float, dt: float = DT) -> None:
        self.yaw = normalize_angle(self.yaw + angular * dt)
        nx = self.x + linear * math.cos(self.yaw) * dt
        ny = self.y + linear * math.sin(self.yaw) * dt
        if self.barrier is None or not self.barrier.contains((nx, ny)):
            self.x, self.y = nx, ny
        self.path.append((self.x, self.y))


def make_planner(registry: dict) -> PathPlanner:
    nav = registry['navigation']
    return PathPlanner(
        obstacles=obstacles_from_registry(registry),
        inflation=nav['robot_radius_m'] + nav['safety_margin_m'],
        hard_inflation=nav['robot_radius_m'],
        bounds=registry['world']['bounds'],
    )


def drive_segment(robot, controller, watchdog, target, clock, on_tick=None):
    """Mirror of WaypointTourNode.drive_to_point, driving the simulated robot."""
    watchdog.reset(clock[0], robot.xy)
    for _ in range(MAX_TICKS_PER_SEGMENT):
        command = controller.command(robot.x, robot.y, robot.yaw, target)
        if command.arrived:
            return 'arrived'
        robot.step(command.linear, command.angular)
        clock[0] += DT
        if on_tick is not None:
            on_tick(robot)
        if watchdog.update(clock[0], robot.xy, command.is_moving):
            return 'stalled'
    return 'timeout'


def rotate(robot, controller, target_yaw, clock, limit=600) -> bool:
    for _ in range(limit):
        command = controller.rotate_command(robot.yaw, target_yaw)
        if command.arrived:
            return True
        robot.step(command.linear, command.angular)
        clock[0] += DT
    return False


# ═══════════════════════════════════════════════════════════════════════════════
def test_full_tour_completes_without_touching_anything():
    registry = load_registry()
    planner = make_planner(registry)
    controller = DiffDriveController(ControlParams.from_registry(registry['navigation']))
    watchdog = StallWatchdog(registry['navigation']['stall_window_s'],
                             registry['navigation']['stall_min_progress_m'])

    home = tuple(registry['world']['home_pose'][:3])
    robot = UnicycleRobot(home[0], home[1], home[2] if len(home) > 2 else 0.0)
    clock = [0.0]

    contacts: list[tuple[str, tuple[float, float]]] = []

    def check_contact(bot):
        for name in planner.occupied_by(bot.xy, inflation=planner.hard_inflation):
            contacts.append((name, (round(bot.x, 2), round(bot.y, 2))))

    goal_tolerance = registry['navigation']['goal_tolerance_m']
    results = []

    itinerary = sorted(registry['stations'].items())
    for marker_id, station in itinerary:
        berth = tuple(station['expected_pose'][:2])
        plan = planner.plan(robot.xy, berth)
        assert not plan.fallback, f'marker {marker_id}: planner found no route'

        for waypoint in plan.waypoints:
            status = drive_segment(robot, controller, watchdog, waypoint, clock,
                                   on_tick=check_contact)
            assert status == 'arrived', \
                f'marker {marker_id}: segment to {waypoint} ended {status}'

        assert rotate(robot, controller, station['approach_yaw'], clock), \
            f'marker {marker_id}: failed to settle on the approach heading'

        error = math.dist(robot.xy, berth)
        yaw_error = abs(normalize_angle(station['approach_yaw'] - robot.yaw))
        results.append((marker_id, error, yaw_error))

        assert error <= goal_tolerance + 1e-6, (
            f'marker {marker_id}: berthed {error * 100:.1f} cm off, tolerance is '
            f'{goal_tolerance * 100:.0f} cm')
        assert error <= station['pose_tolerance_m'], (
            f'marker {marker_id}: berth would fail the security pose check')

    if registry['navigation'].get('return_home', True):
        plan = planner.plan(robot.xy, (home[0], home[1]))
        assert not plan.fallback, 'no route back to the staging dock'
        for waypoint in plan.waypoints:
            assert drive_segment(robot, controller, watchdog, waypoint, clock,
                                 on_tick=check_contact) == 'arrived', \
                'return leg did not complete'

    assert not contacts, f'robot entered an obstacle footprint: {contacts[:5]}'
    assert clock[0] < 400.0, f'tour took {clock[0]:.0f} s of simulated time'

    test_full_tour_completes_without_touching_anything.summary = {
        'duration_s': round(clock[0], 1),
        'path_points': len(robot.path),
        'worst_error_cm': round(max(e for _, e, _ in results) * 100, 1),
        'worst_yaw_deg': round(math.degrees(max(y for _, _, y in results)), 2),
    }


def test_watchdog_fires_when_the_robot_is_blocked():
    """A robot pressed against a panel must abort, not grind out the timeout."""
    registry = load_registry()
    controller = DiffDriveController(ControlParams.from_registry(registry['navigation']))
    watchdog = StallWatchdog(registry['navigation']['stall_window_s'],
                             registry['navigation']['stall_min_progress_m'])

    # A wall 0.5 m ahead of the robot, with the goal on the far side of it.
    barrier = Rect('panel', 1.0, 0.0, 0.10, 4.0)
    robot = UnicycleRobot(0.0, 0.0, 0.0, barrier=barrier)
    clock = [0.0]

    status = drive_segment(robot, controller, watchdog, (3.0, 0.0), clock)
    assert status == 'stalled', f'expected a stall, got {status}'
    assert clock[0] < registry['navigation']['segment_timeout_s'], (
        'the watchdog must abort well before the segment timeout — grinding for '
        'the full timeout is what corrupted every later leg of the old tour')


def test_recovery_frees_a_wedged_robot_and_the_replan_succeeds():
    """Back up, turn, re-plan: the robot must still reach a reachable goal."""
    registry = load_registry()
    nav = registry['navigation']
    controller = DiffDriveController(ControlParams.from_registry(nav))
    watchdog = StallWatchdog(nav['stall_window_s'], nav['stall_min_progress_m'])

    barrier = Rect('panel', 1.0, 0.0, 0.10, 1.2)      # narrow: can be gone around
    planner = PathPlanner([barrier], inflation=0.43, hard_inflation=0.28)
    robot = UnicycleRobot(0.0, 0.0, 0.0, barrier=barrier)
    clock = [0.0]
    goal = (3.0, 0.0)

    # Force the wedge by driving the blocked straight line first.
    assert drive_segment(robot, controller, watchdog, goal, clock) == 'stalled'

    # Recovery: reverse, then turn.
    start = robot.xy
    for _ in range(400):
        robot.step(-nav['recovery_backup_speed'], 0.0)
        clock[0] += DT
        if math.dist(robot.xy, start) >= nav['recovery_backup_m']:
            break
    assert math.dist(robot.xy, start) >= nav['recovery_backup_m'] - 1e-3, \
        'the robot could not back away from the obstruction'
    rotate(robot, controller, normalize_angle(robot.yaw + nav['recovery_turn_rad']), clock)

    # Re-plan from the freed pose and drive it.
    plan = planner.plan(robot.xy, goal)
    assert not plan.fallback and not plan.direct, \
        'the re-plan should route around the panel, not back into it'
    for waypoint in plan.waypoints:
        assert drive_segment(robot, controller, watchdog, waypoint, clock) == 'arrived', \
            f'recovery re-plan failed on the way to {waypoint}'
    assert math.dist(robot.xy, goal) <= nav['goal_tolerance_m'] + 1e-6


def test_controller_does_not_arc_through_a_corner_it_was_routed_around():
    """Large heading errors must be turned out on the spot, not arced away."""
    controller = DiffDriveController(ControlParams())
    command = controller.command(0.0, 0.0, 0.0, (0.0, 2.0))   # 90° to the left
    assert command.linear == 0.0, 'the robot should stop and turn, not arc'
    assert command.angular > 0.0

    command = controller.command(0.0, 0.0, 0.0, (2.0, 0.05))  # nearly straight
    assert command.linear > 0.0 and abs(command.angular) < 0.2


def test_controller_decelerates_into_the_goal():
    controller = DiffDriveController(ControlParams())
    far = controller.command(0.0, 0.0, 0.0, (5.0, 0.0)).linear
    near = controller.command(0.0, 0.0, 0.0, (0.3, 0.0)).linear
    assert far > near, 'linear speed must fall off as the goal approaches'
    assert near >= ControlParams().min_linear_speed - 1e-9


def test_watchdog_ignores_time_spent_stationary_on_purpose():
    """Holding a berth for 30 s must not be mistaken for a stall."""
    watchdog = StallWatchdog(window_s=3.0, min_progress_m=0.04)
    watchdog.reset(0.0, (0.0, 0.0))
    for tick in range(600):
        assert not watchdog.update(tick * DT, (0.0, 0.0), commanding_motion=False)


# ═══════════════════════════════════════════════════════════════════════════════
def main() -> int:
    tests = [(n, o) for n, o in sorted(globals().items())
             if n.startswith('test_') and callable(o)]
    width = max(len(n) for n, _ in tests)

    print('=' * (width + 22))
    print('  OFFLINE TOUR SIMULATION SUITE')
    print('=' * (width + 22))

    failures = []
    for name, fn in tests:
        try:
            fn()
        except AssertionError as exc:
            failures.append((name, str(exc)))
            print(f'  {name:<{width}}  ✗ FAIL')
        except Exception as exc:                        # noqa: BLE001
            failures.append((name, f'{type(exc).__name__}: {exc}'))
            print(f'  {name:<{width}}  ✗ ERROR')
        else:
            extra = getattr(fn, 'summary', None)
            print(f'  {name:<{width}}  ✓ pass' + (f'   {extra}' if extra else ''))

    print('-' * (width + 22))
    for name, message in failures:
        print(f'\n  {name}\n    {message}')
    print(f'\n  {len(tests) - len(failures)}/{len(tests)} passed — '
          f'OVERALL: {"PASS" if not failures else "FAIL"}')
    print('=' * (width + 22))
    return 0 if not failures else 1


if __name__ == '__main__':
    raise SystemExit(main())
