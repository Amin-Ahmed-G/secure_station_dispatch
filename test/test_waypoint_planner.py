#!/usr/bin/env python3
"""
test_waypoint_planner.py
────────────────────────
Regression suite for the Phase 3 navigation fix.  No ROS, no Gazebo — it runs
against the shipped ``config/station_registry.yaml`` in plain Python::

    python3 test/test_waypoint_planner.py     # standalone, prints a summary
    pytest test/test_waypoint_planner.py      # or under pytest

What it pins
────────────
1.  The original defect: with the old world geometry — a marker panel 0.40 m in
    +Y of every berth — the straight line from the Assembly berth to the Storage
    berth passes through the Assembly panel.  The planner must *see* that and
    route around it.  This is the test that would have caught the bug.
2.  The shipped layout: every leg of the tour, including the return to the dock,
    plans to a collision-free polyline at full safety clearance.
3.  The detection budget: each board is legible from its own berth and illegible
    from anywhere the robot merely passes, so no station can be dispatched by a
    robot driving past it.
4.  World/registry agreement, via scripts/check_world_sync.py.
"""

from __future__ import annotations

import math
import os
import sys

import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, 'scripts'))

from secure_station_dispatch.navigation import (           # noqa: E402
    PathPlanner, Rect, obstacles_from_registry, segment_intersects_rect,
)
from secure_station_dispatch.navigation.perception_budget import (  # noqa: E402
    MAST_CAMERA, board_fits_in_frame, detection_window, marker_confidence,
    marker_in_view,
)

REGISTRY_PATH = os.path.join(REPO, 'config', 'station_registry.yaml')


def load_registry() -> dict:
    with open(REGISTRY_PATH) as handle:
        return yaml.safe_load(handle)


def make_planner(registry: dict) -> PathPlanner:
    nav = registry['navigation']
    return PathPlanner(
        obstacles=obstacles_from_registry(registry),
        inflation=nav['robot_radius_m'] + nav['safety_margin_m'],
        hard_inflation=nav['robot_radius_m'],
        bounds=registry['world']['bounds'],
    )


def itinerary(registry: dict) -> list[tuple[int, dict]]:
    return sorted(registry['stations'].items())


def tour_legs(registry: dict) -> list[tuple[str, tuple, tuple]]:
    """(label, start, goal) for every leg the tour drives, dock included."""
    home = tuple(registry['world']['home_pose'][:2])
    legs, cursor = [], home
    for marker_id, station in itinerary(registry):
        berth = tuple(station['expected_pose'][:2])
        legs.append((f'{marker_id}:{station["station_name"]}', cursor, berth))
        cursor = berth
    if registry['navigation'].get('return_home', True):
        legs.append(('return:staging dock', cursor, home))
    return legs


# ═══════════════════════════════════════════════════════════════════════════════
# 1 · Geometry primitives
# ═══════════════════════════════════════════════════════════════════════════════
def test_segment_rect_intersection():
    rect = Rect('r', 0.0, 0.0, 2.0, 2.0)          # x,y in [-1, 1]

    assert segment_intersects_rect((-2, 0), (2, 0), rect), 'straight through'
    assert segment_intersects_rect((0, 0), (0.5, 0.5), rect), 'wholly inside'
    assert segment_intersects_rect((-2, -2), (0, 0), rect), 'diagonal entry'
    assert segment_intersects_rect((-2, 1), (2, 1), rect), 'grazes the top edge'
    assert not segment_intersects_rect((-2, 1.01), (2, 1.01), rect), 'just above'
    assert not segment_intersects_rect((2, 2), (3, 3), rect), 'clear of the box'
    assert not segment_intersects_rect((-2, -2), (-1.5, -1.5), rect), 'stops short'
    # A vertical segment is the degenerate case the slab method must special-case.
    assert segment_intersects_rect((0, -3), (0, 3), rect), 'vertical through'
    assert not segment_intersects_rect((1.5, -3), (1.5, 3), rect), 'vertical beside'


def test_rect_inflation_and_distance():
    rect = Rect('r', 0.0, 0.0, 1.0, 1.0)
    grown = rect.inflated(0.25)
    assert math.isclose(grown.dx, 1.5) and math.isclose(grown.dy, 1.5)
    assert math.isclose(rect.distance_to((0.0, 0.0)), 0.0)
    assert math.isclose(rect.distance_to((1.5, 0.0)), 1.0)
    assert math.isclose(rect.distance_to((1.5, 1.5)), math.hypot(1.0, 1.0))


# ═══════════════════════════════════════════════════════════════════════════════
# 2 · The original defect
# ═══════════════════════════════════════════════════════════════════════════════
def test_original_collision_is_detected_and_routed_around():
    """The exact geometry that wedged the robot, replayed against the planner.

    Old world: every station carried a 0.60 x 0.01 x 0.60 m panel with a solid
    collision box, centred 0.40 m in +Y of the berth.  The tour then drove a
    straight line from Assembly [4.5, 1.0] to Storage [4.5, 5.0] — right through
    the Assembly panel at [4.5, 1.4].
    """
    legacy_berths = {0: (1.0, 2.0), 1: (4.5, 1.0), 2: (4.5, 5.0), 3: (0.5, 5.5)}
    legacy_panels = [
        Rect(f'legacy_panel_{i}', x, y + 0.40, 0.60, 0.01)
        for i, (x, y) in legacy_berths.items()
    ]
    planner = PathPlanner(legacy_panels, inflation=0.43, hard_inflation=0.28)

    start, goal = legacy_berths[1], legacy_berths[2]

    # The straight line really does hit the Assembly panel — this is the bug.
    blockers = planner.blockers(start, goal)
    assert 'legacy_panel_1' in blockers, (
        'the Assembly->Storage straight line should intersect the Assembly panel; '
        f'got blockers={blockers}')

    # And with the planner in the loop it is no longer driven.
    plan = planner.plan(start, goal)
    assert not plan.direct, 'planner must not return the colliding straight line'
    assert not plan.fallback, 'a detour around a 0.6 m panel must exist'

    # The legacy berth sits 0.40 m from its own panel, i.e. inside that panel's
    # safety halo but clear of its hard footprint.  The route is therefore
    # allowed to skim the halo on departure, but must never touch the structure.
    exempt = tuple(set(planner.occupied_by(start)) | set(planner.occupied_by(goal)))
    assert 'legacy_panel_1' in exempt, 'the legacy berth should sit inside its own halo'
    for a, b in zip([start] + plan.waypoints, plan.waypoints):
        assert not planner.blockers(a, b, ignore=exempt), \
            f'detour leg {a}->{b} still collides'
        assert not planner.blockers(a, b, inflation=planner.hard_inflation), \
            f'detour leg {a}->{b} enters a panel\'s hard footprint'


def test_current_layout_would_not_reproduce_the_bug():
    """The re-laid world removes the hazard as well as guarding against it."""
    registry = load_registry()
    planner = make_planner(registry)
    stations = dict(registry['stations'])
    assembly = tuple(stations[1]['expected_pose'][:2])
    storage = tuple(stations[2]['expected_pose'][:2])

    bays = [n for n in planner.blockers(assembly, storage) if n.startswith('bay_')]
    assert not bays, f'a station bay is back on the Assembly->Storage line: {bays}'


# ═══════════════════════════════════════════════════════════════════════════════
# 3 · The shipped layout
# ═══════════════════════════════════════════════════════════════════════════════
def test_berths_and_dock_are_clear():
    registry = load_registry()
    planner = make_planner(registry)

    home = tuple(registry['world']['home_pose'][:2])
    assert planner.point_is_free(home), \
        f'staging dock {home} sits inside {planner.occupied_by(home)}'

    for marker_id, station in itinerary(registry):
        berth = tuple(station['expected_pose'][:2])
        occupied = planner.occupied_by(berth)
        assert not occupied, \
            f'berth {berth} for marker {marker_id} sits inside {occupied}'


def test_every_leg_plans_to_a_collision_free_route():
    registry = load_registry()
    planner = make_planner(registry)

    for label, start, goal in tour_legs(registry):
        plan = planner.plan(start, goal)
        assert not plan.fallback, f'{label}: no collision-free route found'
        assert not plan.degraded, f'{label}: needed reduced clearance'

        exempt = tuple(set(planner.occupied_by(start)) | set(planner.occupied_by(goal)))
        for a, b in zip([start] + plan.waypoints, plan.waypoints):
            hits = planner.blockers(a, b, ignore=exempt)
            assert not hits, f'{label}: planned segment {a}->{b} clips {hits}'


def test_rack_forces_a_detour_on_the_assembly_to_storage_leg():
    """The centre rack sits on the aisle, so that leg must actually be planned."""
    registry = load_registry()
    planner = make_planner(registry)
    stations = dict(registry['stations'])
    start = tuple(stations[1]['expected_pose'][:2])
    goal = tuple(stations[2]['expected_pose'][:2])

    plan = planner.plan(start, goal)
    assert 'rack_centre' in plan.blocking, \
        f'expected rack_centre to block the aisle, got {plan.blocking}'
    assert not plan.direct and len(plan.waypoints) > 1, \
        'the blocked aisle should produce intermediate waypoints'
    assert plan.length < 2.5 * math.dist(start, goal), \
        'the detour is implausibly long — check the visibility graph'


def test_shortcutting_removes_redundant_waypoints():
    """A corridor with one obstacle should need one turn, not four."""
    obstacles = [Rect('block', 0.0, 0.0, 1.0, 4.0)]
    planner = PathPlanner(obstacles, inflation=0.2)
    plan = planner.plan((-3.0, 0.0), (3.0, 0.0))
    assert not plan.direct
    assert len(plan.waypoints) <= 3, f'expected a tight detour, got {plan.waypoints}'


def test_unreachable_goal_reports_fallback_instead_of_lying():
    """A goal sealed inside a box must be reported, not silently 'planned'."""
    walls = [
        Rect('n', 0.0, 1.0, 2.2, 0.2), Rect('s', 0.0, -1.0, 2.2, 0.2),
        Rect('e', 1.0, 0.0, 0.2, 2.2), Rect('w', -1.0, 0.0, 0.2, 2.2),
    ]
    planner = PathPlanner(walls, inflation=0.05)
    plan = planner.plan((5.0, 5.0), (0.0, 0.0))
    assert plan.fallback, 'a sealed goal must be flagged, not returned as a route'
    assert plan.blocking, 'the fallback should name what blocked it'


# ═══════════════════════════════════════════════════════════════════════════════
# 4 · Detection budget
# ═══════════════════════════════════════════════════════════════════════════════
def test_marker_is_legible_from_its_own_berth():
    registry = load_registry()
    board = registry['world']['marker_board']
    gate = registry['min_confidence']
    d_berth = board['standoff_m'] - MAST_CAMERA.forward_offset_m

    confidence = marker_confidence(board['marker_size_m'], d_berth)
    assert confidence >= gate, \
        f'berth confidence {confidence:.2f} is under the {gate:.2f} gate'
    assert board_fits_in_frame(board['board_size_m'], d_berth), (
        'the carrier board overflows the frame at the berth — OpenCV discards '
        'candidate quads that touch the image border')

    near, far = detection_window(board['marker_size_m'], board['board_size_m'], gate)
    assert near < d_berth < far, \
        f'berth range {d_berth:.2f} m is outside the detection window [{near:.2f}, {far:.2f}]'


def test_camera_height_matches_the_board_height():
    registry = load_registry()
    board = registry['world']['marker_board']
    assert math.isclose(MAST_CAMERA.height_m, board['board_height_m'], abs_tol=1e-6), (
        'the mast camera and the marker boards must be at the same height, '
        'otherwise the board is framed off-centre and clips at close range')


def test_no_station_can_be_dispatched_while_merely_driving_past():
    """Sample every planned leg and assert only the destination bay is legible.

    Without this the robot would fire a Storage task halfway up the aisle, the
    security pose check would reject it, and the event log would fill with
    spurious LOCATION_MISMATCH entries.
    """
    registry = load_registry()
    planner = make_planner(registry)
    board = registry['world']['marker_board']
    gate = registry['min_confidence']
    stations = dict(registry['stations'])

    def legible(pose, yaw, station) -> bool:
        visible, distance, _ = marker_in_view(
            pose, yaw,
            (station['marker_pose'][0], station['marker_pose'][1]),
            (station['marker_normal'][0], station['marker_normal'][1]))
        return visible and marker_confidence(board['marker_size_m'], distance) >= gate

    offenders = []
    for label, start, goal in tour_legs(registry):
        destination = None
        for marker_id, station in stations.items():
            if math.dist(tuple(station['expected_pose'][:2]), goal) < 1e-6:
                destination = marker_id

        plan = planner.plan(start, goal)
        for a, b in zip([start] + plan.waypoints, plan.waypoints):
            heading = math.atan2(b[1] - a[1], b[0] - a[0])
            steps = max(2, int(math.dist(a, b) / 0.10))
            for i in range(steps + 1):
                t = i / steps
                pose = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
                # Stop sampling once we are berthing on the destination.
                if destination is not None and math.dist(pose, goal) < 0.30:
                    continue
                for marker_id, station in stations.items():
                    if marker_id == destination:
                        continue
                    if legible(pose, heading, station):
                        offenders.append((label, marker_id, tuple(round(v, 2) for v in pose)))

    assert not offenders, f'stations legible from transit poses: {offenders[:6]}'


def test_each_station_is_legible_once_berthed():
    registry = load_registry()
    board = registry['world']['marker_board']
    gate = registry['min_confidence']

    for marker_id, station in itinerary(registry):
        berth = tuple(station['expected_pose'][:2])
        yaw = station['approach_yaw']
        visible, distance, bearing = marker_in_view(
            berth, yaw,
            (station['marker_pose'][0], station['marker_pose'][1]),
            (station['marker_normal'][0], station['marker_normal'][1]))
        assert visible, f'marker {marker_id} not in view from its own berth'
        assert abs(bearing) < 1e-3, \
            f'marker {marker_id} is {math.degrees(bearing):.1f}° off the optical axis'
        confidence = marker_confidence(board['marker_size_m'], distance)
        assert confidence >= gate, \
            f'marker {marker_id} scores {confidence:.2f} at its berth (gate {gate:.2f})'


def test_navigation_tolerance_keeps_the_marker_legible():
    """Anywhere the controller may legally stop, the board must still read.

    The relevant envelope is the *navigation* goal tolerance, not the much wider
    security ``pose_tolerance_m``: the controller stops within
    ``goal_tolerance_m`` of the berth, and both edges of that band have to stay
    inside the detection window — too far and the score drops through the gate,
    too near and the carrier board clips the frame.
    """
    registry = load_registry()
    board = registry['world']['marker_board']
    gate = registry['min_confidence']
    slack = registry['navigation']['goal_tolerance_m']

    for marker_id, _station in itinerary(registry):
        far = board['standoff_m'] + slack - MAST_CAMERA.forward_offset_m
        near = board['standoff_m'] - slack - MAST_CAMERA.forward_offset_m

        confidence = marker_confidence(board['marker_size_m'], far)
        assert confidence >= gate, (
            f'marker {marker_id}: stopping {slack} m short scores {confidence:.2f}, '
            f'under the {gate:.2f} gate')
        assert board_fits_in_frame(board['board_size_m'], near), (
            f'marker {marker_id}: overshooting by {slack} m clips the board '
            f'against the image border')


def test_security_tolerance_is_wider_than_the_navigation_tolerance():
    """The security radius must never be the thing that fails a good berth."""
    registry = load_registry()
    goal_tolerance = registry['navigation']['goal_tolerance_m']
    for marker_id, station in itinerary(registry):
        assert station['pose_tolerance_m'] > goal_tolerance, (
            f'marker {marker_id}: pose_tolerance_m {station["pose_tolerance_m"]} is '
            f'not wider than goal_tolerance_m {goal_tolerance}, so a correctly '
            f'berthed robot could be rejected as a location mismatch')


# ═══════════════════════════════════════════════════════════════════════════════
# 5 · World / registry agreement
# ═══════════════════════════════════════════════════════════════════════════════
def test_world_matches_registry():
    from check_world_sync import check
    problems = check()
    assert not problems, 'world/registry drift:\n  ' + '\n  '.join(problems)


def test_every_obstacle_in_the_registry_is_inside_the_facility():
    registry = load_registry()
    bounds = registry['world']['bounds']
    for rect in obstacles_from_registry(registry):
        assert rect.max_x >= bounds['x_min'] - 0.5 and rect.min_x <= bounds['x_max'] + 0.5, \
            f'{rect.name} is outside the facility in X'
        assert rect.max_y >= bounds['y_min'] - 0.5 and rect.min_y <= bounds['y_max'] + 0.5, \
            f'{rect.name} is outside the facility in Y'


# ═══════════════════════════════════════════════════════════════════════════════
def main() -> int:
    tests = [(name, obj) for name, obj in sorted(globals().items())
             if name.startswith('test_') and callable(obj)]

    width = max(len(name) for name, _ in tests)
    print('=' * (width + 22))
    print('  WAYPOINT PLANNER / PERCEPTION REGRESSION SUITE')
    print('=' * (width + 22))

    failures = []
    for name, fn in tests:
        try:
            fn()
        except AssertionError as exc:
            failures.append((name, str(exc)))
            print(f'  {name:<{width}}  ✗ FAIL')
        except Exception as exc:                       # noqa: BLE001
            failures.append((name, f'{type(exc).__name__}: {exc}'))
            print(f'  {name:<{width}}  ✗ ERROR')
        else:
            print(f'  {name:<{width}}  ✓ pass')

    print('-' * (width + 22))
    for name, message in failures:
        print(f'\n  {name}\n    {message}')
    print(f'\n  {len(tests) - len(failures)}/{len(tests)} passed — '
          f'OVERALL: {"PASS" if not failures else "FAIL"}')
    print('=' * (width + 22))
    return 0 if not failures else 1


if __name__ == '__main__':
    raise SystemExit(main())
