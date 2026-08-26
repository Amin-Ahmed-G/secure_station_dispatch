#!/usr/bin/env python3
"""
planner.py
──────────
Obstacle-aware path planning for the scripted station tour.

Why this module exists
──────────────────────
The Phase 3 tour originally drove a straight line from one berth to the next.
That is safe only if no station structure ever sits between two berths — and it
did: the leg from the Assembly berth [4.5, 1.0] to the Storage berth [4.5, 5.0]
ran straight through the Assembly marker panel, which carried a solid collision
box.  The robot wedged against it, ``drive_to`` kept commanding forward velocity
until its timeout, and every subsequent leg started from a corrupted pose.

The fix is to stop assuming free space.  Obstacles are declared once in
``config/station_registry.yaml``, inflated by the robot's circumscribed radius
plus a safety margin, and every leg is planned as a shortest collision-free
polyline over the visibility graph of the inflated corners.

Everything here is pure Python + math so it can be exercised by
``test/test_waypoint_planner.py`` without ROS or Gazebo.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field

Point = tuple[float, float]

_EPS = 1e-9


# ═══════════════════════════════════════════════════════════════════════════════
# Geometry
# ═══════════════════════════════════════════════════════════════════════════════
@dataclass(frozen=True)
class Rect:
    """An axis-aligned obstacle box, described by its centre and full extent."""

    name: str
    cx: float
    cy: float
    dx: float
    dy: float

    @property
    def min_x(self) -> float:
        return self.cx - self.dx / 2.0

    @property
    def max_x(self) -> float:
        return self.cx + self.dx / 2.0

    @property
    def min_y(self) -> float:
        return self.cy - self.dy / 2.0

    @property
    def max_y(self) -> float:
        return self.cy + self.dy / 2.0

    def inflated(self, margin: float) -> "Rect":
        return Rect(self.name, self.cx, self.cy,
                    self.dx + 2.0 * margin, self.dy + 2.0 * margin)

    def contains(self, p: Point, tol: float = 0.0) -> bool:
        return (self.min_x - tol <= p[0] <= self.max_x + tol
                and self.min_y - tol <= p[1] <= self.max_y + tol)

    def corners(self, offset: float = 0.0) -> list[Point]:
        """The four corners, optionally pushed diagonally outward by *offset*."""
        return [
            (self.min_x - offset, self.min_y - offset),
            (self.min_x - offset, self.max_y + offset),
            (self.max_x + offset, self.min_y - offset),
            (self.max_x + offset, self.max_y + offset),
        ]

    def distance_to(self, p: Point) -> float:
        """Euclidean distance from *p* to the box (0.0 if inside)."""
        dx = max(self.min_x - p[0], 0.0, p[0] - self.max_x)
        dy = max(self.min_y - p[1], 0.0, p[1] - self.max_y)
        return math.hypot(dx, dy)


def segment_intersects_rect(p: Point, q: Point, rect: Rect) -> bool:
    """True when the closed segment [p, q] meets the closed box *rect*.

    Liang–Barsky slab clipping, clamped to the segment's parameter range.
    """
    t0, t1 = 0.0, 1.0
    for delta, lo, hi, origin in (
        (q[0] - p[0], rect.min_x, rect.max_x, p[0]),
        (q[1] - p[1], rect.min_y, rect.max_y, p[1]),
    ):
        if abs(delta) < _EPS:
            if origin < lo - _EPS or origin > hi + _EPS:
                return False
            continue
        a, b = (lo - origin) / delta, (hi - origin) / delta
        if a > b:
            a, b = b, a
        t0 = max(t0, a)
        t1 = min(t1, b)
        if t0 > t1 + _EPS:
            return False
    return True


# ═══════════════════════════════════════════════════════════════════════════════
# Planner
# ═══════════════════════════════════════════════════════════════════════════════
@dataclass
class PlanResult:
    """Outcome of a single :meth:`PathPlanner.plan` call."""

    waypoints: list[Point] = field(default_factory=list)
    """Intermediate + goal points, excluding the start."""

    direct: bool = True
    """True when the straight line to the goal was already collision-free."""

    inflation_used: float = 0.0
    """Obstacle inflation the accepted plan was produced with."""

    degraded: bool = False
    """True when the full safety inflation had to be relaxed to find a route."""

    fallback: bool = False
    """True when no collision-free route existed and the straight line was used."""

    blocking: tuple[str, ...] = ()
    """Names of the obstacles that blocked the straight line."""

    @property
    def length(self) -> float:
        return self._length_from(self.waypoints)

    def _length_from(self, pts: list[Point]) -> float:
        return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))

    def describe(self, start: Point) -> str:
        if self.fallback:
            return (f'DIRECT (FALLBACK — no clear route, blocked by '
                    f'{", ".join(self.blocking) or "unknown"})')
        if self.direct:
            return 'DIRECT'
        via = ' → '.join(f'[{x:.2f}, {y:.2f}]' for x, y in self.waypoints[:-1])
        tag = 'DETOUR' + (' (reduced clearance)' if self.degraded else '')
        return (f'{tag} around {", ".join(self.blocking)} via {via} '
                f'({self._length_from([start] + self.waypoints):.2f} m)')


class PathPlanner:
    """Shortest collision-free polylines over a visibility graph of inflated corners.

    The obstacle set is small (a dozen boxes), so an O(n^2) visibility graph plus
    A* is both fast enough to run inline between legs and trivial to reason
    about — which matters more here than asymptotic elegance.
    """

    def __init__(
        self,
        obstacles: list[Rect],
        inflation: float,
        hard_inflation: float | None = None,
        corner_clearance: float = 0.06,
        bounds: dict | None = None,
    ) -> None:
        self.obstacles = list(obstacles)
        self.inflation = float(inflation)
        # Two clearance layers.  ``hard_inflation`` is the robot's own footprint
        # radius — crossing it is a real collision.  ``inflation`` adds the
        # safety margin on top; crossing *that* is merely uncomfortable.  The
        # distinction matters because a berth may legitimately sit inside an
        # obstacle's safety halo (the robot has to get close to read the board),
        # and without it the planner would have to exempt the whole obstacle and
        # would happily plan straight through the panel — which is precisely the
        # failure mode this module exists to prevent.
        self.hard_inflation = (self.inflation if hard_inflation is None
                               else float(hard_inflation))
        self.corner_clearance = float(corner_clearance)
        self.bounds = bounds

    # ── Collision queries ─────────────────────────────────────────────────────
    def inflated(self, inflation: float | None = None) -> list[Rect]:
        margin = self.inflation if inflation is None else inflation
        return [o.inflated(margin) for o in self.obstacles]

    def blockers(self, p: Point, q: Point, inflation: float | None = None,
                 ignore: tuple[str, ...] = ()) -> tuple[str, ...]:
        """Names of obstacles the segment [p, q] would clip.

        An obstacle in *ignore* is forgiven for its safety margin only: if the
        segment reaches its hard footprint it still blocks.
        """
        margin = self.inflation if inflation is None else inflation
        hard = min(self.hard_inflation, margin)
        hits: list[str] = []
        for obstacle in self.obstacles:
            if not segment_intersects_rect(p, q, obstacle.inflated(margin)):
                continue
            if obstacle.name in ignore and not segment_intersects_rect(
                    p, q, obstacle.inflated(hard)):
                continue
            hits.append(obstacle.name)
        return tuple(hits)

    def segment_is_free(self, p: Point, q: Point, inflation: float | None = None,
                        ignore: tuple[str, ...] = ()) -> bool:
        return not self.blockers(p, q, inflation, ignore)

    def point_is_free(self, p: Point, inflation: float | None = None) -> bool:
        """True when *p* is in bounds and clear of every inflated obstacle."""
        if self.bounds and not self._in_bounds(p):
            return False
        return not any(r.contains(p) for r in self.inflated(inflation))

    def point_is_colliding(self, p: Point) -> bool:
        """True when *p* is inside an obstacle's hard footprint — a real crash."""
        return any(r.contains(p) for r in self.inflated(self.hard_inflation))

    def occupied_by(self, p: Point, inflation: float | None = None) -> tuple[str, ...]:
        return tuple(r.name for r in self.inflated(inflation) if r.contains(p))

    def _in_bounds(self, p: Point) -> bool:
        b = self.bounds
        return (b['x_min'] <= p[0] <= b['x_max']
                and b['y_min'] <= p[1] <= b['y_max'])

    # ── Planning ──────────────────────────────────────────────────────────────
    def plan(self, start: Point, goal: Point) -> PlanResult:
        """Plan from *start* to *goal*, relaxing clearance only if forced to.

        A berth or the dock may legitimately sit inside another obstacle's
        inflation halo, so obstacles that already contain the start or the goal
        are exempted from collision checks for that call — otherwise the robot
        could never leave a tightly-parked pose.
        """
        exempt = tuple(set(self.occupied_by(start)) | set(self.occupied_by(goal)))

        blocking = self.blockers(start, goal, ignore=exempt)
        if not blocking:
            return PlanResult(waypoints=[goal], direct=True,
                              inflation_used=self.inflation)

        # Full clearance first, then progressively relaxed down to the bare
        # robot radius before we admit defeat.
        attempts = [self.inflation]
        step = self.inflation
        while step > 0.0:
            step = round(step - 0.05, 4)
            if step <= 0.0:
                break
            attempts.append(step)

        for i, inflation in enumerate(attempts):
            route = self._visibility_search(start, goal, inflation, exempt)
            if route is not None:
                return PlanResult(
                    waypoints=route,
                    direct=False,
                    inflation_used=inflation,
                    degraded=(i > 0),
                    blocking=blocking,
                )

        return PlanResult(waypoints=[goal], direct=True, fallback=True,
                          inflation_used=self.inflation, blocking=blocking)

    def _visibility_search(self, start: Point, goal: Point, inflation: float,
                           exempt: tuple[str, ...]) -> list[Point] | None:
        hard = min(self.hard_inflation, inflation)
        rects = [o.inflated(inflation) for o in self.obstacles if o.name not in exempt]
        # Exempted obstacles keep their hard footprint in the graph, so a route
        # may skim a berth's safety halo but never the structure itself.
        rects += [o.inflated(hard) for o in self.obstacles if o.name in exempt]

        nodes: list[Point] = [start, goal]
        for rect in rects:
            for corner in rect.corners(self.corner_clearance):
                if self.bounds and not self._in_bounds(corner):
                    continue
                if any(other.contains(corner) for other in rects):
                    continue
                nodes.append(corner)

        # De-duplicate while preserving order (start=0, goal=1).
        seen: dict[tuple[int, int], int] = {}
        unique: list[Point] = []
        for p in nodes:
            key = (round(p[0] * 1000), round(p[1] * 1000))
            if key not in seen:
                seen[key] = len(unique)
                unique.append(p)
        nodes = unique
        goal_idx = seen[(round(goal[0] * 1000), round(goal[1] * 1000))]

        def visible(a: Point, b: Point) -> bool:
            return not any(segment_intersects_rect(a, b, r) for r in rects)

        n = len(nodes)
        adjacency: list[list[tuple[int, float]]] = [[] for _ in range(n)]
        for i in range(n):
            for j in range(i + 1, n):
                if visible(nodes[i], nodes[j]):
                    w = math.dist(nodes[i], nodes[j])
                    adjacency[i].append((j, w))
                    adjacency[j].append((i, w))

        # A* with a straight-line heuristic.
        best = [math.inf] * n
        best[0] = 0.0
        parent = [-1] * n
        queue: list[tuple[float, float, int]] = [(math.dist(nodes[0], goal), 0.0, 0)]
        closed = [False] * n

        while queue:
            _f, g, idx = heapq.heappop(queue)
            if closed[idx]:
                continue
            closed[idx] = True
            if idx == goal_idx:
                break
            for nxt, w in adjacency[idx]:
                if closed[nxt]:
                    continue
                cand = g + w
                if cand + _EPS < best[nxt]:
                    best[nxt] = cand
                    parent[nxt] = idx
                    heapq.heappush(queue, (cand + math.dist(nodes[nxt], goal), cand, nxt))

        if not closed[goal_idx] or math.isinf(best[goal_idx]):
            return None

        route: list[Point] = []
        idx = goal_idx
        while idx != 0:
            route.append(nodes[idx])
            idx = parent[idx]
            if idx == -1:
                return None
        route.reverse()
        return self._shortcut(start, route, rects)

    @staticmethod
    def _shortcut(start: Point, route: list[Point], rects: list[Rect]) -> list[Point]:
        """Drop intermediate waypoints the robot can see past anyway."""
        pts = [start] + route
        out = [pts[0]]
        i = 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1:
                if not any(segment_intersects_rect(pts[i], pts[j], r) for r in rects):
                    break
                j -= 1
            out.append(pts[j])
            i = j
        return out[1:]


# ═══════════════════════════════════════════════════════════════════════════════
# Registry adapter
# ═══════════════════════════════════════════════════════════════════════════════
def obstacles_from_registry(registry: dict) -> list[Rect]:
    """Build the obstacle set from ``station_registry.yaml``.

    Two sources are merged so the world and the planner cannot drift apart:

    * ``world.obstacles`` — walls, racks and pallets, declared explicitly.
    * ``stations[*]``     — each bay structure, derived from ``marker_pose`` and
      ``bay_footprint``.  This is the part the original tour did not know about,
      and the reason it drove into the Assembly panel.
    """
    rects: list[Rect] = []

    for entry in (registry.get('world', {}) or {}).get('obstacles', []) or []:
        cx, cy = entry['center'][0], entry['center'][1]
        dx, dy = entry['size'][0], entry['size'][1]
        rects.append(Rect(str(entry.get('name', f'obstacle_{len(rects)}')),
                          float(cx), float(cy), float(dx), float(dy)))

    for marker_id, station in sorted((registry.get('stations', {}) or {}).items()):
        pose = station.get('marker_pose')
        footprint = station.get('bay_footprint')
        if not pose or not footprint:
            continue
        rects.append(Rect(f'bay_{marker_id}', float(pose[0]), float(pose[1]),
                          float(footprint[0]), float(footprint[1])))

    return rects
