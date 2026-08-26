#!/usr/bin/env python3
"""
controller.py
─────────────
The differential-drive control law and the stall watchdog, as plain functions.

Both were previously inlined in ``waypoint_tour_node`` and therefore untestable
without a simulator.  Pulling them out means the full tour — planner, control
law, watchdog and a unicycle model of the robot — can be replayed offline in
``test/test_waypoint_planner.py``, and a regression that would drive the robot
into a station bay fails the test suite instead of the demo.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

Point = tuple[float, float]


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


@dataclass(frozen=True)
class ControlParams:
    """Controller tuning, normally loaded from ``navigation`` in the registry."""

    goal_tolerance_m: float = 0.10
    yaw_tolerance_rad: float = 0.06
    max_linear_speed: float = 0.45
    min_linear_speed: float = 0.05
    max_angular_speed: float = 0.90
    min_angular_speed: float = 0.12
    linear_gain: float = 0.80
    angular_gain: float = 1.40
    turn_in_place_rad: float = 0.55

    @classmethod
    def from_registry(cls, navigation: dict | None) -> "ControlParams":
        navigation = navigation or {}
        fields = cls.__dataclass_fields__  # noqa: SLF001 - documented dataclass API
        return cls(**{k: float(navigation[k]) for k in fields if k in navigation})


@dataclass(frozen=True)
class Command:
    linear: float
    angular: float
    arrived: bool
    distance: float
    heading_error: float

    @property
    def is_moving(self) -> bool:
        return abs(self.linear) > 1e-3 or abs(self.angular) > 1e-3


class DiffDriveController:
    """Turn-then-arc pursuit of a single waypoint.

    Heading errors larger than ``turn_in_place_rad`` are resolved on the spot
    rather than by arcing, because arcing wide is how a robot ends up cutting
    the corner the planner just routed it around.  Inside that cone the linear
    speed is proportional to range and attenuated by the heading error, so the
    robot decelerates into the goal instead of overshooting and hunting.
    """

    def __init__(self, params: ControlParams | None = None) -> None:
        self.p = params or ControlParams()

    def command(self, x: float, y: float, yaw: float, target: Point) -> Command:
        dx, dy = target[0] - x, target[1] - y
        distance = math.hypot(dx, dy)
        heading_error = normalize_angle(math.atan2(dy, dx) - yaw)

        if distance <= self.p.goal_tolerance_m:
            return Command(0.0, 0.0, True, distance, heading_error)

        if abs(heading_error) > self.p.turn_in_place_rad:
            angular = math.copysign(
                clamp(self.p.angular_gain * abs(heading_error),
                      self.p.min_angular_speed, self.p.max_angular_speed),
                heading_error)
            return Command(0.0, angular, False, distance, heading_error)

        linear = clamp(self.p.linear_gain * distance,
                       self.p.min_linear_speed, self.p.max_linear_speed)
        linear *= max(0.35, math.cos(heading_error))
        angular = clamp(self.p.angular_gain * heading_error,
                        -self.p.max_angular_speed, self.p.max_angular_speed)
        return Command(linear, angular, False, distance, heading_error)

    def rotate_command(self, yaw: float, target_yaw: float) -> Command:
        error = normalize_angle(target_yaw - yaw)
        if abs(error) <= self.p.yaw_tolerance_rad:
            return Command(0.0, 0.0, True, 0.0, error)
        angular = math.copysign(
            clamp(self.p.angular_gain * abs(error),
                  self.p.min_angular_speed, self.p.max_angular_speed), error)
        return Command(0.0, angular, False, 0.0, error)


class StallWatchdog:
    """Aborts a leg when commanded motion stops producing displacement.

    This is the piece whose absence let the robot grind against the Assembly
    marker panel for the full 20 s leg timeout, then continue the tour from a
    wedged pose as though it had arrived.
    """

    def __init__(self, window_s: float = 3.0, min_progress_m: float = 0.04) -> None:
        self.window_s = float(window_s)
        self.min_progress_m = float(min_progress_m)
        self._anchor: Point | None = None
        self._anchor_t: float = 0.0
        self.travelled: float = 0.0

    def reset(self, now: float, position: Point) -> None:
        self._anchor = position
        self._anchor_t = now
        self.travelled = 0.0

    def update(self, now: float, position: Point, commanding_motion: bool) -> bool:
        """Feed one control tick.  Returns True the moment a stall is confirmed."""
        if self._anchor is None:
            self.reset(now, position)
            return False

        if not commanding_motion:
            # Idle time must not accumulate towards a stall.
            self.reset(now, position)
            return False

        self.travelled = math.dist(position, self._anchor)
        if self.travelled >= self.min_progress_m:
            self.reset(now, position)
            return False

        return (now - self._anchor_t) >= self.window_s
