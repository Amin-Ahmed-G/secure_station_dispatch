#!/usr/bin/env python3
"""
perception_budget.py
────────────────────
The marker-detection budget, expressed as executable maths.

``marker_detector_node`` scores a detection's confidence from the marker's
apparent perimeter in pixels::

    confidence = min(perimeter / 1200, 1.0)

That makes the registry's ``min_confidence`` gate a *range* gate as much as a
quality gate, and the station layout has to be designed against it:

  * too close and the carrier board overflows the 640x480 frame — OpenCV's
    ArUco contour stage discards candidate quads that run off the image border,
    so detection drops out entirely even though the marker is huge and sharp;
  * too far and the perimeter falls under the gate;
  * and a marker that stays above the gate from halfway down an aisle will make
    the robot dispatch a bay it is only driving past.

The functions below are what the world layout, the board sizes and the 1.75 m
berth standoff were all solved against, and ``test/test_waypoint_planner.py``
re-checks them against the shipped registry on every run.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

Point = tuple[float, float]

CONFIDENCE_PERIMETER_NORM = 1200.0
"""Divisor marker_detector_node._estimate_confidence() normalises against."""


@dataclass(frozen=True)
class CameraModel:
    """Pinhole model of the AMR's mast camera."""

    width_px: int = 640
    height_px: int = 480
    hfov_rad: float = 1.047
    forward_offset_m: float = 0.15
    """Optical centre ahead of the robot datum, metres."""
    height_m: float = 0.85
    """Optical centre above the floor, metres."""

    @property
    def half_hfov(self) -> float:
        return self.hfov_rad / 2.0

    def px_per_m_at(self, distance_m: float) -> float:
        """Image scale at *distance_m* metres, px per metre."""
        if distance_m <= 0.0:
            return math.inf
        return (self.width_px / 2.0) / (distance_m * math.tan(self.half_hfov))

    def frame_half_width_m(self, distance_m: float) -> float:
        return distance_m * math.tan(self.half_hfov)

    def frame_half_height_m(self, distance_m: float) -> float:
        return self.frame_half_width_m(distance_m) * self.height_px / self.width_px


MAST_CAMERA = CameraModel()
"""Matches the ``front_camera`` sensor in worlds/industrial_station_world.sdf."""


def apparent_marker_px(marker_size_m: float, distance_m: float,
                       camera: CameraModel = MAST_CAMERA) -> float:
    """Projected edge length, in pixels, of a marker viewed face-on."""
    return marker_size_m * camera.px_per_m_at(distance_m)


def marker_confidence(marker_size_m: float, distance_m: float,
                      camera: CameraModel = MAST_CAMERA) -> float:
    """Confidence marker_detector_node would report for a face-on marker."""
    perimeter = 4.0 * apparent_marker_px(marker_size_m, distance_m, camera)
    return min(perimeter / CONFIDENCE_PERIMETER_NORM, 1.0)


def board_fits_in_frame(board_size_m: float, distance_m: float,
                        camera: CameraModel = MAST_CAMERA,
                        margin: float = 0.05) -> bool:
    """True when the whole carrier board sits inside the image with *margin*.

    The height axis binds first on a 4:3 sensor, so only it is checked.
    """
    return (board_size_m / 2.0) * (1.0 + margin) <= camera.frame_half_height_m(distance_m)


def detection_window(marker_size_m: float, board_size_m: float, gate: float,
                     camera: CameraModel = MAST_CAMERA) -> tuple[float, float]:
    """(near, far) camera ranges, in metres, over which a board is detectable.

    The near limit is where the board starts clipping the frame; the far limit
    is where confidence drops through *gate*.
    """
    near = (board_size_m / 2.0) * 1.05 / math.tan(camera.half_hfov) \
        * camera.width_px / camera.height_px
    far = (4.0 * marker_size_m * camera.width_px / 2.0) \
        / (gate * CONFIDENCE_PERIMETER_NORM * math.tan(camera.half_hfov))
    return near, far


def marker_in_view(robot_xy: Point, robot_yaw: float,
                   marker_xy: Point, marker_normal: Point,
                   camera: CameraModel = MAST_CAMERA) -> tuple[bool, float, float]:
    """Geometric visibility test for one marker board.

    Returns ``(visible, range_m, bearing_rad)`` where *range_m* is measured from
    the optical centre and *bearing_rad* is the marker's angle off the optical
    axis.  A board is visible only when it is inside the horizontal FOV *and*
    the camera is on its front face.
    """
    cam = (robot_xy[0] + camera.forward_offset_m * math.cos(robot_yaw),
           robot_xy[1] + camera.forward_offset_m * math.sin(robot_yaw))
    dx, dy = marker_xy[0] - cam[0], marker_xy[1] - cam[1]
    distance = math.hypot(dx, dy)
    if distance < 1e-6:
        return False, 0.0, 0.0

    bearing = math.atan2(dy, dx) - robot_yaw
    bearing = math.atan2(math.sin(bearing), math.cos(bearing))
    in_fov = abs(bearing) <= camera.half_hfov

    # Front face only: the camera must sit on the +normal side of the board.
    facing = (marker_normal[0] * (cam[0] - marker_xy[0])
              + marker_normal[1] * (cam[1] - marker_xy[1])) > 0.0

    return (in_fov and facing), distance, bearing


def would_dispatch(robot_xy: Point, robot_yaw: float, station: dict, gate: float,
                   camera: CameraModel = MAST_CAMERA) -> bool:
    """True when a robot at this pose would publish an accepted detection.

    Used by the regression tests to prove that no transit pose along the tour
    can trigger a station the robot is merely driving past.
    """
    marker_xy = (station['marker_pose'][0], station['marker_pose'][1])
    normal = (station['marker_normal'][0], station['marker_normal'][1])
    visible, distance, _bearing = marker_in_view(robot_xy, robot_yaw, marker_xy,
                                                 normal, camera)
    if not visible:
        return False
    return marker_confidence(station.get('_marker_size_m', 0.70), distance, camera) >= gate
