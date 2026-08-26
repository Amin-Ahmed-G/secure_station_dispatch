"""
secure_station_dispatch.navigation
──────────────────────────────────
Deliberately ROS-free navigation support for the Phase 3 Gazebo tour.

Keeping the geometry, planning and perception-budget maths out of the node lets
them be unit-tested without a ROS graph or a running simulator
(``test/test_waypoint_planner.py``), which is how the Assembly-bay collision
regression is now pinned.
"""

from .planner import (            # noqa: F401
    Rect,
    PathPlanner,
    PlanResult,
    segment_intersects_rect,
    obstacles_from_registry,
)
from .controller import (         # noqa: F401
    Command,
    ControlParams,
    DiffDriveController,
    StallWatchdog,
    clamp,
    normalize_angle,
)
from .perception_budget import (  # noqa: F401
    CameraModel,
    MAST_CAMERA,
    apparent_marker_px,
    marker_confidence,
    marker_in_view,
)

__all__ = [
    'Rect',
    'PathPlanner',
    'PlanResult',
    'segment_intersects_rect',
    'obstacles_from_registry',
    'Command',
    'ControlParams',
    'DiffDriveController',
    'StallWatchdog',
    'clamp',
    'normalize_angle',
    'CameraModel',
    'MAST_CAMERA',
    'apparent_marker_px',
    'marker_confidence',
    'marker_in_view',
]
