# Changelog

All notable changes to `secure_station_dispatch`.

---

## [3.0.0] — Phase 3 complete

The scripted Gazebo tour stopped at the second bay: the robot drove into the Assembly
Station's marker panel while departing for Storage, wedged, and completed the itinerary
from a corrupted pose — while still printing `WAYPOINT TOUR COMPLETE`. Fixing that
touched the layout, the planner, the controller, the perception geometry and the
reporting. Full analysis in
[root-cause-phase3-collision.md](root-cause-phase3-collision.md).

### Added

- `secure_station_dispatch/navigation/` — a ROS-free navigation package:
  - `planner.py`: `Rect` obstacles, segment/box intersection, visibility-graph A\* with
    shortcutting, and **two clearance layers** (hard footprint vs. safety margin) so a
    berth inside an obstacle's halo cannot license planning through the obstacle itself.
  - `controller.py`: the turn-then-arc control law, `ControlParams`, and `StallWatchdog`.
  - `perception_budget.py`: pinhole camera model, marker confidence, detection window and
    field-of-view visibility, as executable maths.
- `scripts/generate_world.py` — projects `station_registry.yaml` into the Gazebo world, so
  the simulator and the planner cannot describe different facilities.
- `scripts/check_world_sync.py` — fails on any drift between world collision geometry and
  declared registry footprints.
- `scripts/generate_marker_boards.py` — generates the three-part bay signage and verifies
  every board round-trips through the detector, printing a simulated range sweep.
- `scripts/render_facility_map.py` — renders `docs/facility_map.png` and
  `docs/legacy_failure.png` from the registry.
- `test/test_waypoint_planner.py` — 17 planner, layout and perception regressions,
  including the original-geometry collision replay.
- `test/test_tour_simulation.py` — replays the entire tour offline at 20 Hz with a
  unicycle robot model, asserting zero obstacle contacts and per-berth accuracy.
- `test/run_navigation_checks.sh` — the ROS-free suite, ~1 s.
- `/dispatch_status` and `/tour_status` JSON topics.
- `logs/tour_report.json` — per-station position error, yaw error, recovery count, plan
  description and dispatch outcome.
- `docs/` — architecture, root cause analysis, operations runbook, this changelog.

### Changed

- **Facility re-laid.** Bays moved to the perimeter facing inward at a 1.65 m berth
  standoff, each with its own approach heading. No berth-to-berth straight line passes
  through a bay any more.
- **`waypoint_tour_node` rewritten.** Legs are planned, not assumed; stalls abort and
  recover; arrival is verified against registry tolerances; the node exits non-zero if any
  berth was missed; the tour waits for real task completion via `/dispatch_status` instead
  of a fixed dwell; a pre-flight check refuses to start an unreachable itinerary.
- **`station_registry.yaml` is now the single source of truth** — approach headings, board
  poses and normals, bay footprints, facility bounds, obstacle list, navigation tuning and
  the dispatcher cooldown all live there.
- **Camera moved to an 0.85 m sensor mast**, level with the board centres and 0.15 m ahead
  of the robot datum. Board 1.10 m, marker 0.70 m, `min_confidence` 0.75 — solved so a
  board is legible from its own berth and nowhere else.
- **Robot model rebuilt** — four ball casters (was two, which let the chassis pitch),
  correct spawn height, wheel friction, acceleration limits, 30 Hz odometry.
- **Launch file hardened** — `/clock` bridged, `gui` / `use_sim_time` / `run_tour` /
  `shutdown_on_finish` arguments, registry passed to the tour node, Gazebo resource path
  exported, clean teardown when the tour ends.
- **`CMakeLists.txt`** — Python site-packages path derived from the interpreter instead of
  hard-coded to 3.12; `colcon test` wired to the ROS-free suites; `docs/`, `scripts/` and
  `test/` installed; duplicated install blocks removed.
- **`mock_image_publisher`** publishes an empty frame while the robot is between berths
  instead of cycling markers, matching what the real mast camera sees.

### Fixed

- Robot drove into the Assembly marker panel on the Assembly → Storage leg and wedged.
- `drive_to` treated a leg timeout as an arrival, so a failed leg silently corrupted every
  later one.
- No stall detection: the controller pushed against an obstacle for the full leg timeout.
- The tour reported success regardless of what actually happened.
- **Duplicate dispatch** — the same station re-fired on the next camera frame after its
  task completed; now suppressed by a per-marker cooldown, with throttled logging.
- **`min_confidence` and `aruco_dictionary` in the registry were ignored** — node
  parameter defaults silently overrode them, so tuning the YAML had no effect.
- Detection geometry that could not have worked: a 0.60 m marker at 0.40 m standoff
  overflows a 640×480 frame, and OpenCV discards candidate quads touching the border.

---

## [2.0.0] — Phase 2, security extensions

- HMAC-SHA256 authentication of `StationMarker` messages, keyed per station.
- Pose-consistency checking against `expected_pose` / `pose_tolerance_m`.
- `attacks/spoof_marker_publish.py` and `attacks/location_mismatch_attack.py`.
- `test/run_security_test.sh`.

## [1.0.0] — Phase 1, core pipeline

- `marker_detector_node`, `dispatcher_node`, `task_simulator_node`.
- `StationMarker.msg`, `StationTask.action`, `station_registry.yaml`.
- JSONL dispatch event log, `test/test_dispatch_pipeline.py`.
