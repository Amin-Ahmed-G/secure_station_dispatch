# Architecture

`secure_station_dispatch` is a ROS 2 package in which an autonomous mobile robot tours a
walled facility, reads an ArUco fiducial at each station bay, authenticates the detection,
and executes a station-specific task through a ROS 2 action.

It is built in three layers, each additive over the last:

| Phase | Concern | Key modules |
|---|---|---|
| 1 | Perception → dispatch → action pipeline | `marker_detector_node`, `dispatcher_node`, `task_simulator_node` |
| 2 | Authenticating what the camera claims to see | `security/checks.py`, `attacks/` |
| 3 | Making the robot physically get there | `navigation/`, `waypoint_tour_node`, the Gazebo world |

---

## 1. Runtime topology

```
                    ┌──────────────────────────────┐
   Gazebo Sim ─────►│ /camera/image_raw   (Image)  │
   (mast camera)    └──────────────┬───────────────┘
                                   ▼
                        marker_detector_node
                     · cv2.aruco DICT_4X4_50
                     · confidence = perimeter/1200
                     · HMAC-SHA256 signs each detection
                                   │
                    /station_marker │ (StationMarker)
                                   ▼
                          dispatcher_node
                     · IDLE ↔ TASK_IN_PROGRESS
                     · verify_hmac() + check_pose()
                     · per-marker re-dispatch cooldown
                          │                    │
          /station_task   │                    │  /dispatch_status (String/JSON)
          (StationTask)   ▼                    ▼
                  task_simulator_node    waypoint_tour_node
                                          · PathPlanner
                                          · DiffDriveController
                                          · StallWatchdog
                                                │
                              /cmd_vel (Twist)  │  /mock_pose (PoseStamped)
                                                ▼
                                          Gazebo DiffDrive ──► /odom
```

### Topics and interfaces

| Name | Type | Direction | Purpose |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo → detector | Mast camera feed |
| `/station_marker` | `secure_station_dispatch/StationMarker` | detector → dispatcher | Detection + pose + HMAC |
| `/station_task` | `secure_station_dispatch/StationTask` | dispatcher → simulator | Action goal / feedback / result |
| `/dispatch_status` | `std_msgs/String` (JSON) | dispatcher → tour | Every logged outcome, mirrored |
| `/tour_status` | `std_msgs/String` (JSON) | tour → observers | Leg, recovery and completion events |
| `/cmd_vel` | `geometry_msgs/Twist` | tour → Gazebo | Velocity command |
| `/odom` | `nav_msgs/Odometry` | Gazebo → tour | Pose feedback |
| `/mock_pose` | `geometry_msgs/PoseStamped` | tour → detector | Pose stamped into each detection |

---

## 2. The registry is the source of truth

`config/station_registry.yaml` is the only place facility geometry is written down.
Everything else reads it or is generated from it:

```
                        station_registry.yaml
                                 │
        ┌────────────┬───────────┼────────────┬──────────────┐
        ▼            ▼           ▼            ▼              ▼
  marker_detector  dispatcher  waypoint_tour  generate_world.py  check_world_sync.py
   (dictionary,   (hmac_key,   (berths,       (bays, walls,      (fails the build on
    confidence)    tolerance)   obstacles,     racks, decals)     any drift)
                                approach yaw)
```

This structure exists because of a specific failure: in the first Phase 3 build the world
placed solid marker panels 0.40 m in front of every berth, and nothing that planned the
robot's motion knew they existed. See
[root-cause-phase3-collision.md](root-cause-phase3-collision.md).

Regenerate derived artefacts after editing the registry:

```bash
python3 scripts/generate_world.py          # worlds/industrial_station_world.sdf
python3 scripts/generate_marker_boards.py  # models/materials/textures/*.png
python3 scripts/render_facility_map.py     # docs/*.png
bash    test/run_navigation_checks.sh      # verify nothing drifted
```

---

## 3. Navigation

`secure_station_dispatch/navigation/` is deliberately ROS-free, so the whole tour can be
replayed offline in the test suite.

| Module | Responsibility |
|---|---|
| `planner.py` | `Rect` obstacles, segment/box intersection, visibility-graph A\* with two clearance layers |
| `controller.py` | Turn-then-arc differential-drive control law, stall watchdog |
| `perception_budget.py` | Pinhole camera model and the marker legibility maths |

### Clearance model

Two inflations, not one:

* **hard** = `robot_radius_m` (0.28 m) — the circumscribed radius of the chassis and
  wheels. Crossing it is a collision.
* **soft** = hard + `safety_margin_m` (0.15 m) = 0.43 m — the clearance the planner
  prefers to keep.

A berth may legitimately sit inside a soft halo (the robot has to approach a board to
read it), so obstacles containing the start or goal are forgiven their margin — but never
their hard footprint.

### Leg execution

```
execute_leg(goal):
    for attempt in 1..max_recoveries+1:
        plan = planner.plan(current_pose, goal)      # detour if the line is blocked
        for waypoint in plan.waypoints:
            status = drive_to_point(waypoint)        # arrived | stalled | timeout
            if status != arrived: break
        if arrived: return success
        recover()                                    # reverse 0.35 m, turn 0.70 rad
    return failure                                   # reported, never swallowed
```

### Odometry sources

With Gazebo running, poses come from `/odom`. Run headless, no odometry ever arrives, so
after `navigation.odom_wait_s` the node integrates its own commanded velocities. Both
modes drive the *same* controller, planner and watchdog, so the headless smoke test
exercises the real logic.

---

## 4. Marker legibility budget

The detector scores confidence from apparent size:

```
confidence = min(marker_perimeter_px / 1200, 1.0)
```

so `min_confidence` is a **range gate** as much as a quality gate, and the facility layout
has to be solved against it.

| Constraint | Consequence |
|---|---|
| Board must fit wholly inside the frame | OpenCV drops candidate quads touching the border → near limit ≈ 1.33 m |
| Confidence ≥ 0.75 | far limit ≈ 1.72 m from the camera |
| Camera 0.15 m ahead of the datum | berth standoff 1.65 m → 1.50 m of range |
| Camera height = board height = 0.85 m | board framed centrally, no vertical clipping |

Result: a board is legible from **its own berth and nowhere else**. That property is what
prevents a robot driving down the aisle from dispatching a station it is merely passing,
and it is asserted directly by
`test_no_station_can_be_dispatched_while_merely_driving_past`.

| Camera range | Marker px | Confidence | Verdict |
|---|---|---|---|
| 1.20 m | 323 | 1.00 | board clips the frame |
| 1.40 m | 277 | 0.92 | accept |
| **1.50 m** | **259** | **0.86** | **accept — berth** |
| 1.72 m | 226 | 0.75 | accept (gate) |
| 1.90 m | 204 | 0.68 | reject |
| 2.50 m | 155 | 0.52 | reject |

---

## 5. Signage

Each bay carries a three-part sign stack, generated by
`scripts/generate_marker_boards.py`:

| Element | Size | Purpose |
|---|---|---|
| Nameplate | 1.10 × 0.18 m | Station name + bay number, accent coloured |
| Marker board | 1.10 × 1.10 m | Matte white carrier, 0.70 m DICT_4X4_50 marker, 1.7-cell quiet zone |
| Info plate | 1.10 × 0.14 m | Dictionary, ID, marker size, task class |

The text lives on separate plates rather than on the board because the plates may safely
clip the frame while the board may not. The generator verifies every board round-trips
through the detector and prints the simulated range sweep.

---

## 6. Security model

| Attack | Mitigation | Demo |
|---|---|---|
| Forged `StationMarker` on the topic | HMAC-SHA256 over `marker_id + station_name`, keyed per station | `attacks/spoof_marker_publish.py` → `AUTH_FAIL` |
| Valid signature replayed from the wrong place | `check_pose()` compares `robot_pose` against `expected_pose` within `pose_tolerance_m` | `attacks/location_mismatch_attack.py` → `LOCATION_MISMATCH` |
| A real board read from too far away | Confidence gate doubles as a range gate | Geometry, asserted in tests |
| The same bay dispatched repeatedly | Per-marker cooldown after completion | `dispatcher.redispatch_cooldown_s` |

`pose_tolerance_m` (0.5 m) is deliberately wider than the navigation `goal_tolerance_m`
(0.10 m), so a correctly berthed robot can never be rejected as a location mismatch —
asserted by `test_security_tolerance_is_wider_than_the_navigation_tolerance`.

---

## 7. Build notes

The package builds with `ament_cmake` because it generates `rosidl` interfaces.
`ament_python_install_package()` would declare a CMake target colliding with rosidl's, so
the Python tree is installed with plain `install(DIRECTORY ...)` into the interpreter's
`site-packages`; the interpreter version is queried at configure time rather than
hard-coded, so the same `CMakeLists.txt` builds on Humble (3.10) and Jazzy (3.12).

`setup.py` is retained only for `pip install -e .`, which makes the ROS-free suites
runnable outside a workspace.
