# Operations runbook

Everything you need to build, run, verify and debug `secure_station_dispatch`.

---

## Build

```bash
# Dependencies
sudo apt-get install -y \
    ros-$ROS_DISTRO-ros-gz-bridge ros-$ROS_DISTRO-ros-gz-sim \
    ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-vision-opencv
pip install opencv-contrib-python pyyaml     # contrib: cv2.aruco lives there

cd ~/ros2_ws
colcon build --packages-select secure_station_dispatch --symlink-install
source install/setup.bash
```

> `--symlink-install` matters: the generated world references marker textures by absolute
> path into the source tree. If you build without it, or relocate the package, re-run
> `python3 scripts/generate_world.py --texture-root <new path>`.

---

## Run

### Full simulated demo

```bash
ros2 launch secure_station_dispatch gazebo_demo.launch.py
```

| Argument | Default | Effect |
|---|---|---|
| `gui` | `true` | `false` runs Gazebo server-only |
| `use_sim_time` | `false` | `true` drives node clocks from the bridged `/clock` |
| `enable_security` | `true` | Phase 2 HMAC + pose checks |
| `run_tour` | `true` | `false` brings the stack up without driving |
| `shutdown_on_finish` | `true` | tear the stack down when the tour exits |
| `registry_path` | installed | override the station registry |
| `log_path` | `logs/dispatch_events.jsonl` | dispatcher event log |
| `report_path` | `logs/tour_report.json` | per-station tour report |

```bash
# Headless, for CI or a machine without a GPU
ros2 launch secure_station_dispatch gazebo_demo.launch.py gui:=false

# Bring the world up and drive it yourself
ros2 launch secure_station_dispatch gazebo_demo.launch.py run_tour:=false
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
```

### Headless smoke test (no Gazebo)

```bash
bash test/run_gazebo_tour_test.sh
```

The mock camera stands in for the mast camera and the tour integrates its own commanded
velocities. Planner, control law, watchdog and dispatcher are all the real ones. Exits
non-zero unless 4 stations dispatch and 4 complete.

### ROS-free regression suite (about one second)

```bash
bash test/run_navigation_checks.sh
```

Runs world/registry synchronisation, the planner and perception regressions, and the full
offline tour simulation. Run this before every commit.

### Phase 1 / Phase 2 suites

```bash
bash test/run_test.sh            # baseline dispatch pipeline
bash test/run_security_test.sh   # HMAC + pose-consistency attacks
```

---

## Verify a run

```bash
# Per-station tour report
python3 -m json.tool logs/tour_report.json

# Dispatch event log
cat logs/dispatch_events.jsonl | python3 -m json.tool --no-ensure-ascii
```

A healthy tour prints:

```
════════════════════════════════════════════════════════════════════════
  WAYPOINT TOUR REPORT
════════════════════════════════════════════════════════════════════════
  ID Station               Berthed   Err (cm)  Yaw (°)  Recov  Dispatch   Path
  ────────────────────────────────────────────────────────────────────
  0  Inspection Station    yes            4.1      0.8      0  COMPLETED  DIRECT
  1  Assembly Station      yes            6.3      1.2      0  COMPLETED  DIRECT
  2  Storage Station       yes            5.8      0.9      0  COMPLETED  DETOUR around rack_centre
  3  Charging Station      yes            7.2      1.6      0  COMPLETED  DIRECT
  ────────────────────────────────────────────────────────────────────
  berthed 4/4   dispatch COMPLETED 4/4   recoveries 0   OVERALL: PASS
```

---

## Regenerating derived artefacts

`config/station_registry.yaml` is the source of truth. After editing it:

```bash
python3 scripts/generate_world.py           # rebuild the SDF
python3 scripts/generate_marker_boards.py   # rebuild signage + verify detection
python3 scripts/render_facility_map.py      # rebuild the docs figures
bash    test/run_navigation_checks.sh       # confirm nothing drifted
```

`scripts/check_world_sync.py` alone will tell you whether the world and the registry still
agree, without regenerating anything.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `ModuleNotFoundError: cv2.aruco` | `opencv-python` installed instead of contrib | `pip install opencv-contrib-python` |
| Markers render as flat white boards in Gazebo | Texture path no longer resolves | `python3 scripts/generate_world.py --texture-root $(pwd)/models/materials/textures` |
| `STALL: commanded motion but moved only …` | Robot really is against something | Check the pose in the message against `docs/facility_map.png`; if the obstacle is not on the map, add it to `world.obstacles` |
| `Pre-flight route check FAILED` | A berth sits inside an obstacle footprint | The log names the obstacle — fix `expected_pose` or the footprint in the registry |
| `no collision-free route found` | Obstacles seal the goal off | Re-check `world.obstacles`; `robot_radius_m + safety_margin_m` may exceed the gap |
| Detections stop when the robot is close | Board clipping the frame | Raise `world.marker_board.standoff_m`, or shrink `board_size_m` |
| Detections never accepted | Confidence under the gate | Lower `min_confidence`, enlarge `marker_size_m`, or shorten the standoff — then re-run `scripts/generate_marker_boards.py` to see the sweep |
| `LOCATION_MISMATCH` on a station the robot is parked at | `pose_tolerance_m` too tight, or `/mock_pose` not publishing | Check `ros2 topic hz /mock_pose`; tolerance must exceed `goal_tolerance_m` |
| Same station dispatched twice | Cooldown disabled | `dispatcher.redispatch_cooldown_s` in the registry, or `-p redispatch_cooldown_s:=20.0` |
| Action server not found | `task_simulator` started after the dispatcher | The launch file already staggers them; when running by hand start the simulator first |
| World/registry drift on CI | Registry edited without regenerating | `python3 scripts/generate_world.py` and commit the SDF |

### Useful introspection

```bash
ros2 topic hz /camera/image_raw       # is the camera bridged?
ros2 topic echo /station_marker       # what is the detector seeing?
ros2 topic echo /dispatch_status      # every dispatcher outcome as JSON
ros2 topic echo /tour_status          # leg / recovery / completion events
ros2 action info /station_task        # is the action server up?
ros2 param get /marker_detector_node min_confidence
```

---

## Tuning reference

All of these live under `navigation:` in `config/station_registry.yaml`.

| Parameter | Default | Raise it when | Lower it when |
|---|---|---|---|
| `robot_radius_m` | 0.28 | The robot clips corners | Aisles are too tight to plan |
| `safety_margin_m` | 0.15 | You want wider berths from racking | The planner cannot find a route |
| `goal_tolerance_m` | 0.10 | Berthing takes too long to settle | Detections fail at the tolerance edge |
| `max_linear_speed` | 0.45 | The tour is too slow | The robot overshoots berths |
| `turn_in_place_rad` | 0.55 | The robot spins too eagerly | The robot arcs into corners |
| `stall_window_s` | 3.0 | False stalls on slow starts | Collisions are detected too late |
| `stall_min_progress_m` | 0.04 | Odometry is noisy | Slow contact is missed |
| `task_wait_timeout_s` | 25.0 | Tasks take longer | The tour hangs on a dead dispatcher |
