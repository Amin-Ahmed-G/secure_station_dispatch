# secure_station_dispatch

> **ROS 2 package** — AMR ArUco-marker station dispatch simulation  
> Tested on ROS 2 Humble / Jazzy · Python 3.10+

---

## Assignment Requirement Mapping

| Requirement | Satisfied by |
|---|---|
| ArUco IDs | `marker_detector_node.py` — detects 4×4\_50 markers via `cv2.aruco` |
| Camera integration | Subscribes to `/camera/image_raw`; `mock_image_publisher.py` provides a synthetic feed |
| ROS 2 actions | `task_simulator_node.py` implements the `StationTask` action server; `dispatcher_node.py` is the action client |
| State machine | `DispatcherNode` maintains `IDLE ↔ TASK_IN_PROGRESS` states |
| Custom messages | `StationMarker.msg` (Phase 1) |
| Custom actions | `StationTask.action` (Phase 1) |

> **Phase 2** (HMAC auth + pose checks) is a **portfolio addition beyond the assignment scope**. It is guarded by a `enable_security:=true` launch flag and adds zero risk to Phase 1 behaviour.

---

## Package Structure

```
secure_station_dispatch/
├── msg/
│   └── StationMarker.msg          # Custom marker detection message
├── action/
│   └── StationTask.action         # Custom action interface
├── config/
│   └── station_registry.yaml      # Marker ID → station + task mapping
├── launch/
│   └── station_dispatch.launch.py # Brings up all 3 nodes
├── secure_station_dispatch/       # Python package
│   ├── marker_detector_node.py    # Camera sub → ArUco → StationMarker pub
│   ├── dispatcher_node.py         # State machine + action client
│   ├── task_simulator_node.py     # StationTask action server
│   └── security/
│       ├── __init__.py
│       └── checks.py              # Phase 2 stubs (no-ops in Phase 1)
├── test/
│   ├── mock_image_publisher.py    # Synthetic ArUco image publisher
│   └── test_dispatch_pipeline.py  # End-to-end integration test
├── logs/
│   └── dispatch_events.jsonl      # Written at runtime
├── CMakeLists.txt
├── package.xml
└── setup.py
```

---

## Phase 1 — Build & Run

### Prerequisites

```bash
# ROS 2 Humble or Jazzy (sourced)
source /opt/ros/humble/setup.bash   # or /opt/ros/jazzy/setup.bash

# Python dependencies (OpenCV with contrib for ArUco)
pip install opencv-contrib-python pyyaml

# ROS dependencies (if not already installed)
sudo apt-get install -y \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-vision-opencv
```

### Build

```bash
# Clone / copy the package into your workspace src directory
mkdir -p ~/ros2_ws/src
cp -r secure_station_dispatch ~/ros2_ws/src/

cd ~/ros2_ws
colcon build --packages-select secure_station_dispatch --symlink-install
source install/setup.bash
```

### Option A — Launch all nodes at once

```bash
cd ~/ros2_ws
ros2 launch secure_station_dispatch station_dispatch.launch.py
```

In a second terminal, run the mock camera (cycle through all 4 markers):

```bash
source ~/ros2_ws/install/setup.bash
ros2 run secure_station_dispatch mock_image_publisher
```

### Option B — Run nodes individually (easier for debugging)

```bash
# Terminal 1 — Task Simulator (action server, start first)
ros2 run secure_station_dispatch task_simulator

# Terminal 2 — Dispatcher (state machine + action client)
ros2 run secure_station_dispatch dispatcher

# Terminal 3 — Marker Detector (needs camera feed)
ros2 run secure_station_dispatch marker_detector

# Terminal 4 — Mock camera (publishes synthetic ArUco images)
ros2 run secure_station_dispatch mock_image_publisher
```

### Option C — Skip camera, test pipeline directly

This publishes `StationMarker` messages directly (bypasses ArUco detection):

```bash
# Terminals 1 + 2: task_simulator + dispatcher running (see Option B above)

# Terminal 3 — End-to-end pipeline test
ros2 run secure_station_dispatch test_dispatch_pipeline
```

---

## Verifying the Pipeline

### Expected console output (dispatcher terminal)

```
[INFO] DispatcherNode ready | state=IDLE | log=logs/dispatch_events.jsonl
[INFO] [DETECT] Received marker 0 → Inspection Station (conf=0.95)
[INFO] [DISPATCH] Inspection Station | state → TASK_IN_PROGRESS
[INFO] [ACTION] Goal accepted — task running…
[INFO] [FEEDBACK] Inspection Station | Step 1/5: Positioning sensor array (0%)
[INFO] [FEEDBACK] Inspection Station | Step 2/5: Scanning surface zone A (12%)
...
[INFO] [RESULT] Inspection Station → COMPLETED | Inspection complete in 3.0 s
[INFO] [STATE] → IDLE
```

### Expected console output (task simulator terminal)

```
[INFO] TaskSimulatorNode ready — action server /station_task active.
[INFO] [GOAL] Received goal: task_type="Inspection Station" station="Inspection Station"
[INFO] [TASK START] Inspection Station at Inspection Station (5 steps, ~3.0s)
[INFO]   [1/5] Positioning sensor array (0%)
[INFO]   [2/5] Scanning surface zone A (12%)
...
[INFO] [TASK COMPLETE] Inspection Station complete at Inspection Station in 3.0 s (5 steps)
```

### Checking the event log

```bash
cat logs/dispatch_events.jsonl | python3 -m json.tool --no-ensure-ascii
```

Expected record format:

```json
{"timestamp": "2026-08-18T07:00:01.123+00:00", "ros_time": 1234567890000,
 "marker_id": 0, "station": "Inspection Station", "outcome": "DISPATCHED", "summary": ""}
{"timestamp": "2026-08-18T07:00:04.456+00:00", "ros_time": 1234570890000,
 "marker_id": 0, "station": "Inspection Station", "outcome": "COMPLETED",
 "summary": "Inspection Station complete at Inspection Station in 3.0 s (5 steps)"}
```

### Running the automated test

```bash
# With nodes running in background:
python3 test/test_dispatch_pipeline.py

# Expected output (after ~40 seconds for all 4 tasks):
# ============================================================
#   PIPELINE TEST SUMMARY
# ============================================================
#   Events logged    : 8
#   DISPATCHED events: 4
#   COMPLETED events : 4
#
#   Marker 0: dispatched=True  completed=True   ✓ PASS
#   Marker 1: dispatched=True  completed=True   ✓ PASS
#   Marker 2: dispatched=True  completed=True   ✓ PASS
#   Marker 3: dispatched=True  completed=True   ✓ PASS
#
#   OVERALL: ✓ PASS — Phase 1 pipeline working
# ============================================================
```

---

## Inspecting the topic graph

```bash
# List all active topics
ros2 topic list

# Watch marker detections live
ros2 topic echo /station_marker

# Watch action server status
ros2 action list
ros2 action info /station_task

# Node graph
rqt_graph
```

---

## Stations & Markers

| Marker ID | Station Name | Task Type | Sim Duration |
|---|---|---|---|
| 0 | Inspection Station | Inspection | ~3.0 s |
| 1 | Assembly Station | Assembly | ~3.5 s |
| 2 | Storage Station | Storage | ~2.7 s |
| 3 | Charging Station | Charging | ~3.5 s |

---

## Phase 2 (Portfolio Extension — NOT required by assignment)

Phase 2 adds security features on top of Phase 1 **without modifying any Phase 1 code**.

### What's added

| Feature | Implementation |
|---|---|
| HMAC authentication | `marker_detector_node` signs each message; dispatcher verifies |
| Pose consistency check | Dispatcher rejects detections outside expected station radius |
| Spoof attack demo | `attacks/spoof_marker_publish.py` — publishes unsigned message |
| Location attack demo | `attacks/location_mismatch_attack.py` — valid signature, wrong pose |

### Enable Phase 2 checks

```bash
ros2 launch secure_station_dispatch station_dispatch.launch.py enable_security:=true
```

### Architecture

The `security/checks.py` module provides `verify_hmac()` and `check_pose()`.  
In Phase 1 both return `True` unconditionally.  
In Phase 2, replace the stub implementations in that file — **dispatcher_node.py is unchanged**.

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `ModuleNotFoundError: cv2.aruco` | `pip install opencv-contrib-python` (not `opencv-python`) |
| Action server not found | Start `task_simulator` before `dispatcher` |
| No markers detected | Check `/camera/image_raw` is publishing: `ros2 topic hz /camera/image_raw` |
| Log file not created | Ensure `logs/` directory exists in your CWD or set `log_path` parameter |
| Build fails on rosidl | Make sure `rosidl_default_generators` is installed for your distro |

---

## License

MIT — see LICENSE file.
