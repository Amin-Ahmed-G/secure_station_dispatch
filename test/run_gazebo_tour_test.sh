#!/usr/bin/env bash
# ═════════════════════════════════════════════════════════════════════════════
# run_gazebo_tour_test.sh — Phase 3 headless tour integration test
# ═════════════════════════════════════════════════════════════════════════════
# Runs the full dispatch stack WITHOUT Gazebo: the mock camera stands in for the
# mast camera, and waypoint_tour_node falls back to integrating its own commanded
# velocities when no /odom appears.  The planner, the control law, the stall
# watchdog and the dispatcher state machine are all the real ones, so this is a
# fast smoke test of everything except the physics.
#
# For the full simulated demo use:   ros2 launch secure_station_dispatch gazebo_demo.launch.py
# ═════════════════════════════════════════════════════════════════════════════
set -uo pipefail

PKG=secure_station_dispatch
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="${ROS_WS:-$(cd "$PKG_DIR/../.." && pwd)}"

ROS_DISTRO_SETUP="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
[ -f "$ROS_DISTRO_SETUP" ] && source "$ROS_DISTRO_SETUP"
[ -f "$WS_DIR/install/setup.bash" ] && source "$WS_DIR/install/setup.bash"

LOG_DIR="$WS_DIR/logs"
LOG_FILE="$LOG_DIR/dispatch_events.jsonl"
REPORT_FILE="$LOG_DIR/tour_report.json"
REG_FILE="$PKG_DIR/config/station_registry.yaml"

mkdir -p "$LOG_DIR"
rm -f "$LOG_FILE" "$REPORT_FILE"

PIDS=()
cleanup() {
  for pid in "${PIDS[@]:-}"; do kill "$pid" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT

start() {   # start <label> <logfile> <args...>
  local label="$1" logfile="$2"; shift 2
  echo "=== starting $label ==="
  "$@" &> "$logfile" &
  PIDS+=("$!")
}

start "task_simulator" /tmp/task_sim.log \
  ros2 run "$PKG" task_simulator
sleep 2

start "dispatcher (security enabled)" /tmp/dispatcher.log \
  ros2 run "$PKG" dispatcher --ros-args \
    -p log_path:="$LOG_FILE" \
    -p registry_path:="$REG_FILE" \
    -p enable_security:=true
sleep 2

start "marker_detector" /tmp/detector.log \
  ros2 run "$PKG" marker_detector --ros-args \
    -p registry_path:="$REG_FILE"
sleep 2

start "mock camera" /tmp/mock_cam.log \
  ros2 run "$PKG" mock_image_publisher --ros-args \
    -p registry_path:="$REG_FILE" -p rate_hz:=4.0
sleep 2

echo "=== executing the scripted waypoint tour ==="
ros2 run "$PKG" waypoint_tour --ros-args \
  -p registry_path:="$REG_FILE" \
  -p report_path:="$REPORT_FILE"
TOUR_RC=$?

echo "waiting 6 s for the final task to settle…"
sleep 6

echo
echo "=== dispatch_events.jsonl ==="
if [ -f "$LOG_FILE" ]; then
  python3 - "$LOG_FILE" <<'PY'
import json, sys
from collections import Counter
path = sys.argv[1]
rows = [json.loads(line) for line in open(path) if line.strip()]
for row in rows:
    print(f"  {row['outcome']:<20} marker {row['marker_id']}  {row['station']}")
counts = Counter(r['outcome'] for r in rows)
print(f"\n  {len(rows)} events: " + ", ".join(f"{k}={v}" for k, v in sorted(counts.items())))
dispatched = counts.get('DISPATCHED', 0)
completed = counts.get('COMPLETED', 0)
print(f"  DISPATCHED={dispatched}  COMPLETED={completed}  (expected 4 / 4)")
sys.exit(0 if dispatched == 4 == completed else 1)
PY
  LOG_RC=$?
else
  echo "  log file $LOG_FILE not found."
  LOG_RC=1
fi

echo
echo "=== tour_report.json ==="
[ -f "$REPORT_FILE" ] && python3 -m json.tool "$REPORT_FILE" | head -40 || echo "  no report written"

echo
if [ "$TOUR_RC" -eq 0 ] && [ "$LOG_RC" -eq 0 ]; then
  echo "=== PHASE 3 TOUR TEST: PASS ==="
  exit 0
fi
echo "=== PHASE 3 TOUR TEST: FAIL (tour_rc=$TOUR_RC log_rc=$LOG_RC) ==="
echo "    node logs: /tmp/task_sim.log /tmp/dispatcher.log /tmp/detector.log /tmp/mock_cam.log"
exit 1
