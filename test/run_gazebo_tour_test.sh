#!/bin/bash
# run_gazebo_tour_test.sh — Phase 3 Gazebo Sim / Tour Integration Test
set -e
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

LOG_FILE="/home/aarock17/ros2_ws/logs/dispatch_events.jsonl"
REG_FILE="/home/aarock17/ros2_ws/src/secure_station_dispatch/config/station_registry.yaml"

mkdir -p "$(dirname "$LOG_FILE")"
rm -f "$LOG_FILE"

echo "=== Starting task_simulator ==="
ros2 run secure_station_dispatch task_simulator &> /tmp/task_sim.log &
TASK_PID=$!
sleep 2

echo "=== Starting dispatcher with enable_security:=true ==="
ros2 run secure_station_dispatch dispatcher \
  --ros-args -p log_path:="$LOG_FILE" \
  -p registry_path:="$REG_FILE" \
  -p enable_security:=true \
  &> /tmp/dispatcher.log &
DISP_PID=$!
sleep 2

echo "=== Starting marker_detector ==="
ros2 run secure_station_dispatch marker_detector \
  --ros-args -p registry_path:="$REG_FILE" \
  &> /tmp/detector.log &
DET_PID=$!
sleep 2

echo "=== Starting mock camera feed (simulating Gazebo camera) ==="
ros2 run secure_station_dispatch mock_image_publisher &> /tmp/mock_cam.log &
CAM_PID=$!
sleep 2

echo "=== Executing Scripted Waypoint Tour ==="
ros2 run secure_station_dispatch waypoint_tour

# Wait for the last station's action task to finish (~3.5 s) before killing nodes
echo "Waiting 6s for last task to complete..."
sleep 6

echo ""
echo "=== dispatch_events.jsonl ==="
if [ -f "$LOG_FILE" ]; then
  cat "$LOG_FILE" | python3 -m json.tool --no-ensure-ascii 2>/dev/null || cat "$LOG_FILE"
else
  echo "Log file $LOG_FILE not found."
fi

kill $TASK_PID $DISP_PID $DET_PID $CAM_PID 2>/dev/null
echo "=== Phase 3 Tour Test Completed ==="
