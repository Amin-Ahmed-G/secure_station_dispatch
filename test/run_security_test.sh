#!/bin/bash
# run_security_test.sh — launch nodes with enable_security:=true and execute security test suite
set -e
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
mkdir -p ~/ros2_ws/logs

echo "=== Starting task_simulator ==="
ros2 run secure_station_dispatch task_simulator &> /tmp/task_sim.log &
TASK_PID=$!
sleep 2

echo "=== Starting dispatcher with enable_security:=true ==="
ros2 run secure_station_dispatch dispatcher \
  --ros-args -p log_path:=/home/aarock17/ros2_ws/logs/dispatch_events.jsonl \
  -p enable_security:=true \
  &> /tmp/dispatcher.log &
DISP_PID=$!
sleep 2

echo "task_sim PID=$TASK_PID  dispatcher PID=$DISP_PID"
echo ""

echo "=== Running Phase 2 Security Test Suite ==="
ros2 run secure_station_dispatch test_security_checks \
  --ros-args -p log_path:=/home/aarock17/ros2_ws/logs/dispatch_events.jsonl

echo ""
echo "=== dispatch_events.jsonl ==="
cat ~/ros2_ws/logs/dispatch_events.jsonl | python3 -m json.tool --no-ensure-ascii 2>/dev/null || cat ~/ros2_ws/logs/dispatch_events.jsonl

kill $TASK_PID $DISP_PID 2>/dev/null
echo "=== Security Test Finished ==="
