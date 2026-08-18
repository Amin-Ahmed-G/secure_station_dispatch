#!/bin/bash
# run_test.sh — start all nodes and run the pipeline test in one sourced shell
set -e
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
mkdir -p ~/ros2_ws/logs

echo "=== Starting task_simulator ==="
ros2 run secure_station_dispatch task_simulator &> /tmp/task_sim.log &
TASK_PID=$!
sleep 2

echo "=== Starting dispatcher ==="
ros2 run secure_station_dispatch dispatcher \
  --ros-args -p log_path:=/home/aarock17/ros2_ws/logs/dispatch_events.jsonl \
  &> /tmp/dispatcher.log &
DISP_PID=$!
sleep 2

echo "task_sim PID=$TASK_PID  dispatcher PID=$DISP_PID"
echo ""
echo "=== task_simulator startup ==="
cat /tmp/task_sim.log
echo ""
echo "=== dispatcher startup ==="
cat /tmp/dispatcher.log
echo ""

echo "=== Running pipeline test (publishes all 4 markers) ==="
ros2 run secure_station_dispatch test_dispatch_pipeline \
  --ros-args -p log_path:=/home/aarock17/ros2_ws/logs/dispatch_events.jsonl

echo ""
echo "=== dispatch_events.jsonl ==="
cat ~/ros2_ws/logs/dispatch_events.jsonl | python3 -m json.tool --no-ensure-ascii 2>/dev/null || cat ~/ros2_ws/logs/dispatch_events.jsonl

# Cleanup
kill $TASK_PID $DISP_PID 2>/dev/null
echo "=== Done ==="
