#!/bin/bash
set -e

echo "=== [0/5] Installing Python deps ==="
pip3 install -q pandas numpy

echo "=== [1/5] Starting Xvfb virtual display ==="
mkdir -p /tmp/.X11-unix && chmod 1777 /tmp/.X11-unix
Xvfb :99 -screen 0 1280x1024x24 &
export DISPLAY=:99
sleep 2

echo "=== [2/5] Sourcing ROS 2 Humble ==="
source /opt/ros/humble/setup.bash

echo "=== [3/5] Building navigation workspace ==="
cd /navigation
colcon build --symlink-install 2>&1
source /navigation/install/setup.bash

echo "=== [4/5] Setting up 9-map resume run (maps 4-9: Corridors + Mazes) ==="
cd /Benchmarking_dataset

echo "Existing CSV rows (should be 3 from previous Rooms run):"
wc -l dataset/ann_real_world_targets.csv 2>/dev/null || echo "(no output file yet)"

echo "=== [5/5] Running benchmarker (RRT_ITERATIONS=3, START_MAP_INDEX=3) ==="
export RRT_ITERATIONS=3
export SINGLE_RUN_TIMEOUT_SEC=600
export NAV2_ACTIVE_TIMEOUT_SEC=600
export ODOM_WAIT_TIMEOUT_SEC=1200
export START_MAP_INDEX=3
export MAX_SPAWN_RETRIES=1

python3 master_benchmarker.py
EXIT_CODE=$?

echo "=== 9-map resume finished (exit $EXIT_CODE) ==="
echo "=== CSV rows written: ==="
wc -l dataset/ann_real_world_targets.csv 2>/dev/null || echo "(no output file)"
echo "=== CSV output ==="
cat dataset/ann_real_world_targets.csv 2>/dev/null || echo "(no output file)"
