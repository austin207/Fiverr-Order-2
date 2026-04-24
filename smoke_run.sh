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

echo "=== [4/5] Setting up smoke test ==="
cd /Benchmarking_dataset
cp dataset/gazebo_worlds/calibration_manifest.csv dataset/gazebo_worlds/calibration_manifest.csv.bak
cp dataset/gazebo_worlds/smoke_manifest.csv dataset/gazebo_worlds/calibration_manifest.csv
echo "Smoke manifest active (1 map: map_59_Random_easy)"

echo "=== [5/5] Running benchmarker (RRT_ITERATIONS=1) ==="
export RRT_ITERATIONS=1
export SINGLE_RUN_TIMEOUT_SEC=180
export NAV2_ACTIVE_TIMEOUT_SEC=120

python3 master_benchmarker.py
EXIT_CODE=$?

echo "=== Restoring original manifest ==="
cp dataset/gazebo_worlds/calibration_manifest.csv.bak dataset/gazebo_worlds/calibration_manifest.csv

echo "=== Smoke test finished (exit $EXIT_CODE) ==="
echo "=== CSV output ==="
cat dataset/ann_real_world_targets.csv 2>/dev/null || echo "(no output file)"
