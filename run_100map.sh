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
if [ -f "install/setup.bash" ]; then
  echo "Install already exists — skipping rebuild."
else
  colcon build --symlink-install 2>&1
fi
source /navigation/install/setup.bash

echo "=== [4/5] Setting up full 100-map run ==="
cd /Benchmarking_dataset
cp dataset/gazebo_worlds/calibration_manifest.csv dataset/gazebo_worlds/calibration_manifest.csv.bak
echo "Using full calibration_manifest.csv (100 maps)"
wc -l dataset/gazebo_worlds/calibration_manifest.csv

# Clear any previous partial results
rm -f dataset/ann_real_world_targets.csv

echo "=== [5/5] Running benchmarker (RRT_ITERATIONS=20) ==="
export RRT_ITERATIONS=20
export SINGLE_RUN_TIMEOUT_SEC=300
export NAV2_ACTIVE_TIMEOUT_SEC=120

python3 master_benchmarker.py
EXIT_CODE=$?

echo "=== Restoring manifest ==="
cp dataset/gazebo_worlds/calibration_manifest.csv.bak dataset/gazebo_worlds/calibration_manifest.csv

echo "=== 100-map run finished (exit $EXIT_CODE) ==="
echo "=== CSV rows written: ==="
wc -l dataset/ann_real_world_targets.csv 2>/dev/null || echo "(no output file)"
echo "=== CSV preview (last 5 rows) ==="
tail -n 5 dataset/ann_real_world_targets.csv 2>/dev/null || echo "(no output file)"
