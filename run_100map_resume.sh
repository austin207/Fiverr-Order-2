#!/bin/bash
set -e

# Resume the 100-map production run from a specific map index.
#
# Usage: set START_MAP_INDEX to the number of data rows already in the CSV
# (i.e. wc -l ann_real_world_targets.csv minus 1 for the header).
#
# Example: CSV has 7 lines (header + 6 rows) → set START_MAP_INDEX=6

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

echo "=== [4/5] Checking resume state ==="
cd /Benchmarking_dataset

EXISTING_ROWS=$(wc -l < dataset/ann_real_world_targets.csv 2>/dev/null || echo 1)
DATA_ROWS=$(( EXISTING_ROWS - 1 ))
echo "CSV currently has ${DATA_ROWS} data rows (maps completed so far)."
echo "Resuming from START_MAP_INDEX=${START_MAP_INDEX:-UNSET}"

if [ -z "${START_MAP_INDEX}" ]; then
  echo "ERROR: START_MAP_INDEX is not set."
  echo "  Count completed rows: wc -l Benchmarking_dataset/dataset/ann_real_world_targets.csv"
  echo "  Then launch with:     -e START_MAP_INDEX=<data_row_count>"
  exit 1
fi

echo "=== [5/5] Running benchmarker (RRT_ITERATIONS=20, resuming from map ${START_MAP_INDEX}) ==="
export RRT_ITERATIONS=20
export SINGLE_RUN_TIMEOUT_SEC=600
export NAV2_ACTIVE_TIMEOUT_SEC=600
export ODOM_WAIT_TIMEOUT_SEC=1200   # Corridors maps have 1,700+ SDF links and need up to 700s to spawn
export MAX_SPAWN_RETRIES=1

python3 master_benchmarker.py
EXIT_CODE=$?

echo "=== 100-map resume finished (exit ${EXIT_CODE}) ==="
echo "=== CSV rows written: ==="
wc -l dataset/ann_real_world_targets.csv 2>/dev/null || echo "(no output file)"
echo "=== CSV tail (last 5 rows) ==="
tail -n 5 dataset/ann_real_world_targets.csv 2>/dev/null || echo "(no output file)"
