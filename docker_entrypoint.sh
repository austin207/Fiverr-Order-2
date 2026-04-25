#!/bin/bash
# =============================================================================
# docker_entrypoint.sh
# Runs inside the container: builds the ROS 2 workspace, then executes
# master_benchmarker.py against the first 10 maps only.
# =============================================================================
set -eo pipefail

# ── 1. Start virtual display (required even in headless mode for some Qt components) ──
echo ">>> Starting Xvfb virtual display"
mkdir -p /tmp/.X11-unix && chmod 1777 /tmp/.X11-unix
Xvfb :99 -screen 0 1280x1024x24 &
export DISPLAY=:99
sleep 2

# ── 2. Source ROS 2 Humble ───────────────────────────────────────────────────
echo ">>> Sourcing ROS 2 Humble"
source /opt/ros/humble/setup.bash

# ── 3. Build workspace ───────────────────────────────────────────────────────
echo ">>> Building workspace at /ros2_ws"
mkdir -p /ros2_ws
cd /ros2_ws
if [ -f "install/setup.bash" ]; then
  echo ">>> Install already exists — skipping rebuild."
else
  colcon build --symlink-install
fi
source /ros2_ws/install/setup.bash

# ── 4. Install required Python packages ─────────────────────────────────────
echo ">>> Installing Python dependencies"
pip install --quiet pandas numpy

# ── 4b. Software renderer env vars ───────────────────────────────────────────
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe

# ── 5. Truncate manifest to requested MAP_COUNT ──────────────────────────────
cd /benchmarking
MANIFEST="dataset/gazebo_worlds/calibration_manifest.csv"
# Use a temp backup so we never overwrite calibration_manifest_full.csv
BACKUP="/tmp/calibration_manifest_original.csv"

echo ">>> Backing up manifest and limiting to ${MAP_COUNT:-10} maps"
cp "${MANIFEST}" "${BACKUP}"
MAP_COUNT="${MAP_COUNT:-10}"
LINES_TO_KEEP=$((MAP_COUNT + 1))
if [ "${MAP_COUNT}" = "1" ] && [ "${PREFER_EASY_MAP:-0}" = "1" ]; then
    # keep header + first easy-map row (fallback: first data row)
    easy_row="$(grep -m 1 "_easy\.sdf" "${MANIFEST}" || true)"
    {
        head -n 1 "${MANIFEST}"
        if [ -n "${easy_row}" ]; then
            printf '%s\n' "${easy_row}"
        else
            sed -n '2p' "${MANIFEST}"
        fi
    } > /tmp/manifest_limited.csv
else
    # keep header + MAP_COUNT data rows
    head -n "${LINES_TO_KEEP}" "${MANIFEST}" > /tmp/manifest_limited.csv
fi
cp /tmp/manifest_limited.csv "${MANIFEST}"

# Ensure manifest is always restored on exit (including SIGINT/SIGTERM/error)
restore_manifest() {
    echo ">>> Restoring original manifest (trap)"
    cp "${BACKUP}" "${MANIFEST}" 2>/dev/null || true
}
trap restore_manifest EXIT

# ── 5. Run benchmarker ───────────────────────────────────────────────────────
echo ">>> Starting benchmark (${MAP_COUNT:-10} maps)"
python3 master_benchmarker.py
STATUS=$?

# ── 6. Restore original manifest ────────────────────────────────────────────
echo ">>> Restoring original manifest"
cp "${BACKUP}" "${MANIFEST}"

if [ "${STATUS}" -ne 0 ]; then
    echo ">>> benchmarker exited with code ${STATUS}"
    exit "${STATUS}"
fi

# ── 7. Print output CSV ──────────────────────────────────────────────────────
echo ""
echo "=== ann_real_world_targets.csv ==="
cat dataset/ann_real_world_targets.csv
