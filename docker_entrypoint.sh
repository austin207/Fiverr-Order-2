#!/bin/bash
# =============================================================================
# docker_entrypoint.sh
# Runs inside the container: builds the ROS 2 workspace, then executes
# master_benchmarker.py against the first 10 maps only.
# =============================================================================
set -eo pipefail

# ── 1. Source ROS 2 Humble ───────────────────────────────────────────────────
# Note: ROS 2 setup scripts reference unbound variables, so disable -u around them.
echo ">>> Sourcing ROS 2 Humble"
source /opt/ros/humble/setup.bash

# ── 2. Build workspace ───────────────────────────────────────────────────────
echo ">>> Building workspace at /ros2_ws"
mkdir -p /ros2_ws
cd /ros2_ws
colcon build --symlink-install
source /ros2_ws/install/setup.bash

# ── 3. Install required Python packages ─────────────────────────────────────
echo ">>> Installing Python dependencies"
pip install --quiet pandas numpy

# ── 4. Truncate manifest to first 10 maps ───────────────────────────────────
cd /benchmarking
MANIFEST="dataset/gazebo_worlds/calibration_manifest.csv"
BACKUP="dataset/gazebo_worlds/calibration_manifest_full.csv"

echo ">>> Backing up full manifest and creating 10-map manifest"
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
echo ">>> Starting benchmark (${MAP_COUNT} maps)"
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
