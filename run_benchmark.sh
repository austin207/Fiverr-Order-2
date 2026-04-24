#!/bin/bash
# =============================================================================
# run_benchmark.sh
# ─────────────────────────────────────────────────────────────────────────────
# Tasks performed:
#   1. Builds the Docker image from navigation/.devcontainer/Dockerfile
#      (base: osrf/ros:humble-desktop + Gazebo Fortress + Nav2)
#   2. Runs the container headless with --gpus all (RTX 5070 passthrough)
#   3. Mounts:
#        navigation/src/  →  /ros2_ws/src   (ROS 2 source packages)
#        Benchmarking_dataset/  →  /benchmarking
#   4. Inside the container the entrypoint:
#        - colcon build --symlink-install
#        - runs master_benchmarker.py against the first 10 maps
#   5. Streams logs to benchmark_run.log in this directory
# =============================================================================
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAVIGATION_SRC="${REPO_DIR}/navigation/src"
BENCHMARKING_DIR="${REPO_DIR}/Benchmarking_dataset"
ENTRYPOINT="${REPO_DIR}/docker_entrypoint.sh"

IMAGE_NAME="ros2_benchmark:humble"
LOG_FILE="${REPO_DIR}/benchmark_run.log"

# ── Verify prerequisites ─────────────────────────────────────────────────────
if ! command -v docker &>/dev/null; then
    echo "ERROR: docker not found. Install Docker first." >&2
    exit 1
fi
if ! docker info --format '{{.Runtimes}}' 2>/dev/null | grep -q nvidia; then
    echo "WARNING: NVIDIA container runtime not detected." \
         "GPU passthrough may fail. Continuing anyway..." >&2
fi

# ── Build image ──────────────────────────────────────────────────────────────
echo "=== [1/3] Building Docker image: ${IMAGE_NAME} ==="
docker build \
    --file "${REPO_DIR}/navigation/.devcontainer/Dockerfile" \
    --tag  "${IMAGE_NAME}" \
    "${REPO_DIR}/navigation/.devcontainer/"

# ── Run container ────────────────────────────────────────────────────────────
echo "=== [2/3] Launching benchmark container ==="
echo "          Log → ${LOG_FILE}"

docker run \
    --rm \
    --user root \
    --network host \
    --env MAP_COUNT="${MAP_COUNT:-10}" \
    --env RRT_ITERATIONS="${RRT_ITERATIONS:-20}" \
    --env SINGLE_RUN_TIMEOUT_SEC="${SINGLE_RUN_TIMEOUT_SEC:-0}" \
    --env MAP_TIMEOUT_SEC="${MAP_TIMEOUT_SEC:-0}" \
    --env NAV2_ACTIVE_TIMEOUT_SEC="${NAV2_ACTIVE_TIMEOUT_SEC:-0}" \
    --env PREFER_EASY_MAP="${PREFER_EASY_MAP:-0}" \
    --env NVIDIA_DRIVER_CAPABILITIES=all \
    --env LIBGL_ALWAYS_SOFTWARE=1 \
    --env ROS_DOMAIN_ID=0 \
    --env IGN_PARTITION=benchmark \
    --env RCUTILS_LOGGING_MIN_SEVERITY=INFO \
    -v "${NAVIGATION_SRC}:/ros2_ws/src:rw" \
    -v "${BENCHMARKING_DIR}:/benchmarking:rw" \
    -v "${ENTRYPOINT}:/entrypoint.sh:ro" \
    --workdir /benchmarking \
    "${IMAGE_NAME}" \
    bash /entrypoint.sh \
    2>&1 | tee "${LOG_FILE}"

# ── Show output CSV ──────────────────────────────────────────────────────────
echo ""
echo "=== [3/3] Results written to ==="
echo "          ${BENCHMARKING_DIR}/dataset/ann_real_world_targets.csv"
echo ""
if [ -f "${BENCHMARKING_DIR}/dataset/ann_real_world_targets.csv" ]; then
    echo "--- CSV preview ---"
    cat "${BENCHMARKING_DIR}/dataset/ann_real_world_targets.csv"
else
    echo "WARNING: Output CSV not found. Check ${LOG_FILE} for errors."
fi
