#!/bin/bash
set -e
REPO=$(cd "$(dirname "$0")" && pwd)
docker run --rm --name benchmark_smoke --privileged -v "$REPO/navigation:/navigation" -v "$REPO/Benchmarking_dataset:/Benchmarking_dataset" -v "$REPO/smoke_run.sh:/smoke_run.sh" ros2_benchmark:humble bash /smoke_run.sh
