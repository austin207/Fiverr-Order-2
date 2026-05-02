#!/bin/bash
set -e
REPO=$(cd "$(dirname "$0")" && pwd)

if [ -z "$1" ]; then
    DATA_ROWS=$(( $(wc -l < "$REPO/Benchmarking_dataset/dataset/ann_real_world_targets.csv" 2>/dev/null || echo 1) - 1 ))
    echo "Usage: bash resume_benchmark.sh <start_index>"
    echo ""
    echo "Your CSV currently has $DATA_ROWS completed maps."
    echo "Run:  bash resume_benchmark.sh $DATA_ROWS"
    exit 1
fi

docker rm -f benchmark_resume 2>/dev/null || true
docker run -d --name benchmark_resume --privileged -e START_MAP_INDEX="$1" -v "$REPO/navigation:/navigation" -v "$REPO/Benchmarking_dataset:/Benchmarking_dataset" -v "$REPO/run_100map_resume.sh:/run_100map_resume.sh" ros2_benchmark:humble bash /run_100map_resume.sh
echo "Resuming from map $1. Monitor with:"
echo "  docker logs -f benchmark_resume 2>&1 | grep -E 'Testing Map|Completed Map|Run result|spawn failure'"
