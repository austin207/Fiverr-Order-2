#!/bin/bash
set -e
REPO=$(cd "$(dirname "$0")" && pwd)
docker rm -f benchmark_100map 2>/dev/null || true
docker run -d --name benchmark_100map --privileged -v "$REPO/navigation:/navigation" -v "$REPO/Benchmarking_dataset:/Benchmarking_dataset" -v "$REPO/run_100map.sh:/run_100map.sh" ros2_benchmark:humble bash /run_100map.sh
echo "Benchmark started. Monitor with:"
echo "  docker logs -f benchmark_100map 2>&1 | grep -E 'Testing Map|Completed Map|Run result|spawn failure'"
