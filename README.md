# Fiverr-Order-2

ROS 2 Humble + Gazebo benchmark pipeline for Dijkstra, A*, and RRT*.

## What this project produces

- Output CSV: Benchmarking_dataset/dataset/ann_real_world_targets.csv
- Columns include:
	- D_Mem, D_Cost, D_PlanTime, D_ExecTime, D_Turns, D_Battery
	- A_Mem, A_Cost, A_PlanTime, A_ExecTime, A_Turns, A_Battery
	- RRT_Mem, RRT_Cost, RRT_PlanTime, RRT_ExecTime, RRT_Turns, RRT_Battery

## Prerequisites

- Linux with Docker installed
- NVIDIA runtime is optional (script warns if not detected)

## Quick run (client reference)

From repo root:

chmod +x run_benchmark.sh docker_entrypoint.sh
./run_benchmark.sh

Logs are written to:

- benchmark_run.log

CSV is written to:

- Benchmarking_dataset/dataset/ann_real_world_targets.csv

## Fast smoke run (recommended for quick validation)

Run a single map with minimal RRT iterations:

MAP_COUNT=1 RRT_ITERATIONS=1 ./run_benchmark.sh

## Runtime controls

These environment variables are supported:

- MAP_COUNT (default: 10)
	- Limits how many rows from calibration_manifest.csv are used.
- RRT_ITERATIONS (default: 20)
	- Number of RRT attempts per map.
- PREFER_EASY_MAP (default: 0)
	- If MAP_COUNT=1 and set to 1, tries to select an easy map row from manifest.
	- Falls back safely to the first data row if no easy row exists.

Example:

MAP_COUNT=1 RRT_ITERATIONS=3 PREFER_EASY_MAP=1 ./run_benchmark.sh

## Why metrics now appear even when navigation fails

Benchmark logic now writes collected planner/path metrics even when follow_path aborts.

Meaning:

- Dijkstra/A* columns are populated when path planning returns data,
	even if controller execution does not reach goal.
- RRT columns populate when at least one iteration produces a path.

If RRT fails all attempts for a map, RRT columns remain empty for that row.

## Verify result quickly

wc -l Benchmarking_dataset/dataset/ann_real_world_targets.csv
tail -n 5 Benchmarking_dataset/dataset/ann_real_world_targets.csv

## Notes

- This benchmark can take significant time on hard maps.
- For deadline scenarios, run smoke first, then increase MAP_COUNT and RRT_ITERATIONS.
