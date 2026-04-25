# Fiverr-Order-2 — ROS 2 Nav2 Benchmarking System

Headless benchmark pipeline for three path-planning algorithms — **Dijkstra**, **A\***, and **RRT\*** — across procedurally generated Gazebo Ignition environments. Output is a structured CSV for training a machine learning model (ANN) to predict the best planner for a given map.

---

## Output

| File | Description |
|------|-------------|
| `Benchmarking_dataset/dataset/ann_real_world_targets.csv` | Final benchmark dataset |
| `logs/benchmark_10map_final.log` | Verified 10-map run log (delivered) |
| `logs/smoke_2map_final.log` | 2-map cross-map verification log |

### CSV columns

```
world_file,
D_Mem, D_Cost, D_PlanTime, D_ExecTime, D_Turns, D_Battery,
A_Mem, A_Cost, A_PlanTime, A_ExecTime, A_Turns, A_Battery,
RRT_Mem, RRT_Cost, RRT_PlanTime, RRT_ExecTime, RRT_Turns, RRT_Battery
```

> **Note:** `*_Mem` and `*_PlanTime` columns require the C++ planner plugins to publish `/planner_metrics`. This is the client's integration task — the remaining columns are fully populated.

---

## Prerequisites

- Linux with Docker installed
- Docker image `ros2_benchmark:humble` built (see below)

### Check / build the image

```bash
# Check if the image already exists
docker images | grep ros2_benchmark

# If not present, build it once (takes ~10 min)
docker build \
  -f navigation/.devcontainer/Dockerfile \
  -t ros2_benchmark:humble \
  navigation/.devcontainer/
```

---

## Running the full 100-map benchmark

```bash
docker run -d \
  --name benchmark_100map \
  --privileged \
  -v "$(pwd)/navigation:/navigation" \
  -v "$(pwd)/Benchmarking_dataset:/Benchmarking_dataset" \
  -v "$(pwd)/run_100map.sh:/run_100map.sh" \
  ros2_benchmark:humble \
  bash /run_100map.sh
```

Monitor progress:

```bash
docker logs -f benchmark_100map 2>&1 | grep -E "Testing Map|Run result|Completed Map"
```

Expected runtime: **~15 hours** (100 maps × RRT_ITERATIONS=20).

Output is written incrementally — the CSV gains one row after each map completes, so progress is never lost if the container is stopped.

Check results:

```bash
wc -l Benchmarking_dataset/dataset/ann_real_world_targets.csv
tail -n 5 Benchmarking_dataset/dataset/ann_real_world_targets.csv
```

---

## Quick smoke test (1 map, verify system is working)

```bash
docker run --rm \
  --privileged \
  -v "$(pwd)/navigation:/navigation" \
  -v "$(pwd)/Benchmarking_dataset:/Benchmarking_dataset" \
  -v "$(pwd)/smoke_run.sh:/smoke_run.sh" \
  ros2_benchmark:humble \
  bash /smoke_run.sh
```

Expected: all three planners return non-zero Cost, Turns, and Battery values for `map_59_Random_easy`.

---

## Validated 10-map batch (for intermediate verification)

```bash
docker run -d \
  --name benchmark_10map \
  --privileged \
  -v "$(pwd)/navigation:/navigation" \
  -v "$(pwd)/Benchmarking_dataset:/Benchmarking_dataset" \
  -v "$(pwd)/run_10map.sh:/run_10map.sh" \
  ros2_benchmark:humble \
  bash /run_10map.sh
```

Runs 3 Easy + 3 Moderate + 4 Hard maps with RRT_ITERATIONS=3. Expected runtime ~90 minutes. A delivered CSV from this run is in `Benchmarking_dataset/dataset/ann_real_world_targets.csv`.

---

## Available run scripts

| Script | Maps | RRT iters | Purpose |
|--------|------|-----------|---------|
| `smoke_run.sh` | 1 | 1 | Quick system check |
| `smoke_run_2map.sh` | 2 | 1 | Cross-map odom fix verification |
| `run_10map.sh` | 10 (mixed) | 3 | Intermediate batch validation |
| `run_100map.sh` | 100 (full) | 20 | **Full production dataset** |

---

## Repository structure

```
Fiverr-Order-2/
├── README.md
├── run_100map.sh                          ← full 100-map production run
├── run_10map.sh                           ← 10-map validation batch
├── smoke_run.sh                           ← 1-map smoke test
├── smoke_run_2map.sh                      ← 2-map cross-map test
├── run_benchmark.sh                       ← alternative Docker runner
├── docker_entrypoint.sh                   ← entrypoint for run_benchmark.sh
├── logs/
│   ├── benchmark_10map_final.log          ← delivered 10-map run log
│   └── smoke_2map_final.log               ← cross-map fix verification log
├── Benchmarking_dataset/
│   ├── master_benchmarker.py              ← main automation script
│   └── dataset/
│       └── gazebo_worlds/
│           ├── calibration_manifest.csv   ← full 100-map input manifest
│           ├── calibration_manifest_full.csv ← backup copy of full manifest
│           ├── manifest_10.csv            ← 10-map mixed-difficulty manifest
│           ├── smoke_manifest.csv         ← 1-map smoke manifest
│           └── map_XXXX_TYPE_DIFF.{sdf,yaml,png}
└── navigation/
    └── src/
        ├── robot_description/
        ├── robot_bringup/
        │   ├── config/
        │   │   ├── nav2_params_Dijkstra.yaml
        │   │   ├── nav2_params_A_star.yaml
        │   │   └── nav2_params_rrt.yaml
        │   └── launch/
        │       └── robot_gazebo_launch.py
        ├── robot_control/
        └── TurtleBot-RRT-Star/            ← C++ RRT* Nav2 planner plugin
```

---

## Metrics collected

| Metric | Source |
|--------|--------|
| `ExecTime` | Wall-clock navigation time |
| `Cost` | Sum of Euclidean distances along `/plan` path |
| `Turns` | Direction changes > 0.5 rad on path |
| `Battery` | Integrated `abs(linear.x) × dt × 0.01` from `/cmd_vel` |
| `Mem` | From `/planner_metrics` topic *(requires C++ plugin)* |
| `PlanTime` | From `/planner_metrics` topic *(requires C++ plugin)* |
