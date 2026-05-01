# Fiverr-Order-2 — ROS 2 Nav2 Benchmarking System

Headless benchmark pipeline for three path-planning algorithms — **Dijkstra**, **A\***, and **RRT\*** — across procedurally generated Gazebo Ignition environments. Output is a structured CSV for training a machine learning model (ANN) to predict the best planner for a given map.

---

## Output

| File | Description |
|------|-------------|
| `Benchmarking_dataset/dataset/ann_real_world_targets.csv` | Benchmark dataset (grows one row per map as the run proceeds) |

### CSV columns

```
world_file,
D_Mem, D_Cost, D_PlanTime, D_ExecTime, D_Turns, D_Battery,
A_Mem, A_Cost, A_PlanTime, A_ExecTime, A_Turns, A_Battery,
RRT_Mem, RRT_Cost, RRT_PlanTime, RRT_ExecTime, RRT_Turns, RRT_Battery
```

> **`*_Mem` and `*_PlanTime`** are populated from the `/planner_metrics` ROS topic. The C++ planner plugins must publish a `std_msgs/msg/String` message containing a JSON payload with keys `"Mem"` (float, KB) and `"PlanTime"` (float, seconds) — e.g. `{"PlanTime": 0.084618, "Mem": 12}`. The subscriber is already implemented in `master_benchmarker.py`. All other columns are fully populated without any plugin changes.

---

## Prerequisites

- Linux host with Docker installed
- Docker image `ros2_benchmark:humble` built from the repo's Dockerfile (see below)

### Build the Docker image (once)

```bash
# Run from the repo root
docker build \
  -f navigation/.devcontainer/Dockerfile \
  -t ros2_benchmark:humble \
  navigation/.devcontainer/
```

This takes ~10 minutes and only needs to be done once. The image contains ROS 2 Humble, Nav2, Ignition Gazebo, and all C++ build dependencies.

---

## Running the full 100-map benchmark

From the repo root:

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

### Monitor progress

```bash
# Live tail of key events
docker logs -f benchmark_100map 2>&1 | grep -E "Testing Map|Completed Map|Run result|spawn failure"

# Count rows written so far (header + data rows = maps done + 1)
wc -l Benchmarking_dataset/dataset/ann_real_world_targets.csv

# Preview the last completed row
tail -n 1 Benchmarking_dataset/dataset/ann_real_world_targets.csv
```

### Expected runtime

| Map type | Spawn time | Per-map time (RRT×20) |
|----------|-----------|----------------------|
| Random / Rooms | ~60 s | ~25–35 min |
| Mazes | ~250 s | ~30–40 min |
| Corridors | ~700 s | ~40–50 min |

**Total: ~60–80 hours** for all 100 maps with `RRT_ITERATIONS=20`.

The CSV is written **incrementally** — one row per completed map — so the run can be stopped and resumed at any map index without losing data (see [Resuming a partial run](#resuming-a-partial-run) below).

### Environment variables (set inside `run_100map.sh`)

| Variable | Default | Meaning |
|----------|---------|---------|
| `RRT_ITERATIONS` | `20` | RRT* runs per map (results are averaged) |
| `SINGLE_RUN_TIMEOUT_SEC` | `600` | Max wall-clock seconds for one navigation attempt |
| `NAV2_ACTIVE_TIMEOUT_SEC` | `600` | Max seconds to wait for Nav2 lifecycle nodes to activate |
| `ODOM_WAIT_TIMEOUT_SEC` | `1200` | Max seconds to wait for `/odometry/filtered` after launching Gazebo. Corridors maps have 1,700+ SDF links and need up to ~700 s to fully load. |
| `MAX_SPAWN_RETRIES` | `1` | How many times to retry a map if the robot fails to spawn (odom never appears) |
| `START_MAP_INDEX` | `0` | Row index in `calibration_manifest.csv` to start from (0-based, skip header) |

---

## Resuming a partial run

If the container exits early (e.g. host reboot, OOM), resume from the last completed map without re-running earlier maps:

1. Check how many rows are in the CSV (subtract 1 for the header):
   ```bash
   wc -l Benchmarking_dataset/dataset/ann_real_world_targets.csv
   # e.g. "7 lines" means 6 maps are done → resume from index 6
   ```

2. Edit `run_9map_resume.sh` (or copy it) and set `START_MAP_INDEX` to the next map index, then launch:
   ```bash
   docker run -d \
     --name benchmark_resume \
     --privileged \
     -v "$(pwd)/navigation:/navigation" \
     -v "$(pwd)/Benchmarking_dataset:/Benchmarking_dataset" \
     -v "$(pwd)/run_9map_resume.sh:/run_9map_resume.sh" \
     ros2_benchmark:humble \
     bash /run_9map_resume.sh
   ```

   The resume script does **not** delete the existing CSV — it appends from where you left off.

> **Important:** Do not modify `START_MAP_INDEX` in `run_100map.sh` for a resume — that script always clears the CSV first. Use `run_9map_resume.sh` (or a copy of it) for resumes.

---

## Quick smoke test (verify the system works before a long run)

```bash
docker run --rm \
  --privileged \
  -v "$(pwd)/navigation:/navigation" \
  -v "$(pwd)/Benchmarking_dataset:/Benchmarking_dataset" \
  -v "$(pwd)/smoke_run.sh:/smoke_run.sh" \
  ros2_benchmark:humble \
  bash /smoke_run.sh
```

Expected: all three planners return non-zero `Cost`, `Turns`, and `Battery` values for the first map. Runtime ~5 minutes.

---

## Available run scripts

| Script | Maps | RRT iters | Purpose |
|--------|------|-----------|---------|
| `smoke_run.sh` | 1 | 1 | Quick system check |
| `smoke_run_2map.sh` | 2 | 1 | Cross-map odom isolation verification |
| `run_9map.sh` | 9 (Rooms + Corridors + Mazes) | 3 | Validation batch |
| `run_9map_resume.sh` | 9 (resume from map 4) | 3 | Resume after partial 9-map run |
| `run_100map.sh` | 100 (full) | 20 | **Full production dataset** |

---

## Repository structure

```
Fiverr-Order-2/
├── README.md
├── run_100map.sh                          ← full 100-map production run
├── run_9map.sh                            ← 9-map validation batch (Rooms+Corridors+Mazes)
├── run_9map_resume.sh                     ← resume from map 4 (Corridors+Mazes)
├── smoke_run.sh                           ← 1-map smoke test
├── smoke_run_2map.sh                      ← 2-map cross-map isolation test
├── Benchmarking_dataset/
│   ├── master_benchmarker.py              ← main automation script
│   └── dataset/
│       ├── ann_real_world_targets.csv     ← output dataset
│       └── gazebo_worlds/
│           ├── calibration_manifest.csv   ← full 100-map input manifest
│           └── map_XXXX_TYPE_DIFF.{sdf,yaml,png}
└── navigation/
    ├── .devcontainer/
    │   └── Dockerfile                     ← Docker image definition
    └── src/
        ├── robot_description/             ← Kina robot URDF/Xacro
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
| `ExecTime` | Wall-clock navigation time (seconds) |
| `Cost` | Sum of Euclidean distances along the `/plan` path |
| `Turns` | Direction changes > 0.5 rad along the path |
| `Battery` | Integrated `abs(linear.x) × dt × 0.01` from `/cmd_vel` |
| `Mem` | Published by C++ planner plugin on `/planner_metrics` (JSON key `"Mem"`, KB) |
| `PlanTime` | Published by C++ planner plugin on `/planner_metrics` (JSON key `"PlanTime"`, seconds) |

### `/planner_metrics` topic format

The benchmarker subscribes to `/planner_metrics` as a `std_msgs/msg/String` and expects a JSON payload:

```json
{"PlanTime": 0.084618, "Mem": 12}
```

The C++ planner plugins must publish this message type and format. The subscriber parses it with `json.loads()` and maps `"Mem"` → `*_Mem` and `"PlanTime"` → `*_PlanTime` in the CSV.

---

## Troubleshooting

**Corridors or Mazes maps never spawn (odom timeout)**

Corridors worlds contain ~1,700 SDF links. Ignition Gazebo physics initialization scales super-linearly with link count and can take 700+ seconds inside Docker. The default `ODOM_WAIT_TIMEOUT_SEC=1200` in `run_100map.sh` accounts for this. If you see repeated `"robot did not spawn. Triggering retry"` messages, your machine may be slower than the test host — increase `ODOM_WAIT_TIMEOUT_SEC` further (e.g. `1800`).

**`Mem` and `PlanTime` are all 0.0**

The C++ planner plugins in the container are not publishing `/planner_metrics`, or are publishing it as `Float32MultiArray` (old format). Rebuild the container after ensuring the plugins publish `std_msgs/msg/String` JSON. All other metrics are unaffected.

**Container exits with non-zero code**

Check the logs:
```bash
docker logs benchmark_100map 2>&1 | tail -50
```
The CSV up to the point of failure is preserved. Find the last completed map index and resume using `run_9map_resume.sh` with `START_MAP_INDEX` set accordingly.
