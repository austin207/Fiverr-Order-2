# Fiverr-Order-2 — ROS 2 Nav2 Benchmarking System

Headless benchmark pipeline for three path-planning algorithms — **Dijkstra**, **A\***, and **RRT\*** — across procedurally generated Gazebo Ignition environments. Output is a structured CSV for training a machine learning model (ANN) to predict the best planner for a given map.

The C++ planner plugins (`SmacMetricsPlanner2D` and `NavFnMetricsPlanner`) are already included in `navigation/src/robot_bringup/src/`. No external plugins need to be added.

---

## Step 0 — Delete Old Build Artifacts (Required if re-running)

> **Do this before every fresh Docker run. Skipping it causes the container to use a stale compiled workspace that may be missing the C++ metrics plugins, giving `Mem = 0.0` and `PlanTime = 0.0` in all rows.**

```bash
rm -rf navigation/build navigation/install navigation/log
```

If this is your first time running from this zip, these directories won't exist — that's fine, skip this step.

---

## Step 1 — Build the Docker Image

From the repo root (the folder containing this README):

```bash
bash build_image.sh
```

**Build time:** ~10–15 minutes on the first run. Subsequent builds are faster due to layer caching.

**What the image contains:**
- Ubuntu 22.04 + ROS 2 Humble (desktop)
- Ignition Gazebo Fortress
- Nav2 full stack (`ros-humble-navigation2`, `ros-humble-nav2-bringup`, `ros-humble-nav2-mppi-controller`)
- Robot localisation (`ros-humble-robot-localization`)
- ROS 2 Control stack
- Xvfb (virtual display for headless Gazebo rendering)
- Python 3 + colcon build tools

The image does **not** pre-build the ROS workspace. The C++ plugins are compiled from source at container startup via `colcon build`.

---

## Step 2 — Smoke Test (Strongly Recommended)

Run one easy map to confirm the full pipeline works before committing to the 60–80 hour production run:

```bash
bash test_smoke.sh
```

**Expected output:**
- Three `Run result` lines — one each for `GridBased (A*)`, `GridBased` (Dijkstra), and `RRTStar`
- `Cost`, `Turns`, and `BatteryDrain` are non-zero
- `Mem` and `PlanTime` are **non-zero** (proves the C++ plugin compiled and is publishing)
- Final line: `smoke test finished (exit 0)`

**Runtime:** ~5 minutes.

If `Mem` and `PlanTime` are 0.0, the C++ plugin did not compile. Check container logs for build errors. The most common cause is leftover `navigation/install/` from a previous run — go back to Step 0.

---

## Step 3 — Run the Full 100-Map Benchmark

```bash
bash run_benchmark.sh
```

The container runs detached in the background. The CSV is written **incrementally** — one row per completed map — so the run can be interrupted and resumed without data loss.

### Monitor progress

```bash
# Live tail of key events
docker logs -f benchmark_100map 2>&1 | grep -E "Testing Map|Completed Map|Run result|spawn failure"

# Count rows completed (subtract 1 for the header row)
wc -l Benchmarking_dataset/dataset/ann_real_world_targets.csv

# Preview the most recent completed row
tail -n 1 Benchmarking_dataset/dataset/ann_real_world_targets.csv
```

### Expected runtimes

| Map type | Gazebo spawn time | Per-map time (RRT×20) |
|----------|--------------------|----------------------|
| Random / Rooms | ~60 s | ~25–35 min |
| Mazes | ~250 s | ~30–40 min |
| Corridors | ~700 s | ~40–50 min |

**Total: ~60–80 hours** for all 100 maps with `RRT_ITERATIONS=20`.

### Configuration — environment variables

These are set inside `run_100map.sh`. Edit the script to override them before launching.

| Variable | Default | Meaning |
|----------|---------|---------|
| `RRT_ITERATIONS` | `20` | RRT\* runs per map (results are averaged) |
| `SINGLE_RUN_TIMEOUT_SEC` | `600` | Max wall-clock seconds for one navigation attempt |
| `NAV2_ACTIVE_TIMEOUT_SEC` | `600` | Max seconds waiting for Nav2 lifecycle nodes to activate |
| `ODOM_WAIT_TIMEOUT_SEC` | `1200` | Max seconds waiting for `/odometry/filtered` after Gazebo launch. Corridors maps have 1,700+ SDF links and need up to ~700 s to fully load. |
| `MAX_SPAWN_RETRIES` | `1` | Retries if the robot fails to spawn (odom never appears within the timeout) |
| `START_MAP_INDEX` | `0` | Row index in `calibration_manifest.csv` to start from. Used for resuming partial runs. |

---

## Resuming a Partial Run

If the container exits early (host reboot, OOM, etc.), resume from the last completed map without re-running earlier rows:

**1. Check how many maps are done:**

```bash
wc -l Benchmarking_dataset/dataset/ann_real_world_targets.csv
# "7 lines" = header + 6 data rows → 6 maps done → resume from index 6
```

**2. Resume, passing the row count as an argument:**

```bash
bash resume_benchmark.sh 6
```

Replace `6` with your actual data row count. If you run `bash resume_benchmark.sh` with no argument, it will tell you exactly how many rows are in the CSV and what number to use.

The resume script does **not** clear the CSV — it appends from where you left off.

> **Important:** Never use `run_100map.sh` to resume — it deletes the existing CSV before starting. Always use `run_100map_resume.sh` for resumes.

---

## Available Run Scripts

| Script | Maps | RRT iters | Clears CSV | Purpose |
|--------|------|-----------|------------|---------|
| `smoke_run.sh` | 1 (Random easy) | 1 | yes | Quick system check |
| `smoke_run_2map.sh` | 2 (Random easy) | 1 | yes | Cross-map odom isolation test |
| `run_9map.sh` | 9 (Rooms + Corridors + Mazes) | 3 | **yes** | Validation batch, fresh start |
| `run_9map_resume.sh` | resumes from index 3 | 3 | **no** | Resume after Rooms done |
| `run_6map_resume.sh` | resumes from index 2 | 3 | **no** | Resume 6-map test after Rooms done |
| `run_10map.sh` | 10 (mixed difficulty) | 3 | yes | Mixed-difficulty validation |
| `run_100map.sh` | 100 (full) | 20 | **yes** | **Full production dataset** |
| `run_100map_resume.sh` | 100 (resume from any index) | 20 | **no** | **Resume interrupted production run** |

---

## Output

| File | Description |
|------|-------------|
| `Benchmarking_dataset/dataset/ann_real_world_targets.csv` | Benchmark dataset — one row per map, written incrementally |

### CSV columns

```
world_file,
D_Mem, D_Cost, D_PlanTime, D_ExecTime, D_Turns, D_Battery,
A_Mem, A_Cost, A_PlanTime, A_ExecTime, A_Turns, A_Battery,
RRT_Mem, RRT_Cost, RRT_PlanTime, RRT_ExecTime, RRT_Turns, RRT_Battery
```

### Metrics collected

| Metric | Source | Notes |
|--------|--------|-------|
| `ExecTime` | Wall-clock navigation time | Seconds, around `followPath()` call |
| `Cost` | Sum of Euclidean distances along `/plan` path | Lower = more direct path |
| `Turns` | Direction changes > 0.5 rad on path | Lower = smoother path |
| `Battery` | Integrated `abs(linear.x) × dt × 0.01` from `/cmd_vel` | Proxy for energy use |
| `Mem` | `/planner_metrics` JSON key `"Mem"` | KB — from `SmacMetricsPlanner2D` C++ plugin |
| `PlanTime` | `/planner_metrics` JSON key `"PlanTime"` | Seconds — from `SmacMetricsPlanner2D` C++ plugin |

---

## Repository Structure

```
Fiverr-Order-2/
├── README.md
├── run_100map.sh                          ← full 100-map production run (clears CSV)
├── run_100map_resume.sh                   ← resume 100-map run from any index (safe)
├── run_9map.sh                            ← 9-map validation batch
├── run_9map_resume.sh                     ← resume from map 4 (Corridors+Mazes)
├── run_6map_resume.sh                     ← resume 6-map test from map 2
├── run_10map.sh                           ← 10-map mixed-difficulty batch
├── smoke_run.sh                           ← 1-map quick smoke test
├── smoke_run_2map.sh                      ← 2-map cross-map odom isolation test
├── Benchmarking_dataset/
│   ├── master_benchmarker.py              ← main automation script
│   └── dataset/
│       ├── ann_real_world_targets.csv     ← output dataset
│       └── gazebo_worlds/
│           ├── calibration_manifest.csv   ← active input manifest (100 maps)
│           ├── smoke_manifest.csv         ← 1-map manifest for smoke test
│           └── map_XXXX_TYPE_DIFF.sdf     ← Gazebo world files
└── navigation/
    ├── .devcontainer/
    │   └── Dockerfile                     ← Docker image definition
    └── src/
        ├── robot_description/             ← Kina robot URDF/Xacro + STL meshes
        ├── robot_bringup/
        │   ├── src/
        │   │   ├── smac_metrics_planner_2d.cpp  ← SmacPlanner2D + /planner_metrics (primary)
        │   │   └── navfn_metrics_planner.cpp    ← NavFn + /planner_metrics (reference)
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

## Troubleshooting

**`Mem` and `PlanTime` are 0.0 across all rows**

The C++ plugin was not compiled into the workspace. Most likely cause: a stale `navigation/install/` directory from a previous container run was reused. Delete it and re-run:
```bash
rm -rf navigation/build navigation/install navigation/log
```
Then repeat from Step 1.

**Spawn failure warning on map 1 — "attempt 1/2"**

This is normal. The first map in a fresh container takes longer to initialise (DDS discovery, Gazebo cold-start). The retry mechanism handles it automatically. Map 1 will complete on the second attempt.

**Corridors or Mazes maps never spawn — repeated "robot did not spawn" messages**

Corridors worlds have ~1,700 SDF links. Physics initialisation takes 688–770 s on typical hardware. The default `ODOM_WAIT_TIMEOUT_SEC=1200` covers this. If timeouts persist, increase to `1800` by editing the run script.

**Container exits with non-zero code**

```bash
docker logs benchmark_100map 2>&1 | tail -50
```

The CSV up to the point of failure is preserved. Count completed rows and resume with `run_100map_resume.sh`.

**Nav2 never activates — "waiting for Nav2 lifecycle" hangs**

Increase `NAV2_ACTIVE_TIMEOUT_SEC` (default 600). On slower machines Nav2 lifecycle activation can take longer.

**colcon build fails inside the container**

Run the container interactively to see the full error:

```bash
REPO=$(cd "$(dirname "$0")" && pwd)
docker run -it --rm --privileged -v "$REPO/navigation:/navigation" ros2_benchmark:humble bash
# Inside the container:
source /opt/ros/humble/setup.bash
cd /navigation
colcon build --symlink-install
```

Common causes: missing `package.xml`, missing `rosdep` dependency, or a C++ compile error in a plugin.
