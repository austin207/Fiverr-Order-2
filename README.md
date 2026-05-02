# Fiverr-Order-2 — ROS 2 Nav2 Benchmarking System

Headless benchmark pipeline for three path-planning algorithms — **Dijkstra**, **A\***, and **RRT\*** — across procedurally generated Gazebo Ignition environments. Output is a structured CSV for training a machine learning model (ANN) to predict the best planner for a given map.

---

## Before You Begin — Add Your C++ Planner Plugins

> **This step is required before building the Docker image. Without the plugins, `Mem` and `PlanTime` will be 0.0 in all CSV rows.**

The benchmarker reads memory usage and planning time from a ROS topic published by the C++ planner plugins. You must place your plugin source code into `navigation/src/` so it gets compiled when the Docker image builds.

### What is required

Two modified C++ Nav2 planner plugins that each publish a `std_msgs/msg/String` message on the `/planner_metrics` topic after every plan is computed, with this exact JSON payload:

```json
{"PlanTime": 0.084618, "Mem": 12}
```

| Field | Type | Unit |
|-------|------|------|
| `PlanTime` | float | seconds — time taken to compute the path |
| `Mem` | float | KB — memory allocated during planning |

### Plugins needed

| Plugin | Nav2 planner name in YAML | Role |
|--------|--------------------------|------|
| Modified **SmacPlanner2D** | `GridBased` | Used for both Dijkstra (`use_astar: false`) and A\* (`use_astar: true`). Toggled at runtime via `/planner_server/set_parameters`. |
| Modified **RRT\*** | `RRTStar` | Already provided in `navigation/src/TurtleBot-RRT-Star/`. Verify it publishes `/planner_metrics` as `std_msgs/msg/String` JSON. |

### Where to place them

```
navigation/src/
├── TurtleBot-RRT-Star/       ← already present (RRT* plugin)
├── your_smac_plugin/         ← ADD: modified SmacPlanner2D that publishes /planner_metrics
└── ... (other existing packages)
```

Copy your plugin source directory into `navigation/src/`. It must have a valid `package.xml` and `CMakeLists.txt` so `colcon build` picks it up automatically.

### Verifying the plugin publishes correctly

Inside the container (after build), confirm the topic type is correct:

```bash
ros2 topic info /planner_metrics
# Expected: Type: std_msgs/msg/String
```

If the type is `std_msgs/msg/Float32MultiArray` — that is the old format and must be updated to JSON String.

---

## Step 1 — Build the Docker Image

Run from the **repo root**. This only needs to be done once, or whenever the Dockerfile or your C++ plugin source changes.

```bash
docker build \
  -f navigation/.devcontainer/Dockerfile \
  -t ros2_benchmark:humble \
  navigation/.devcontainer/
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

The image does **not** pre-build the ROS workspace — that happens inside the container on first launch via `colcon build`.

---

## Step 2 — Verify the Build (Smoke Test)

Run a single map to confirm the full pipeline works end-to-end before committing to a long run:

```bash
docker run --rm \
  --name benchmark_smoke \
  --privileged \
  -v "$(pwd)/navigation:/navigation" \
  -v "$(pwd)/Benchmarking_dataset:/Benchmarking_dataset" \
  -v "$(pwd)/smoke_run.sh:/smoke_run.sh" \
  ros2_benchmark:humble \
  bash /smoke_run.sh
```

**Expected output:**
- Three `Run result` lines — one each for `GridBased (A*)`, `GridBased` (Dijkstra), and `RRTStar`
- `Cost`, `Turns`, and `BatteryDrain` should be non-zero
- `Mem` and `PlanTime` should be non-zero if your C++ plugins are publishing correctly
- Final line: `smoke test finished (exit 0)`

**Runtime:** ~5 minutes.

---

## Step 3 — Run the Full 100-Map Benchmark

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

The container runs detached (`-d`). Output is streamed to Docker logs. The CSV is written **incrementally** — one row per completed map — so the run can be interrupted and resumed without data loss.

### Monitor progress

```bash
# Live tail of key events
docker logs -f benchmark_100map 2>&1 | grep -E "Testing Map|Completed Map|Run result|spawn failure"

# Count rows completed (header + data: subtract 1)
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

**2. Launch `run_100map_resume.sh`, passing `START_MAP_INDEX` as an environment variable:**

```bash
docker run -d \
  --name benchmark_resume \
  --privileged \
  -e START_MAP_INDEX=6 \
  -v "$(pwd)/navigation:/navigation" \
  -v "$(pwd)/Benchmarking_dataset:/Benchmarking_dataset" \
  -v "$(pwd)/run_100map_resume.sh:/run_100map_resume.sh" \
  ros2_benchmark:humble \
  bash /run_100map_resume.sh
```

Replace `-e START_MAP_INDEX=6` with your actual data row count. The script will error and exit immediately if `START_MAP_INDEX` is not set, so a misconfigured resume never silently overwrites data.

The resume script does **not** clear the CSV — it appends from where you left off.

> **Important:** Never use `run_100map.sh` or `run_9map.sh` to resume — both scripts delete the existing CSV before starting. Always use `run_100map_resume.sh` for production resumes.

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
| `Mem` | `/planner_metrics` JSON key `"Mem"` | KB — requires C++ plugin (see Step 0) |
| `PlanTime` | `/planner_metrics` JSON key `"PlanTime"` | Seconds — requires C++ plugin (see Step 0) |

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
│           └── map_XXXX_TYPE_DIFF.sdf     ← Gazebo world files
└── navigation/
    ├── .devcontainer/
    │   └── Dockerfile                     ← Docker image definition
    └── src/
        ├── robot_description/             ← Kina robot URDF/Xacro + STL meshes
        ├── robot_bringup/
        │   ├── config/
        │   │   ├── nav2_params_Dijkstra.yaml
        │   │   ├── nav2_params_A_star.yaml
        │   │   └── nav2_params_rrt.yaml
        │   └── launch/
        │       └── robot_gazebo_launch.py
        ├── robot_control/
        └── TurtleBot-RRT-Star/            ← C++ RRT* Nav2 planner plugin (provided)
```

---

## Troubleshooting

**`Mem` and `PlanTime` are 0.0 across all rows**

The C++ plugins are either not present in `navigation/src/`, were not compiled into the image, or are publishing `Float32MultiArray` instead of `std_msgs/msg/String` JSON. Rebuild the image after adding the correct plugin source and verifying the publish format.

**Corridors or Mazes maps never spawn — repeated "robot did not spawn" messages**

Corridors worlds have ~1,700 SDF links. Physics initialisation scales super-linearly with link count and takes 688–770 s on typical hardware. The default `ODOM_WAIT_TIMEOUT_SEC=1200` covers this. If you still see timeouts, increase to `1800` by editing the relevant run script.

**Container exits with non-zero code**

```bash
docker logs benchmark_100map 2>&1 | tail -50
```

The CSV up to the point of failure is preserved. Count completed rows and resume using `run_9map_resume.sh` with `START_MAP_INDEX` set to the row count (minus 1 for the header).

**Nav2 never activates — "waiting for Nav2 lifecycle" hangs**

Increase `NAV2_ACTIVE_TIMEOUT_SEC` (default 600). On slower machines or with larger worlds, Nav2 lifecycle activation can take longer than usual.

**colcon build fails inside the container**

The workspace is built on first launch. If it fails, the error will appear in the container logs. Common causes: missing `package.xml` in your plugin directory, missing `rosdep` dependency, or a C++ compile error in your plugin. Run the container interactively to debug:

```bash
docker run -it --rm \
  --privileged \
  -v "$(pwd)/navigation:/navigation" \
  ros2_benchmark:humble \
  bash
# Inside:
source /opt/ros/humble/setup.bash
cd /navigation
colcon build --symlink-install
```
