# CLAUDE.md — Project Context
## Fiverr Order 2: ROS 2 Nav2 Benchmarking System

---

## Project Overview

This repository contains a complete ROS 2 (Humble) benchmarking system that measures the performance of three pathfinding algorithms — **Dijkstra**, **A\***, and **RRT\*** — across procedurally generated Gazebo Ignition simulation environments. The output is a structured CSV dataset for training a machine learning model (ANN) to predict the best planner for a given map.

---

## Repository Structure

```
Fiverr-Order-2/
├── CLAUDE.md                              ← this file
├── run_100map.sh                          ← full 100-map production run
├── run_9map.sh                            ← 9-map validation batch (fresh run)
├── run_9map_resume.sh                     ← resume 9-map run from map index 3
├── run_6map_resume.sh                     ← resume 6-map test from map index 2
├── run_10map.sh                           ← 10-map mixed-difficulty batch
├── smoke_run.sh                           ← 1-map quick system check
├── smoke_run_2map.sh                      ← 2-map cross-map odom isolation test
├── run_benchmark.sh                       ← legacy Docker runner (docker_entrypoint.sh)
├── Benchmarking_dataset/
│   ├── master_benchmarker.py              ← main automation script
│   └── dataset/
│       ├── ann_real_world_targets.csv     ← output dataset (appended incrementally)
│       └── gazebo_worlds/
│           ├── calibration_manifest.csv  ← active input manifest
│           └── map_XXXX_TYPE_DIFF.sdf    ← Gazebo world files
└── navigation/
    ├── .devcontainer/
    │   └── Dockerfile                     ← Docker image (ros2_benchmark:humble)
    └── src/
        ├── robot_description/             ← Kina robot URDF/Xacro + STL meshes
        ├── robot_control/                 ← placeholder Python velocity control pkg
        ├── robot_bringup/
        │   ├── config/
        │   │   ├── nav2_params_Dijkstra.yaml
        │   │   ├── nav2_params_A_star.yaml
        │   │   └── nav2_params_rrt.yaml
        │   └── launch/
        │       └── robot_gazebo_launch.py ← launched by master_benchmarker.py
        └── TurtleBot-RRT-Star/            ← C++ Nav2 RRT* global planner plugin
```

---

## Components

### 1. `master_benchmarker.py`

ROS 2 Python node (`MasterBenchmarker`) that automates the full benchmark pipeline:

1. Reads `calibration_manifest.csv` (world file, spawn/goal coords, obstacle count)
2. For each map:
   - Launches Gazebo + Nav2 headless via `robot_gazebo_launch.py`
   - Runs **A\*** first: calls `toggle_astar_param(True)` → `execute_single_run("GridBased")` → `toggle_astar_param(False)`
   - Runs **Dijkstra**: `execute_single_run("GridBased")` with `use_astar=False` (default)
   - Runs **RRT\*** (`RRT_ITERATIONS` runs, averaged): `execute_single_run("RRTStar")`
   - Resets robot between runs via `ign service set_pose` + AMCL `/initialpose`
   - Appends result row to `dataset/ann_real_world_targets.csv`
   - Kills the Gazebo process group + aggressive `pkill` cleanup to reclaim RAM

#### Planner Configuration

Both Dijkstra and A\* use the **same Nav2 plugin** `GridBased` (SmacPlanner2D). The client's `nav2_params_A_star.yaml` and `nav2_params_Dijkstra.yaml` both have `planner_plugins: ["GridBased"]`. The toggle between them is done at runtime by calling `/planner_server/set_parameters` to flip `GridBased.use_astar`:

```python
def toggle_astar_param(self, use_astar: bool):
    # Calls /planner_server/set_parameters to set GridBased.use_astar
```

**Do not** use `planner_id="GridBasedAstar"` — that plugin does not exist and will crash immediately.

#### Metric Collection

| Metric | Method |
|--------|--------|
| `ExecTime` | Wall-clock time around `followPath()` in Python |
| `Cost` | Sum of Euclidean distances between poses on `/plan` topic |
| `Turns` | Count of angle changes > 0.5 rad between path segments on `/plan` |
| `Mem` | From `/planner_metrics` topic (JSON String, key `"Mem"`, KB) |
| `PlanTime` | From `/planner_metrics` topic (JSON String, key `"PlanTime"`, seconds) |
| `BatteryDrain` | Integrated `abs(linear.x) × dt × 0.01` from `/cmd_vel` |

#### `/planner_metrics` Topic Format

The subscriber expects `std_msgs/msg/String` with a JSON payload:
```json
{"PlanTime": 0.084618, "Mem": 12}
```
Parsed with `json.loads()`. The C++ planner plugins must publish this exact format and type. If they publish `Float32MultiArray` (old format), `Mem` and `PlanTime` will silently read as `0.0`.

#### Output CSV Columns

```
world_file,
D_Mem, D_Cost, D_PlanTime, D_ExecTime, D_Turns, D_Battery,
A_Mem, A_Cost, A_PlanTime, A_ExecTime, A_Turns, A_Battery,
RRT_Mem, RRT_Cost, RRT_PlanTime, RRT_ExecTime, RRT_Turns, RRT_Battery
```

#### Key Environment Variables

| Variable | Default | Purpose |
|----------|---------|---------|
| `RRT_ITERATIONS` | `20` | RRT\* runs per map (averaged) |
| `SINGLE_RUN_TIMEOUT_SEC` | `600` | Max navigation time per planner run |
| `NAV2_ACTIVE_TIMEOUT_SEC` | `600` | Max time waiting for Nav2 lifecycle to activate |
| `ODOM_WAIT_TIMEOUT_SEC` | `600` | Max time waiting for `/odometry/filtered` after Gazebo launch. **Must be 1200 for Corridors/Mazes maps** (see Bug 7). |
| `MAX_SPAWN_RETRIES` | `3` | Retries on odom timeout (robot didn't spawn). Nav failures do NOT retry. |
| `START_MAP_INDEX` | `0` | Row index in manifest to start from — used for resuming partial runs. |

---

### 2. Run Scripts

| Script | Maps | RRT iters | `ODOM_WAIT` | Clears CSV | Purpose |
|--------|------|-----------|-------------|------------|---------|
| `smoke_run.sh` | 1 (Random easy) | 1 | 600s | yes | Quick sanity check |
| `smoke_run_2map.sh` | 2 (Random easy) | 1 | 600s | yes | Cross-map odom isolation test |
| `run_9map.sh` | 9 (Rooms+Corridors+Mazes) | 3 | **1200s** | yes | Validation batch, fresh run |
| `run_9map_resume.sh` | resumes from index 3 | 3 | **1200s** | no | Resume after Rooms done |
| `run_6map_resume.sh` | resumes from index 2 | 3 | **1200s** | no | Resume 6-map test after Rooms done |
| `run_10map.sh` | 10 (mixed) | 3 | **1200s** | yes | Mixed-difficulty batch |
| `run_100map.sh` | 100 (full) | 20 | **1200s** | yes | **Full production dataset** |

> **Critical:** Smoke scripts use 600s odom timeout because they only run Random/easy maps (<60s spawn). All other scripts must use 1200s — Corridors maps have 1,700+ SDF links and take up to ~700s to spawn in Docker.

---

### 3. `calibration_manifest.csv`

Active input manifest. Swapped between runs for different subsets:

**Columns:** `world_file, category, difficulty, obstacles, a_mem, a_time, a_turns, d_mem, d_time, d_turns, rrt_mem, rrt_time, rrt_turns, spawn_x, spawn_y, spawn_z, goal_x, goal_y, wall_count`

**Map type spawn times in Docker (observed):**
- Random / Rooms: ~60s
- Mazes: ~250s
- Corridors: ~688–770s (1,700+ SDF links, physics init is super-linear in link count)

---

## All Bug Fixes Applied

### BUG 1 — Node never spun during navigation (callbacks silently dropped)

**File:** `master_benchmarker.py`

**Problem:** The blocking `while not isTaskComplete(): time.sleep(0.1)` loop never called `rclpy.spin_once()`, so `/plan` and `/planner_metrics` callbacks never fired. All metrics read as 0.0.

**Fix:**
```python
while not self.navigator.isTaskComplete():
    rclpy.spin_once(self, timeout_sec=0.0)
    time.sleep(0.1)
```

---

### BUG 2 — Wrong localizer name in `waitUntilNav2Active`

**File:** `master_benchmarker.py`

**Problem:** `localizer='bt_navigator'` is the navigator, not the localizer — Nav2 could start before AMCL was ready.

**Fix:** `self.navigator.waitUntilNav2Active(localizer='amcl')`

---

### BUG 3 — Battery drain metric missing entirely

**File:** `master_benchmarker.py`

**Problem:** No energy efficiency metric in the dataset.

**Fix:** Added `/cmd_vel` subscriber integrating `abs(linear.x) × dt × 0.01`. Reset per run. Written as `D_Battery`, `A_Battery`, `RRT_Battery`.

---

### BUG 4 — Cross-map stale TF odom causes A*/RRT* instant "0 poses" on map 2+

**File:** `master_benchmarker.py`

**Problem:** `tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())` with `time=0` returns the highest-timestamp transform — map 1's stale odom coordinates. This produced a grossly wrong map→odom static TF on all subsequent maps, causing MPPI controller to find 0 path poses.

**Fix:** Subscribe directly to `/odometry/filtered`; use those x/y values in `_update_map_odom_static_tf()` instead of the TF lookup. Reset `current_odom_x/y = 0.0` at the start of each map.

**Verified:** 2-map smoke test — map 2 shows fresh `robot_odom=(21.878, -21.370)`, A*/RRT* get 138/458 poses respectively.

---

### BUG 5 — Dijkstra (NavFn) fails on Corridors and Mazes with `GETPATH RETURNED NONE`

**File:** `nav2_params_Dijkstra.yaml` / `nav2_params_rrt.yaml`

**Problem:** `NavfnPlanner` uses gradient wavefront propagation. In tight corridors/mazes, inflation creates high-cost cells that collapse the gradient — the wavefront can't propagate to the goal even when a valid path exists.

**Fix:** Replace `GridBased` plugin with `SmacPlanner2D` and `cost_travel_multiplier: 0.0` (uniform-cost = Dijkstra). A\* uses `cost_travel_multiplier: 2.0`.

---

### BUG 6 — `/planner_metrics` subscribed as `Float32MultiArray` instead of JSON String

**File:** `master_benchmarker.py`

**Problem:** The subscriber used `Float32MultiArray` with indexed access (`msg.data[0]`, `msg.data[1]`, `msg.data[2]`). The client's C++ plugins publish `std_msgs/msg/String` with JSON `{"PlanTime": float, "Mem": float}`. Type mismatch → `Mem` and `PlanTime` silently 0.0 in all runs.

**Fix:**
```python
from std_msgs.msg import String as StringMsg
import json

self.metrics_sub = self.create_subscription(StringMsg, '/planner_metrics', self.metrics_callback, 10)

def metrics_callback(self, msg):
    try:
        data = json.loads(msg.data)
        self.current_mem = float(data["Mem"])
        self.current_plan_time = float(data["PlanTime"])
    except (json.JSONDecodeError, KeyError, ValueError) as e:
        self.get_logger().warn(f"planner_metrics parse error: {e} | raw: {msg.data[:120]}")
```

---

### BUG 7 — Corridors/Mazes maps never spawned — 600s odom timeout too short

**Files:** `master_benchmarker.py`, all run scripts

**Problem:** `ODOM_WAIT_TIMEOUT_SEC` was hardcoded at 600s. Corridors worlds have 1,700+ SDF links; Ignition Gazebo physics initialization is super-linear in link count and takes ~688–770s on the test host. Every Corridors map timed out before the robot spawned.

**Diagnosed by:** Comparing SDF link counts: `grep -c "<link name="` → Random=~200, Rooms=1,000, Mazes=1,277, Corridors=1,737.

**Fix:**
- Made odom timeout configurable: `odom_wait_sec = float(os.getenv('ODOM_WAIT_TIMEOUT_SEC', '600'))`
- Set `ODOM_WAIT_TIMEOUT_SEC=1200` in all scripts that include Corridors/Mazes maps
- Odom timeout now raises `TimeoutError` immediately (instead of proceeding anyway) to trigger the retry logic

---

### BUG 8 — Odom timeout proceeded anyway instead of triggering retry

**File:** `master_benchmarker.py`

**Problem:** When odom never appeared within the timeout, the code logged a warning and continued — producing NaN metrics and no retry.

**Fix:**
```python
else:
    self.get_logger().error(
        "Odometry did not appear within Xs — robot did not spawn. Triggering retry.")
    raise TimeoutError("nav2 active timeout")
```
Only odom timeouts retry (robot didn't spawn). Navigation timeouts (robot spawned but navigation failed) are valid data points and do not retry.

---

### BUG 9 — A\* run crashed with `planner_id="GridBasedAstar"` (plugin does not exist)

**File:** `master_benchmarker.py`

**Problem:** The A\* run called `execute_single_run(..., planner_id="GridBasedAstar")`. The client's Nav2 YAML has only one planner named `"GridBased"` that switches between Dijkstra and A\* via `use_astar`. No `GridBasedAstar` plugin exists — Nav2 raised an immediate error.

**Fix:** Added `toggle_astar_param()` method that calls `/planner_server/set_parameters` at runtime:
```python
# A* run:
self.toggle_astar_param(use_astar=True)
res_a = self.execute_single_run(..., planner_id="GridBased")
self.toggle_astar_param(use_astar=False)  # restore Dijkstra mode

# Dijkstra run (use_astar already False):
res_d = self.execute_single_run(..., planner_id="GridBased")
```

---

### BUG 10 — Stale timeouts in `run_10map.sh` and missing odom timeout in `run_9map.sh`

**Files:** `run_9map.sh`, `run_10map.sh`, `run_benchmark.sh`

**Problem:**
- `run_9map.sh` missing `ODOM_WAIT_TIMEOUT_SEC` → Corridors maps timed out, required mid-run container restart
- `run_10map.sh` had `SINGLE_RUN_TIMEOUT_SEC=300` (too short for 400s Corridors navigation) and `NAV2_ACTIVE_TIMEOUT_SEC=120`
- `run_benchmark.sh` didn't pass `ODOM_WAIT_TIMEOUT_SEC`, `MAX_SPAWN_RETRIES`, or `START_MAP_INDEX` to the container

**Fix:** All three updated. See script table above for current values.

---

### BUG 11 — `run_9map.sh` used as resume base deletes CSV and resets `START_MAP_INDEX=0`

**Operational issue** (not a code bug)

**Problem:** When trying to resume a partial run by passing `-e START_MAP_INDEX=2` to Docker, the `run_9map.sh` script's own `export START_MAP_INDEX=0` and `rm -f dataset/ann_real_world_targets.csv` overwrite the env var and wipe the previously completed rows.

**Fix/Lesson:** Never use `run_9map.sh` for resuming. Use a dedicated resume script (`run_9map_resume.sh`, `run_6map_resume.sh`) which does not clear the CSV and exports the correct `START_MAP_INDEX`. If in doubt, read the script's exports before launching.

---

## Important Notes

- **Corridors maps**: Always need `ODOM_WAIT_TIMEOUT_SEC=1200`. Default 600s is insufficient — confirmed by observing 688–770s spawn times across multiple runs.
- **RRT\* "trivial 1-pose" results**: When the robot reset lands on or near the goal position, RRT\* returns 1 pose with `Success=False`. This is expected behavior — it gets averaged with other runs. Not a bug.
- **`Mem` and `PlanTime` = 0.0**: If these are zero in the CSV, the C++ plugins are either not running or publishing `Float32MultiArray` instead of JSON String. Rebuild the container after ensuring the plugins publish the correct format.
- World name parsing (`parts[0]_parts[1]`) assumes filenames follow `map_XXXX_TYPE_DIFF.sdf`. Will break for any differently named files.
- `TurtleBot-RRT-Star` is included as a regular directory, not a git submodule. It has its own `.git` history from upstream.
- The `time.sleep(20)` post-map cleanup (after `pkill -9`) replaced the original `time.sleep(5)`. Do not reduce without testing — lingering Gazebo processes cause port conflicts on the next map.

---

## How to Run (Docker — recommended)

```bash
# 1. Build the image once (~10 min)
docker build \
  -f navigation/.devcontainer/Dockerfile \
  -t ros2_benchmark:humble \
  navigation/.devcontainer/

# 2. Full 100-map production run
docker run -d \
  --name benchmark_100map \
  --privileged \
  -v "$(pwd)/navigation:/navigation" \
  -v "$(pwd)/Benchmarking_dataset:/Benchmarking_dataset" \
  -v "$(pwd)/run_100map.sh:/run_100map.sh" \
  ros2_benchmark:humble \
  bash /run_100map.sh

# 3. Monitor
docker logs -f benchmark_100map 2>&1 | grep -E "Testing Map|Completed Map|Run result|spawn failure"

# 4. Resume from map N (after interruption)
# First check how many rows are in the CSV (subtract 1 for header) to find N:
wc -l Benchmarking_dataset/dataset/ann_real_world_targets.csv
# Then edit run_9map_resume.sh to set START_MAP_INDEX=N and launch:
docker run -d --name benchmark_resume --privileged \
  -v "$(pwd)/navigation:/navigation" \
  -v "$(pwd)/Benchmarking_dataset:/Benchmarking_dataset" \
  -v "$(pwd)/run_9map_resume.sh:/run_9map_resume.sh" \
  ros2_benchmark:humble bash /run_9map_resume.sh
```
