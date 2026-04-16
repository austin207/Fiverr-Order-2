# CLAUDE.md — Project Context
## Fiverr Order 2: ROS 2 Nav2 Benchmarking System

---

## Project Overview

This repository contains a complete ROS 2 (Humble) benchmarking system that measures the performance of three pathfinding algorithms — **Dijkstra**, **A\***, and **RRT\*** — across 100 procedurally generated Gazebo Ignition simulation environments. The output is a structured CSV dataset intended for training a machine learning model (ANN) to predict the best planner for a given map.

---

## Repository Structure

```
Fiverr-Order-2/
├── CLAUDE.md                              ← this file
├── Benchmarking_dataset/
│   ├── master_benchmarker.py              ← main automation script
│   └── dataset/
│       └── gazebo_worlds/
│           ├── calibration_manifest.csv  ← 100-map input manifest
│           └── map_XXXX_TYPE_DIFF.{sdf,yaml,png}  ← 96 world files × 3
└── navigation/
    ├── .devcontainer/                     ← Docker dev container (ROS 2 Humble)
    ├── .vscode/
    └── src/
        ├── robot_description/             ← Robot URDF/Xacro + STL meshes
        ├── robot_control/                 ← Python ROS 2 control package
        ├── robot_bringup/                 ← Launch files + Nav2 configs
        │   ├── launch/
        │   │   ├── robot_gazebo_launch.py ← launched by master_benchmarker.py
        │   │   ├── bringup_nav.py
        │   │   └── slam_launch.py
        │   ├── config/
        │   │   ├── nav2_params_Dijkstra.yaml
        │   │   ├── nav2_params_A_star.yaml
        │   │   └── nav2_params_rrt.yaml
        │   └── worlds/
        └── TurtleBot-RRT-Star/            ← C++ Nav2 RRT* global planner plugin
            ├── src/rrtstar_planner.cpp
            └── include/nav2_rrtstar_planner/rrtstar_planner.hpp
```

---

## Components

### 1. `master_benchmarker.py`

The central automation script. It is a ROS 2 Python node (`MasterBenchmarker`) that:

1. Reads `calibration_manifest.csv` (100 rows — world file, spawn/goal coords, obstacle count)
2. For each map:
   - Launches Gazebo + Nav2 headless via `robot_bringup robot_gazebo_launch.py`
   - Runs **Dijkstra** (1 run) using `NavFn` with `use_astar=false`
   - Runs **A\*** (1 run) using `NavFn` with `use_astar=true`
   - Runs **RRT\*** (20 runs, averaged) using the custom `TurtleBot-RRT-Star` plugin
   - Resets robot between runs via `ign service set_pose` + AMCL `/initialpose`
   - Saves results incrementally to `dataset/ann_real_world_targets.csv`
   - Kills the Gazebo process group via `SIGINT` to reclaim RAM

#### Metric Collection

| Metric | Method |
|--------|--------|
| `ExecTime` | Wall-clock time around `followPath()` in Python |
| `Cost` | Sum of Euclidean distances between poses on `/plan` topic |
| `Turns` | Count of angle changes > 0.5 rad between path segments on `/plan` |
| `Mem` | Broadcast from modified C++ planner plugin via `/planner_metrics` topic |
| `PlanTime` | Broadcast from modified C++ planner plugin via `/planner_metrics` topic |
| `BatteryDrain` | Integrated `abs(linear.x) * dt * 0.01` from `/cmd_vel` topic |

#### Output CSV Columns

`world_file, D_Mem, D_Cost, D_PlanTime, D_ExecTime, D_Turns, D_Battery, A_Mem, A_Cost, A_PlanTime, A_ExecTime, A_Turns, A_Battery, RRT_Mem, RRT_Cost, RRT_PlanTime, RRT_ExecTime, RRT_Turns, RRT_Battery`

---

### 2. `navigation/` — ROS 2 Workspace

The ROS 2 workspace the robot runs in. Must be built with `colcon build` on a Linux/Docker machine with ROS 2 Humble installed.

**Key packages:**
- `robot_description` — Xacro URDF of the "Kina" mobile robot with LIDAR, diff drive, and ros2_control
- `robot_bringup` — launch files, Nav2 parameter configs (one per algorithm), Gazebo worlds, EKF config
- `robot_control` — placeholder Python package for velocity control
- `TurtleBot-RRT-Star` — third-party C++ Nav2 global planner plugin implementing RRT\*

**The `robot_gazebo_launch.py` accepts these arguments** (called by `master_benchmarker.py`):
- `world` — path to `.sdf` file
- `spawn_x`, `spawn_y` — robot start position
- `use_rviz`, `rviz`, `headless` — GUI controls (all `false`/`true` for headless)
- `gz_args` — passed to Ignition Gazebo (`-r -s` = run headless server-only)

---

### 3. `calibration_manifest.csv`

100-row input file. Each row defines one benchmark run with pre-measured reference values.

**Columns:** `world_file, category, difficulty, obstacles, a_mem, a_time, a_turns, d_mem, d_time, d_turns, rrt_mem, rrt_time, rrt_turns, spawn_x, spawn_y, spawn_z, goal_x, goal_y, wall_count`

**Map distribution (100 total):**
- Random: 3 easy / 6 moderate / 16 hard
- Rooms: 3 easy / 6 moderate / 15 hard
- Corridors: 3 easy / 6 moderate / 16 hard
- Mazes: 3 easy / 6 moderate / 17 hard

---

## Bug Fixes Applied (this session)

### BUG 1 — Node never spun during navigation (callbacks silently dropped)

**File:** `master_benchmarker.py` line 193–195

**Problem:** The blocking wait loop `while not isTaskComplete(): time.sleep(0.1)` never called `rclpy.spin_once()`, so the `/plan` and `/planner_metrics` callbacks never fired during navigation. All metrics were read as their reset values (0.0).

**Fix:**
```python
while not self.navigator.isTaskComplete():
    rclpy.spin_once(self, timeout_sec=0.0)  # process pending callbacks
    time.sleep(0.1)
```

---

### BUG 2 — Wrong localizer name in `waitUntilNav2Active`

**File:** `master_benchmarker.py` line 236

**Problem:** `localizer='bt_navigator'` is the navigator action server, not the localizer. This caused Nav2 to wait on the wrong lifecycle node, potentially starting navigation before AMCL was ready.

**Fix:**
```python
self.navigator.waitUntilNav2Active(localizer='amcl')
```

---

### BUG 3 — Battery drain metric missing entirely

**File:** `master_benchmarker.py` — multiple locations

**Problem:** No battery consumption metric existed. The dataset lacked energy efficiency data for the ML model.

**Fix:** Added `/cmd_vel` subscriber with time-integrated linear velocity:
```python
def cmd_vel_callback(self, msg):
    now = time.time()
    if self.last_cmd_vel_time is not None:
        dt = now - self.last_cmd_vel_time
        linear_velocity = abs(msg.linear.x)
        self.current_battery_drain += linear_velocity * dt * self.drain_constant
    self.last_cmd_vel_time = now
```
- `drain_constant = 0.01` (tunable)
- State reset at the start of every `execute_single_run()`
- Returned as `"BatteryDrain"` in the result dict
- Written to CSV as `D_Battery`, `A_Battery`, `RRT_Battery`

---

## Important Notes

- The `time.sleep(60)` on line 235 has a stale comment saying "15 seconds" — the actual wait is 60 seconds. Do not reduce this without testing on target hardware.
- World name parsing (`parts[0]_parts[1]`) on line 252 assumes filenames follow the pattern `map_XXXX_TYPE_DIFF.sdf`. This will break for any differently named file.
- `TurtleBot-RRT-Star` has its own `.git` history (cloned from upstream). It is included as a regular directory, not a git submodule.
- The `/planner_metrics` topic requires **modified C++ Nav2 planner plugins** that broadcast telemetry. The standard Nav2 NavFn plugin does NOT publish this topic.
- The `.devcontainer/Dockerfile` is the recommended way to build and run this on a non-Linux machine.

---

## How to Run

```bash
# 1. Build the navigation workspace (on Linux / inside dev container)
cd navigation
colcon build --symlink-install
source install/setup.bash

# 2. Run the benchmarker (from the Benchmarking_dataset directory)
cd Benchmarking_dataset
python3 master_benchmarker.py
```

Output will be written to `dataset/ann_real_world_targets.csv`.
