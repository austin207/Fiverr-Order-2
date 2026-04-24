"""
=========================================================================================
MASTER BENCHMARKER SCRIPT
=========================================================================================
Description:
This script automates the benchmarking of ROS 2 (Humble) Nav2 pathfinding algorithms 
(Dijkstra, A*, and RRT*) across multiple Gazebo Ignition simulation worlds. It is 
designed to run completely headless to preserve RAM and generate a comprehensive 
performance dataset for machine learning.

Workflow per Map:
1. Reads map configuration and safe coordinates from 'calibration_manifest.csv'.
2. Launches the Gazebo world and Nav2 stack headless (no RViz, no GUI because of crash issues).
3. Executes Dijkstra (1 run), logs metrics, and teleports the robot back to start.
4. Dynamically switches parameters to A* (1 run), logs metrics, and teleports back.
5. Executes RRT* (20 runs to account for randomness), averages the successful runs,
   and teleports back.
6. Appends the map's algorithmic metrics to 'ann_real_world_targets.csv'.
7. Performs a hard process-group kill to completely clear system RAM before the next map.

Metric Collection Methods:
- ExecTime: Wall-clock time measured via Python during 'followPath'.
- Cost & Turns: Calculated geometrically by subscribing to the '/plan' topic.
- Mem & PlanTime: Captured from modified C++ planner plugins that broadcast their 
  internal compute telemetry to a custom '/planner_metrics' topic.
=========================================================================================
"""


import copy
import rclpy
import rclpy.time
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray 
import pandas as pd
import numpy as np
import subprocess
import time
import os
from lifecycle_msgs.srv import GetState
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
import signal

class MasterBenchmarker(Node):
    def __init__(self):
        super().__init__('master_benchmarker')
        self.navigator = BasicNavigator()
        self.rrt_iterations = int(os.getenv('RRT_ITERATIONS', '20'))
        self.single_run_timeout_sec = float(os.getenv('SINGLE_RUN_TIMEOUT_SEC', '0'))
        self.map_timeout_sec = float(os.getenv('MAP_TIMEOUT_SEC', '0'))
        self.nav2_active_timeout_sec = float(os.getenv('NAV2_ACTIVE_TIMEOUT_SEC', '0'))
        
        # Subscriptions to capture C++ metrics and Path Cost
        self.path_sub = self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.metrics_sub = self.create_subscription(Float32MultiArray, '/planner_metrics', self.metrics_callback, 10)
        
        # State variables for the current run
        self.current_path_cost = 0.0
        self.current_mem = 0.0
        self.current_plan_time = 0.0
        self.current_turns = 0
        self.current_plan_success = True
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.plan_received = False

        # Battery drain state
        self.current_battery_drain = 0.0
        self.last_cmd_vel_time = None
        self.drain_constant = 0.01
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # File paths
        self.manifest_path = "dataset/gazebo_worlds/calibration_manifest.csv"
        self.results_path = "dataset/ann_real_world_targets.csv"
        self.worlds_dir = "dataset/gazebo_worlds/"

    def path_callback(self, msg):
        """Calculates physical distance (Cost) and counts Turns from the generated path."""
        cost = 0.0
        turns = 0
        if len(msg.poses) > 1:
            # Calculate Distance
            for i in range(1, len(msg.poses)):
                p1 = msg.poses[i-1].pose.position
                p2 = msg.poses[i].pose.position
                cost += math.hypot(p2.x - p1.x, p2.y - p1.y)
            
            # Estimate Turns (checking angle changes between vectors)
            for i in range(2, len(msg.poses)):
                p0 = msg.poses[i-2].pose.position
                p1 = msg.poses[i-1].pose.position
                p2 = msg.poses[i].pose.position
                
                ang1 = math.atan2(p1.y - p0.y, p1.x - p0.x)
                ang2 = math.atan2(p2.y - p1.y, p2.x - p1.x)
                diff = abs(math.atan2(math.sin(ang2 - ang1), math.cos(ang2 - ang1)))
                
                if diff > 0.5:  # Roughly 28 degrees threshold for a "turn"
                    turns += 1

        self.current_path_cost = cost
        self.current_turns = turns
        self.plan_received = True

    def metrics_callback(self, msg):
        """Catches Mem, PlanTime, and Success flag from custom C++ planner plugins."""
        if msg and len(msg.data) >= 2:
            self.current_mem = float(msg.data[0])
            self.current_plan_time = float(msg.data[1])
            self.current_plan_success = bool(msg.data[2]) if len(msg.data) >= 3 else True

    def cmd_vel_callback(self, msg):
        """Integrates linear velocity over time to estimate battery drain per run."""
        now = time.time()
        if self.last_cmd_vel_time is not None:
            dt = now - self.last_cmd_vel_time
            linear_velocity = abs(msg.linear.x)
            self.current_battery_drain += linear_velocity * dt * self.drain_constant
        self.last_cmd_vel_time = now

    def _spin_for(self, seconds):
        """Spin this node for exactly `seconds` wall-clock time, processing callbacks."""
        t_end = time.time() + seconds
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def reset_robot_to_start(self, spawn_x, spawn_y, world_name):
        """Teleport robot physically in Gazebo Harmonic/Fortress AND reset AMCL."""
        self.get_logger().info(f"Teleporting robot back to {spawn_x}, {spawn_y}...")

        # 1. Teleport Gazebo Model FIRST so the diff-drive odometry resets to the
        #    new position before AMCL is told where the robot is in map frame.
        #    Publishing /initialpose BEFORE teleport causes AMCL to compute
        #    map→odom using the pre-teleport (goal-position) odom, leaving the
        #    robot TF at the goal. With recovery_alpha=0.0 AMCL never self-corrects,
        #    so MPPI prunes the entire global path out-of-costmap → "0 poses" error.
        ign_cmd = [
            "ign", "service", "-s", f"/world/{world_name}/set_pose",
            "--reqtype", "ignition.msgs.Pose",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "2000",
            "--req", f'name: "robot", position: {{x: {spawn_x}, y: {spawn_y}, z: 0.1}}, orientation: {{w: 1.0, x: 0.0, y: 0.0, z: 0.0}}'
        ]
        result = subprocess.run(ign_cmd, capture_output=True, text=True)
        if result.returncode != 0:
            self.get_logger().warn(f"ign teleport failed: {result.stderr.strip()}")

        # 2. Let Gazebo physics and diff-drive odometry settle at the new position.
        #    Must be a real wall-clock wait — spin_once loops drain instantly when
        #    the callback queue is full, so count-based loops don't guarantee time.
        self._spin_for(1.0)

        # 3. Now reset AMCL — it reads the current (post-teleport) odom frame
        #    and correctly localizes the robot in map frame.
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = rclpy.time.Time().to_msg()  # zero = use latest TF
        msg.pose.pose.position.x = float(spawn_x)
        msg.pose.pose.position.y = float(spawn_y)
        msg.pose.pose.orientation.w = 1.0
        self.initialpose_pub.publish(msg)

        # 4. AMCL must receive /initialpose, process it, ingest multiple laser scans
        #    (10 Hz), and publish the updated map→odom TF before MPPI is called.
        #    3 seconds = 30 laser scans — sufficient for AMCL to converge.
        self._spin_for(3.0)

        # Clear costmaps so stale obstacle cells from the goal don't block the start.
        self.navigator.clearAllCostmaps()

        # Let the static layer repopulate the global costmap before getPath.
        self._spin_for(1.0)
    def execute_single_run(self, spawn_x, spawn_y, goal_pose, planner_id):
        """Executes a single navigation attempt and returns the 5 metrics."""
        # Reset state BEFORE sending goal to avoid stale callback data
        self.current_path_cost = 0.0
        self.current_mem = 0.0
        self.current_plan_time = 0.0
        self.current_turns = 0
        self.plan_received = False
        self.current_plan_success = True
        self.current_battery_drain = 0.0
        self.last_cmd_vel_time = None

        # Drain any pending callbacks left over from the previous run so stale
        # action-server state doesn't corrupt the next goal (fixes 0-poses bug).
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.0)

        # 1. Create a precise Start Pose
        start_pose = PoseStamped()
        start_pose.header.frame_id = 'map'
        start_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        start_pose.pose.position.x = float(spawn_x)
        start_pose.pose.position.y = float(spawn_y)
        start_pose.pose.orientation.w = 1.0

        # 2. Ask for the Path using specific Planner (ISOLATED PLANNING)
        self.get_logger().info(f"Computing path with {planner_id}...")
        path = self.navigator.getPath(start_pose, goal_pose, planner_id=planner_id, use_start=True)
        
        if not path:
            self.get_logger().warn(f"GETPATH RETURNED NONE for planner {planner_id}")
            return {
                "Mem": self.current_mem,
                "Cost": self.current_path_cost,
                "PlanTime": self.current_plan_time,
                "ExecTime": 0.0,
                "Turns": self.current_turns,
                "BatteryDrain": self.current_battery_drain,
                "Success": False,
                "PathFound": False
            }

        self.get_logger().info(f"GETPATH SUCCESS for planner {planner_id}, poses={len(path.poses)}")

        # 3. Tell the robot to drive along that exact path (ISOLATED EXECUTION)
        # Deep-copy avoids the action-result-future GC bug where the C++ backing
        # buffer can be freed between getPath() and followPath(), sending 0 poses.
        self.get_logger().info("Path found! Driving to goal...")
        motion_start = time.time()
        self.navigator.followPath(copy.deepcopy(path))
        
        while not self.navigator.isTaskComplete():
            if self.single_run_timeout_sec > 0 and (time.time() - motion_start) > self.single_run_timeout_sec:
                self.get_logger().warn(
                    f"{planner_id} execution timeout after {self.single_run_timeout_sec:.1f}s. Cancelling task.")
                self.navigator.cancelTask()
                break
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.1)

        exec_time = time.time() - motion_start
        result = self.navigator.getResult()
        self.get_logger().info(f"TaskResult for {planner_id}: {result}")

        if result == TaskResult.SUCCEEDED:
            return {
                "Mem": self.current_mem,
                "Cost": self.current_path_cost,
                "PlanTime": self.current_plan_time,
                "ExecTime": exec_time,
                "Turns": self.current_turns,
                "BatteryDrain": self.current_battery_drain,
                "Success": True,
                "PathFound": True
            }
        else:
            self.get_logger().warn(
                f"{planner_id} followPath did not succeed. TaskResult={result}")
            return {
                "Mem": self.current_mem,
                "Cost": self.current_path_cost,
                "PlanTime": self.current_plan_time,
                "ExecTime": exec_time,
                "Turns": self.current_turns,
                "BatteryDrain": self.current_battery_drain,
                "Success": False,
                "PathFound": True
            }

    def _node_is_active(self, node_name, timeout_sec=5.0):
        """Return True if the given lifecycle node is in 'active' state."""
        client = self.create_client(GetState, f'{node_name}/get_state')
        deadline = time.time() + timeout_sec
        while not client.service_is_ready():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() > deadline:
                return False
        future = client.call_async(GetState.Request())
        deadline2 = time.time() + timeout_sec
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() > deadline2:
                return False
        if future.result() and future.result().current_state.label == 'active':
            return True
        return False

    def wait_nav2_active_with_timeout(self):
        timeout = self.nav2_active_timeout_sec if self.nav2_active_timeout_sec > 0 else 120.0
        deadline = time.time() + timeout
        nodes_needed = ['map_server', 'amcl', 'bt_navigator']
        active = set()

        self.get_logger().info(f"Waiting for Nav2 nodes: {nodes_needed}")
        while active < set(nodes_needed):
            if time.time() > deadline:
                self.get_logger().warn(
                    f"Nav2 activation timeout after {timeout:.1f}s. Active: {active}")
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
            for n in nodes_needed:
                if n not in active and self._node_is_active(n, timeout_sec=2.0):
                    active.add(n)
                    self.get_logger().info(f"{n} is active")

        self.get_logger().info("All Nav2 nodes active")
        return True

    def run_benchmark(self):
        df = pd.read_csv(self.manifest_path)
        final_results = []

        for index, row in df.iterrows():
            world_file = os.path.join(self.worlds_dir, row['world_file'])
            spawn_x = float(row['spawn_x'])
            spawn_y = float(row['spawn_y'])
            goal_x = float(row['goal_x'])
            goal_y = float(row['goal_y'])

            self.get_logger().info(f"\n{'='*50}\nTesting Map {index+1}/{len(df)}: {row['world_file']}\n"
                                   f"Manifest spawn=({spawn_x:.2f}, {spawn_y:.2f}) goal=({goal_x:.2f}, {goal_y:.2f})\n"
                                   f"{'='*50}")

            row_data = {
                "world_file": row['world_file'],
                "D_Mem": np.nan, "D_Cost": np.nan, "D_PlanTime": np.nan, "D_ExecTime": np.nan, "D_Turns": np.nan, "D_Battery": np.nan,
                "A_Mem": np.nan, "A_Cost": np.nan, "A_PlanTime": np.nan, "A_ExecTime": np.nan, "A_Turns": np.nan, "A_Battery": np.nan,
                "RRT_Mem": np.nan, "RRT_Cost": np.nan, "RRT_PlanTime": np.nan, "RRT_ExecTime": np.nan, "RRT_Turns": np.nan, "RRT_Battery": 0.0
            }

            process = None
            map_start = time.time()
            try:
                # 1. Launch Gazebo & Nav2
                launch_cmd = [
                    "ros2", "launch", "robot_bringup", "robot_gazebo_launch.py",
                    f"world:={world_file}",
                    f"spawn_x:={spawn_x}", f"spawn_y:={spawn_y}",
                    "use_rviz:=false",
                    "rviz:=false",
                    "headless:=true",
                    "gz_args:=-r -s --headless-rendering"
                ]
                process = subprocess.Popen(launch_cmd, preexec_fn=os.setsid)

                self.get_logger().info("Waiting for simulation to stabilize...")
                time.sleep(60)

                if not self.wait_nav2_active_with_timeout():
                    raise TimeoutError("nav2 active timeout")

                # 2. Setup Goal Pose
                def make_goal_pose():
                    gp = PoseStamped()
                    gp.header.frame_id = 'map'
                    gp.header.stamp = self.navigator.get_clock().now().to_msg()
                    gp.pose.position.x = goal_x
                    gp.pose.position.y = goal_y
                    gp.pose.orientation.w = 1.0
                    return gp

                # ---> Extract the base map name (e.g., 'map_19639') to use as the Gazebo world name
                parts = row['world_file'].split('_')
                world_name = f"{parts[0]}_{parts[1]}"

                # ==========================================
                # RUN 1: DIJKSTRA
                # ==========================================
                self.get_logger().info("--- Running Dijkstra ---")
                res_d = self.execute_single_run(spawn_x, spawn_y, make_goal_pose(), planner_id="GridBased")
                self.get_logger().info(f"Run result for GridBased: {res_d}")
                row_data.update({"D_Mem": res_d["Mem"], "D_Cost": res_d["Cost"], "D_PlanTime": res_d["PlanTime"], "D_ExecTime": res_d["ExecTime"], "D_Turns": res_d["Turns"], "D_Battery": res_d["BatteryDrain"]})
                self.reset_robot_to_start(spawn_x, spawn_y, world_name)

                if self.map_timeout_sec > 0 and (time.time() - map_start) > self.map_timeout_sec:
                    self.get_logger().warn(f"Map timeout ({self.map_timeout_sec:.1f}s) reached after Dijkstra. Skipping remaining planners.")
                    raise TimeoutError("map timeout")

                # ==========================================
                # RUN 2: A* (A-STAR)
                # ==========================================
                self.get_logger().info("--- Running A* ---")
                res_a = self.execute_single_run(spawn_x, spawn_y, make_goal_pose(), planner_id="GridBasedAstar")
                self.get_logger().info(f"Run result for GridBasedAstar: {res_a}")
                row_data.update({"A_Mem": res_a["Mem"], "A_Cost": res_a["Cost"], "A_PlanTime": res_a["PlanTime"], "A_ExecTime": res_a["ExecTime"], "A_Turns": res_a["Turns"], "A_Battery": res_a["BatteryDrain"]})
                self.reset_robot_to_start(spawn_x, spawn_y, world_name)

                if self.map_timeout_sec > 0 and (time.time() - map_start) > self.map_timeout_sec:
                    self.get_logger().warn(f"Map timeout ({self.map_timeout_sec:.1f}s) reached after A*. Skipping RRT*.")
                    raise TimeoutError("map timeout")

                # ==========================================
                # RUN 3: RRT* (N Iterations)
                # ==========================================
                self.get_logger().info(f"--- Running RRT* ({self.rrt_iterations} Iterations) ---")
                rrt_metrics = {"Mem": [], "Cost": [], "PlanTime": [], "ExecTime": [], "Turns": [], "BatteryDrain": []}
                rrt_battery_samples = []

                for i in range(self.rrt_iterations):
                    if self.map_timeout_sec > 0 and (time.time() - map_start) > self.map_timeout_sec:
                        self.get_logger().warn(f"Map timeout ({self.map_timeout_sec:.1f}s) reached during RRT* at iteration {i+1}.")
                        break
                    self.get_logger().info(f"  > RRT Iteration {i+1}/{self.rrt_iterations}...")
                    self.reset_robot_to_start(spawn_x, spawn_y, world_name)
                    res_rrt = self.execute_single_run(spawn_x, spawn_y, make_goal_pose(), planner_id="RRTStar")
                    self.get_logger().info(f"Run result for RRTStar: {res_rrt}")
                    rrt_battery_samples.append(res_rrt["BatteryDrain"])
                    if res_rrt.get("PathFound", False):
                        for key in rrt_metrics.keys():
                            rrt_metrics[key].append(res_rrt[key])

                if len(rrt_metrics["Mem"]) > 0:
                    row_data.update({
                        "RRT_Mem": np.mean(rrt_metrics["Mem"]),
                        "RRT_Cost": np.mean(rrt_metrics["Cost"]),
                        "RRT_PlanTime": np.mean(rrt_metrics["PlanTime"]),
                        "RRT_ExecTime": np.mean(rrt_metrics["ExecTime"]),
                        "RRT_Turns": np.mean(rrt_metrics["Turns"]),
                        "RRT_Battery": np.mean(rrt_metrics["BatteryDrain"])
                    })
                else:
                    self.get_logger().warn("RRT* produced no valid path plans in this map.")
                    row_data.update({
                        "RRT_Mem": np.nan,
                        "RRT_Cost": np.nan,
                        "RRT_PlanTime": np.nan,
                        "RRT_ExecTime": np.nan,
                        "RRT_Turns": np.nan,
                        "RRT_Battery": float(np.mean(rrt_battery_samples)) if len(rrt_battery_samples) > 0 else 0.0
                    })

            except TimeoutError:
                pass
            except Exception as err:
                self.get_logger().error(f"Map {row['world_file']} failed with exception: {err}")
            finally:
                # Save one row per map, even if partially complete
                final_results.append(row_data)
                pd.DataFrame(final_results).to_csv(self.results_path, index=False, na_rep='NaN')

                # Cleanup Gazebo before next map
                if process is not None:
                    self.get_logger().info("Shutting down Gazebo gracefully...")
                    try:
                        os.killpg(os.getpgid(process.pid), signal.SIGINT)
                        process.wait(timeout=30)
                    except Exception:
                        try:
                            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                        except Exception:
                            pass
                    time.sleep(5)

                self.get_logger().info(f"Completed Map {index+1}/{len(df)}")

        self.get_logger().info(f"\nData Collection Complete! Saved to {self.results_path}")

def main(args=None):
    rclpy.init(args=args)
    benchmarker = MasterBenchmarker()
    try:
        benchmarker.run_benchmark()
    except KeyboardInterrupt:
        pass
    finally:
        benchmarker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
