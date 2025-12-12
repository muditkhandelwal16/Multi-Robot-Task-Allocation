#!/usr/bin/env python3

import copy
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException

from custom_interfaces.srv import PlanPath

from . import hungarian  # our hungarian.py


class PathAllocationNode(Node):
    """
    ROS2 node that:
      - gets robot poses from TF (map -> tb3_i/base_footprint),
      - calls A* planner service to get paths & lengths,
      - feeds those lengths into the Hungarian dynamic allocation,
      - publishes ONE concatenated nav_msgs/Path per robot on /tb3_i/path:
          R  -> T1 -> T2 -> ... (full route in one Path).
    """

    def __init__(self):
        super().__init__("path_allocation")

        # TF buffer + listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Service client to A* planner
        self.plan_client = self.create_client(PlanPath, "a_star/plan_path")
        while not self.plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for a_star/plan_path service...")

        # Robot <-> TF frame mapping
        self.robot_frames = {
            "R1": "tb3_1/base_footprint",
            "R2": "tb3_2/base_footprint",
            "R3": "tb3_3/base_footprint",
            "R4": "tb3_4/base_footprint",
            "R5": "tb3_5/base_footprint",
        }

        # Robot <-> path topic mapping
        self.robot_path_topics = {
            "R1": "/tb3_1/path",
            "R2": "/tb3_2/path",
            "R3": "/tb3_3/path",
            "R4": "/tb3_4/path",
            "R5": "/tb3_5/path",
        }

        self.path_publishers = {
            rid: self.create_publisher(Path, topic, 10)
            for rid, topic in self.robot_path_topics.items()
        }

        # Initial abstract robot/task models
        self.robots = self.create_initial_robots()
        self.tasks = self.create_initial_tasks()

        # Cache: ((start_x_rounded, start_y_rounded), task_id) -> {"length": float, "path": Path}
        self.path_cache = {}

    # ------------------------------------------------------------------
    # Config: robots and tasks
    # ------------------------------------------------------------------

    def create_initial_robots(self):
        """
        Initial robot models.
        Positions will be overwritten with TF-based positions at runtime.
        """
        return [
            {
                "id": "R1",
                "c_robot": np.array([1, 1, 0, 1, 0]),
                "battery": 75,
                "status": 0,
                "position": [0.0, 0.0],
            },
            {
                "id": "R2",
                "c_robot": np.array([0, 0, 1, 1, 1]),
                "battery": 40,
                "status": 0,
                "position": [0.0, 0.0],
            },
            {
                "id": "R3",
                "c_robot": np.array([1, 0, 0, 1, 0]),
                "battery": 82,
                "status": 0,
                "position": [0.0, 0.0],
            },
            {
                "id": "R4",
                "c_robot": np.array([0, 1, 1, 0, 1]),
                "battery": 60,
                "status": 0,
                "position": [0.0, 0.0],
            },
            {
                "id": "R5",
                "c_robot": np.array([1, 1, 1, 1, 0]),
                "battery": 90,
                "status": 0,
                "position": [0.0, 0.0],
            },
        ]

    def create_initial_tasks(self):
        """
        Initial task models (same semantic fields as in hungarian.py).
        Coordinates are assumed to be in the 'map' frame.
        """
        return [
            {
                "id": "T1",
                "r_tasks": np.array([1, 1, 0, 1, 0]),
                "coordinate": [-1.5,4.5],
                "dist": 50,
                "t_ser": 5,
                "priority": 1,
                "status": 0,
                "assigned_robot": None,
            },
            {
                "id": "T2",
                "r_tasks": np.array([0, 0, 1, 1, 0]),
                "coordinate": [-5.5,6],
                "dist": 70,
                "t_ser": 10,
                "priority": 0,
                "status": 0,
                "assigned_robot": None,
            },
            {
                "id": "T3",
                "r_tasks": np.array([1, 0, 0, 1, 0]),
                "coordinate": [1.9,6],
                "dist": 150,
                "t_ser": 12,
                "priority": 2,
                "status": 0,
                "assigned_robot": None,
            },
            {
                "id": "T4",
                "r_tasks": np.array([1, 1, 1, 0, 0]),
                "coordinate": [5.5,8],
                "dist": 300,
                "t_ser": 4,
                "priority": 1,
                "status": 0,
                "assigned_robot": None,
            },
            {
                "id": "T5",
                "r_tasks": np.array([1, 1, 1, 0, 0]),
                "coordinate": [5.5,-4.5],
                "dist": 100,
                "t_ser": 7,
                "priority": 0,
                "status": 0,
                "assigned_robot": None,
            },
            {
                "id": "T6",
                "r_tasks": np.array([0, 1, 1, 1, 0]),
                "coordinate": [-5.5,-3],
                "dist": 180,
                "t_ser": 8,
                "priority": 2,
                "status": 0,
                "assigned_robot": None,
            },
            {
                "id": "T7",
                "r_tasks": np.array([1, 0, 1, 0, 1]),
                "coordinate": [1.5,-7],
                "dist": 120,
                "t_ser": 6,
                "priority": 1,
                "status": 0,
                "assigned_robot": None,
            },
            {
                "id": "T8",
                "r_tasks": np.array([0, 1, 0, 1, 1]),
                "coordinate": [-1.7,-7],
                "dist": 250,
                "t_ser": 15,
                "priority": 2,
                "status": 0,
                "assigned_robot": None,
            },
        ]

    # ------------------------------------------------------------------
    # TF and pose utilities
    # ------------------------------------------------------------------

    def update_robot_positions_from_tf(self):
        """
        Overwrite robots[*]['position'] with TF-measured positions in 'map' frame.
        """
        for r in self.robots:
            frame = self.robot_frames.get(r["id"])
            if frame is None:
                continue
            try:
                tf = self.tf_buffer.lookup_transform("map", frame, rclpy.time.Time())
                r["position"][0] = tf.transform.translation.x
                r["position"][1] = tf.transform.translation.y
                self.get_logger().info(
                    f"Robot {r['id']} pose from TF: "
                    f"({r['position'][0]:.2f}, {r['position'][1]:.2f})"
                )
            except (LookupException, ExtrapolationException) as e:
                self.get_logger().warn(
                    f"TF lookup failed for {frame} ({type(e).__name__}: {e}); "
                    f"keeping default position for {r['id']}."
                )

    def make_pose_stamped(self, x, y, frame_id="map"):
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation.w = 1.0  # no orientation used for A*
        return ps

    # ------------------------------------------------------------------
    # Service call helper to A* planner
    # ------------------------------------------------------------------

    def call_plan_service(self, start_ps: PoseStamped, goal_ps: PoseStamped):
        req = PlanPath.Request()
        req.start = start_ps
        req.goal = goal_ps

        future = self.plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error("PlanPath service call failed (None result).")
            return None

        resp = future.result()
        if not resp.success or not resp.path.poses:
            self.get_logger().warn(f"PlanPath failed: {resp.message}")
            return None

        return {"length": float(resp.length), "path": resp.path}

    # ------------------------------------------------------------------
    # Distance function used by Hungarian (robot, task) -> path length
    # ------------------------------------------------------------------

    def distance_fn(self, robot, task):
        """
        Called by hungarian.simulate_dynamic_reallocation_with_distance_fn.
        Uses robot['position'] and task['coordinate'] to plan via A* and
        returns the path length. Results are cached by (start_xy, task_id).
        """
        start_x = float(robot["position"][0])
        start_y = float(robot["position"][1])
        goal_x = float(task["coordinate"][0])
        goal_y = float(task["coordinate"][1])

        key = ((round(start_x, 2), round(start_y, 2)), task["id"])

        if key in self.path_cache:
            return self.path_cache[key]["length"]

        start_ps = self.make_pose_stamped(start_x, start_y, "map")
        goal_ps = self.make_pose_stamped(goal_x, goal_y, "map")

        path_info = self.call_plan_service(start_ps, goal_ps)
        if path_info is None:
            # No path -> treat as very large distance
            self.get_logger().warn(
                f"No path for {robot['id']} from ({start_x:.2f},{start_y:.2f}) to "
                f"{task['id']}@({goal_x:.2f},{goal_y:.2f}); using q_max."
            )
            return hungarian.q_max

        self.path_cache[key] = path_info
        return path_info["length"]

    # ------------------------------------------------------------------
    # Publish ONE concatenated path per robot schedule
    # ------------------------------------------------------------------

    def publish_paths_for_schedule(self, schedule, tasks_list):
        """
        For each robot, build a SINGLE nav_msgs/Path that goes:

            robot_start -> task1 -> task2 -> ...

        according to the dynamic schedule, and publish it on /tb3_i/path.
        """
        task_by_id = {t["id"]: t for t in tasks_list}

        for rid, task_entries in schedule.items():
            pub = self.path_publishers.get(rid)
            if pub is None:
                self.get_logger().warn(f"No path publisher for robot {rid}")
                continue

            base_robot = next((r for r in self.robots if r["id"] == rid), None)
            if base_robot is None:
                self.get_logger().warn(f"No base robot model found for {rid}")
                continue

            # Sort tasks in execution order, just in case
            robot_schedule = sorted(
                task_entries,
                key=lambda e: e.get("start_time", 0.0),
            )

            if not robot_schedule:
                self.get_logger().info(f"{rid} has empty schedule; skipping path.")
                continue

            # Current position starts at robot's pose from TF
            cur_x = float(base_robot["position"][0])
            cur_y = float(base_robot["position"][1])

            combined_path = Path()
            combined_path.header.frame_id = "map"
            combined_path.header.stamp = self.get_clock().now().to_msg()
            combined_poses = []

            for idx, entry in enumerate(robot_schedule):
                tid = entry["task_id"]
                if tid not in task_by_id:
                    self.get_logger().warn(f"Task {tid} not in task list; skipping.")
                    continue
                t = task_by_id[tid]

                key = ((round(cur_x, 2), round(cur_y, 2)), tid)
                if key not in self.path_cache:
                    start_ps = self.make_pose_stamped(cur_x, cur_y, "map")
                    goal_ps = self.make_pose_stamped(
                        t["coordinate"][0], t["coordinate"][1], "map"
                    )
                    path_info = self.call_plan_service(start_ps, goal_ps)
                    if path_info is None:
                        self.get_logger().warn(
                            f"Could not compute path for {rid} -> {tid}"
                        )
                        continue
                    self.path_cache[key] = path_info

                segment_path: Path = self.path_cache[key]["path"]
                if not segment_path.poses:
                    self.get_logger().warn(
                        f"Segment path for {rid} -> {tid} is empty; skipping."
                    )
                    continue

                # Concatenate segments:
                # - include all poses for the first segment
                # - for subsequent segments, drop the first pose to avoid duplicates
                if not combined_poses:
                    combined_poses.extend(segment_path.poses)
                else:
                    combined_poses.extend(segment_path.poses[1:])

                # Update current position to this task's coordinate for next leg
                cur_x = float(t["coordinate"][0])
                cur_y = float(t["coordinate"][1])

            if not combined_poses:
                self.get_logger().warn(
                    f"No valid segments for {rid}; no path will be published."
                )
                continue

            combined_path.poses = combined_poses
            self.get_logger().info(
                f"Publishing concatenated path for {rid} with "
                f"{len(combined_path.poses)} poses "
                f"(start -> {' -> '.join(e['task_id'] for e in robot_schedule)})"
            )
            pub.publish(combined_path)

    # ------------------------------------------------------------------
    # Main allocation + path generation pipeline
    # ------------------------------------------------------------------

    def run_allocation(self):
        """
        1) Update robot positions from TF.
        2) Run Hungarian dynamic allocation with A* distances.
        3) Print timeline and per-robot schedule.
        4) Publish one concatenated path per robot schedule.
        """
        self.get_logger().info("Updating robot poses from TF...")
        self.update_robot_positions_from_tf()

        # Work on deep copies so self.robots/self.tasks stay as initial models
        robots_copy = copy.deepcopy(self.robots)
        tasks_copy = copy.deepcopy(self.tasks)

        self.get_logger().info(
            "Running Hungarian dynamic reallocation with A* path lengths..."
        )
        result = hungarian.simulate_dynamic_reallocation_with_distance_fn(
            robots_copy, tasks_copy, self.distance_fn
        )

        timeline = result["timeline"]
        schedule = result["schedule"]

        self.get_logger().info("=== Allocation timeline ===")
        for t, event in timeline:
            self.get_logger().info(f"[{t:.1f}] {event}")

        self.get_logger().info("=== Per-robot schedule ===")
        for rid, entries in schedule.items():
            s = ", ".join(
                f"{e['task_id']}({e['start_time']:.1f}->{e['finish_time']:.1f})"
                for e in entries
            )
            self.get_logger().info(f"{rid}: {s}")

        # Publish one path for each robot:
        # robot_start -> first task -> next task -> ...
        self.publish_paths_for_schedule(schedule, tasks_copy)


def main(args=None):
    rclpy.init(args=args)
    node = PathAllocationNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

        # Give TF/map/services a moment to be ready
    start_time = time.time()
    while time.time() - start_time < 4.0:
        executor.spin_once(timeout_sec=0.1)

    try:
        node.run_allocation()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
