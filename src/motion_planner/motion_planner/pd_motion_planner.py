#!/usr/bin/env python3
import numpy as np
if not hasattr(np, "float"):
    # Compatibility for older code that expects np.float to exist
    np.float = float

import math

import rclpy
from rclpy.node import Node
from typing import Optional

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener
from tf_transformations import (
    quaternion_matrix,
    quaternion_from_matrix,
    translation_from_matrix,
    inverse_matrix,
    concatenate_matrices,
)


class PDMotionPlanner(Node):
    """
    Simple PD-based path follower for a single robot.

    This node is meant to be run once per robot, with topics and frames
    configured via parameters, e.g.:

      ros2 run <your_package> pd_motion_planner --ros-args \
        -p odom_frame:=tb3_1/odom \
        -p base_frame:=tb3_1/base_footprint \
        -p path_topic:=/tb3_1/path \
        -p cmd_vel_topic:=/tb3_1/cmd_vel
    """

    def __init__(self) -> None:
        super().__init__("pd_motion_planner_node")

        # TF buffer and listener
        self.tf_buffer: Buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Controller parameters ---
        self.declare_parameter("kp", 2.0)
        self.declare_parameter("kd", 0.1)
        self.declare_parameter("step_size", 0.2)
        self.declare_parameter("max_linear_velocity", 0.25)
        self.declare_parameter("max_angular_velocity", 1.0)

        self.kp: float = self.get_parameter("kp").value
        self.kd: float = self.get_parameter("kd").value
        self.step_size: float = self.get_parameter("step_size").value
        self.max_linear_velocity: float = self.get_parameter("max_linear_velocity").value
        self.max_angular_velocity: float = self.get_parameter("max_angular_velocity").value

        # --- Frames (per-robot configurable) ---
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.odom_frame: str = self.get_parameter("odom_frame").value
        self.base_frame: str = self.get_parameter("base_frame").value

        # --- Topics (per-robot configurable) ---
        # Defaults keep your original single-robot behaviour.
        self.declare_parameter("path_topic", "/a_star/path")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("next_pose_topic", "/pd/next_pose")

        path_topic: str = self.get_parameter("path_topic").value
        cmd_vel_topic: str = self.get_parameter("cmd_vel_topic").value
        next_pose_topic: str = self.get_parameter("next_pose_topic").value

        # Subscribers and publishers
        self.path_sub = self.create_subscription(Path, path_topic, self.path_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.next_pose_pub = self.create_publisher(PoseStamped, next_pose_topic, 10)

        # Control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        self.global_plan: Optional[Path] = None

        self.prev_angular_error: float = 0.0
        self.prev_linear_error: float = 0.0
        self.last_cycle_time = self.get_clock().now()

        self.get_logger().info(
            f"PD Motion Planner started with:\n"
            f"  odom_frame: {self.odom_frame}\n"
            f"  base_frame: {self.base_frame}\n"
            f"  path_topic: {path_topic}\n"
            f"  cmd_vel_topic: {cmd_vel_topic}\n"
            f"  next_pose_topic: {next_pose_topic}"
        )

    # ------------------------------------------------------------------
    # Callbacks and control loop
    # ------------------------------------------------------------------
    def path_callback(self, path: Path) -> None:
        """Store the latest global plan."""
        self.global_plan = path
        if path.poses:
            self.get_logger().info(
                f"Received path with {len(path.poses)} poses in frame '{path.header.frame_id}'"
            )

    def control_loop(self) -> None:
        """Main control loop: follow the current global path with a PD controller."""
        if not self.global_plan or not self.global_plan.poses:
            return

        # Get the robot's current pose in the odom frame
        try:
            robot_pose_transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),
            )
        except Exception as ex:
            self.get_logger().warn(f"Could not transform {self.odom_frame} -> {self.base_frame}: {ex}")
            return

        # Transform plan to robot's frame
        if not self.transform_plan(robot_pose_transform.header.frame_id):
            self.get_logger().error("Unable to transform plan into robot's frame")
            return

        robot_pose = PoseStamped()
        robot_pose.header.frame_id = robot_pose_transform.header.frame_id
        robot_pose.pose.position.x = robot_pose_transform.transform.translation.x
        robot_pose.pose.position.y = robot_pose_transform.transform.translation.y
        robot_pose.pose.orientation = robot_pose_transform.transform.rotation

        next_pose: PoseStamped = self.get_next_pose(robot_pose)
        dx = next_pose.pose.position.x - robot_pose.pose.position.x
        dy = next_pose.pose.position.y - robot_pose.pose.position.y
        distance = math.sqrt(dx * dx + dy * dy)

        # If we're close enough, consider the goal reached
        if distance <= 0.1:
            self.get_logger().info("Goal reached, clearing current plan.")
            self.global_plan.poses.clear()
            # Stop the robot
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)
            return

        # For debugging / visualization
        self.next_pose_pub.publish(next_pose)

        # ------------------------------------------------------------------
        # PD control in the robot's local frame
        # ------------------------------------------------------------------
        # Transform robot pose and next pose into matrices
        robot_tf = quaternion_matrix(
            [
                robot_pose.pose.orientation.x,
                robot_pose.pose.orientation.y,
                robot_pose.pose.orientation.z,
                robot_pose.pose.orientation.w,
            ]
        )
        robot_tf[0][3] = robot_pose.pose.position.x
        robot_tf[1][3] = robot_pose.pose.position.y

        next_pose_tf = quaternion_matrix(
            [
                next_pose.pose.orientation.x,
                next_pose.pose.orientation.y,
                next_pose.pose.orientation.z,
                next_pose.pose.orientation.w,
            ]
        )
        next_pose_tf[0][3] = next_pose.pose.position.x
        next_pose_tf[1][3] = next_pose.pose.position.y

        # Compute relative transform: next_pose_robot_tf = robot_tf^{-1} * next_pose_tf
        next_pose_robot_tf = concatenate_matrices(inverse_matrix(robot_tf), next_pose_tf)

        # Extract relative position (x = forward, y = lateral)
        angular_error = next_pose_robot_tf[1, 3]
        linear_error = next_pose_robot_tf[0, 3]

        now = self.get_clock().now()
        dt = (now - self.last_cycle_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 1e-6

        angular_error_derivative = (angular_error - self.prev_angular_error) / dt
        linear_error_derivative = (linear_error - self.prev_linear_error) / dt

        # PD control law with simple saturation
        cmd_vel = Twist()
        raw_angular = self.kp * angular_error + self.kd * angular_error_derivative
        raw_linear = self.kp * linear_error + self.kd * linear_error_derivative

        cmd_vel.angular.z = max(
            -self.max_angular_velocity, min(raw_angular, self.max_angular_velocity)
        )
        cmd_vel.linear.x = max(
            -self.max_linear_velocity, min(raw_linear, self.max_linear_velocity)
        )

        self.cmd_pub.publish(cmd_vel)

        # Save state for next iteration
        self.prev_angular_error = angular_error
        self.prev_linear_error = linear_error
        self.last_cycle_time = now

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def get_next_pose(self, robot_pose: PoseStamped) -> PoseStamped:
        """
        Returns the next pose along the global plan that is at least step_size
        away from the current robot pose. If no such pose exists, returns the
        final pose of the plan.
        """
        assert self.global_plan is not None

        next_pose = self.global_plan.poses[-1]
        for pose in reversed(self.global_plan.poses):
            dx = pose.pose.position.x - robot_pose.pose.position.x
            dy = pose.pose.position.y - robot_pose.pose.position.y
            distance = math.sqrt(dx * dx + dy * dy)
            if distance > self.step_size:
                next_pose = pose
            else:
                break
        return next_pose

    def transform_plan(self, target_frame: str) -> bool:
        """
        Transforms all poses in the global plan into the given target frame.
        """
        assert self.global_plan is not None

        if self.global_plan.header.frame_id == target_frame:
            return True

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                self.global_plan.header.frame_id,
                rclpy.time.Time(),
            )
        except Exception as ex:
            self.get_logger().error(
                f"Couldn't transform plan from frame '{self.global_plan.header.frame_id}' "
                f"to '{target_frame}': {ex}"
            )
            return False

        # Build transform matrix
        transform_matrix = quaternion_matrix(
            [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ]
        )
        transform_matrix[0][3] = transform.transform.translation.x
        transform_matrix[1][3] = transform.transform.translation.y

        # Apply transform to each pose in the plan
        for pose in self.global_plan.poses:
            pose_matrix = quaternion_matrix(
                [
                    pose.pose.orientation.x,
                    pose.pose.orientation.y,
                    pose.pose.orientation.z,
                    pose.pose.orientation.w,
                ]
            )
            pose_matrix[0][3] = pose.pose.position.x
            pose_matrix[1][3] = pose.pose.position.y

            transformed_pose = concatenate_matrices(pose_matrix, transform_matrix)

            (
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            ) = translation_from_matrix(transformed_pose)

            (
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ) = quaternion_from_matrix(transformed_pose)

        self.global_plan.header.frame_id = target_frame
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PDMotionPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
