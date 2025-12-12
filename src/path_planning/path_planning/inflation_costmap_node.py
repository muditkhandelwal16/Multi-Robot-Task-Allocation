#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    HistoryPolicy,
)
from nav_msgs.msg import OccupancyGrid


class InflationCostmapNode(Node):
    def __init__(self):
        super().__init__("inflation_costmap")

        # ---- Parameters ----
        self.declare_parameter("inflation_radius", 0.55)
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("lethal_threshold", 100)

        self.inflation_radius = float(self.get_parameter("inflation_radius").value)
        self.resolution = float(self.get_parameter("resolution").value)
        self.lethal_threshold = int(self.get_parameter("lethal_threshold").value)

        self.inflation_cells = int(math.ceil(self.inflation_radius / self.resolution))
        self.get_logger().info(
            f"Inflating map with radius {self.inflation_radius} m "
            f"({self.inflation_cells} cells)"
        )

        # ---- QoS: match nav2_map_server for /map (transient_local) ----
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        # Subscribe to /map with matching QoS so we actually receive the latched map
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            map_qos,
        )

        # Publisher for /inflated_costmap
        # Normal reliable/volatile is fine; we republish periodically.
        pub_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.costmap_pub = self.create_publisher(
            OccupancyGrid,
            "/inflated_costmap",
            pub_qos,
        )

        # Store inflated map; republish at 1 Hz so late subscribers see it
        self._inflated = None
        self.timer = self.create_timer(1.0, self.timer_cb)

    # ---- Callbacks ----

    def map_callback(self, msg: OccupancyGrid):
        self.get_logger().info(
            f"Got /map: {msg.info.width} x {msg.info.height}, "
            f"res={msg.info.resolution}"
        )
        self._inflated = self.inflate_map(msg)
        self.costmap_pub.publish(self._inflated)
        self.get_logger().info("Published inflated costmap (and will keep republishing)")

    def timer_cb(self):
        # Periodic republish so rviz2 / ros2 topic echo can always see something
        if self._inflated is not None:
            self.costmap_pub.publish(self._inflated)

    # ---- Core inflation logic ----

    def inflate_map(self, msg: OccupancyGrid) -> OccupancyGrid:
        width = msg.info.width
        height = msg.info.height

        # int8 matches OccupancyGrid's data type
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        obstacles = data >= self.lethal_threshold
        inflated_mask = np.zeros_like(obstacles, dtype=bool)
        r = self.inflation_cells

        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r * r:
                    continue

                y_start = max(0, dy)
                y_end = height + min(0, dy)
                x_start = max(0, dx)
                x_end = width + min(0, dx)
                if y_start >= y_end or x_start >= x_end:
                    continue

                shifted = np.zeros_like(obstacles)
                shifted[y_start:y_end, x_start:x_end] = obstacles[
                    y_start - dy : y_end - dy,
                    x_start - dx : x_end - dx,
                ]
                inflated_mask |= shifted

        inflated_data = data.copy()
        inflated_data[inflated_mask & (data != -1)] = 100

        out = OccupancyGrid()
        out.header = msg.header
        out.info = msg.info

        # Cast to int8 and convert to plain Python ints
        inflated_data = inflated_data.astype(np.int8)
        out.data = inflated_data.flatten().tolist()

        return out



def main(args=None):
    rclpy.init(args=args)
    node = InflationCostmapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
