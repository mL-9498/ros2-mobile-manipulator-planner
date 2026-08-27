#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Path
from planner_interfaces.msg import CoverageCandidate
from planner_interfaces.srv import ComputeSetCover, PlanGridPath
from visualization_msgs.msg import Marker, MarkerArray


class SyntheticPlanningDemo(Node):
    def __init__(self):
        super().__init__("synthetic_planning_demo")
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, "demo/map", qos)
        self.marker_pub = self.create_publisher(MarkerArray, "demo/markers", qos)
        self.path_pub = self.create_publisher(Path, "demo/path", qos)
        self.set_cover = self.create_client(ComputeSetCover, "compute_set_cover")
        self.astar = self.create_client(PlanGridPath, "plan_grid_path")

        self.width, self.height, self.resolution = 40, 30, 0.2
        self.occupancy = [0] * (self.width * self.height)
        self._add_obstacles()
        self.tasks = [(1.2, 1.0), (1.4, 4.4), (6.6, 1.2), (6.4, 4.6)]
        self.candidates = {
            0: (1.8, 2.6, [0, 1]),
            1: (6.0, 2.8, [2, 3]),
            2: (3.0, 1.2, [0, 2]),
            3: (5.0, 4.6, [1, 3]),
        }
        self.selected = []
        self.started = False
        self.timer = self.create_timer(0.5, self.start)

    def _add_obstacles(self):
        for y in range(self.height):
            if 12 <= y <= 17:
                continue
            self.occupancy[y * self.width + 20] = 100
        for x in range(8, 16):
            for y in range(8, 11):
                self.occupancy[y * self.width + x] = 100

    def start(self):
        self.publish_scene()
        if self.started:
            return
        if not self.set_cover.service_is_ready() or not self.astar.service_is_ready():
            self.get_logger().info("Waiting for planner services...")
            return
        self.started = True
        self.timer.cancel()

        request = ComputeSetCover.Request()
        request.task_count = len(self.tasks)
        for base_id, (x, y, covered) in self.candidates.items():
            candidate = CoverageCandidate()
            candidate.base_id = base_id
            candidate.pose.x = x
            candidate.pose.y = y
            candidate.pose.theta = 0.0
            candidate.covered_task_ids = covered
            request.candidates.append(candidate)
        self.set_cover.call_async(request).add_done_callback(self.on_set_cover)

    def on_set_cover(self, future):
        try:
            result = future.result()
        except Exception as error:
            self.get_logger().error(f"Set Cover call failed: {error}")
            return
        if not result.success:
            self.get_logger().error(result.message)
            return
        self.selected = list(result.selected_base_ids)
        self.get_logger().info(f"Selected base poses: {self.selected}")
        self.publish_scene()
        if len(self.selected) < 2:
            return

        start = self.candidates[self.selected[0]]
        goal = self.candidates[self.selected[1]]
        request = PlanGridPath.Request()
        request.width = self.width
        request.height = self.height
        request.resolution = self.resolution
        request.origin_x = 0.0
        request.origin_y = 0.0
        request.occupancy = self.occupancy
        request.start.x, request.start.y = start[0], start[1]
        request.goal.x, request.goal.y = goal[0], goal[1]
        self.astar.call_async(request).add_done_callback(self.on_path)

    def on_path(self, future):
        try:
            result = future.result()
        except Exception as error:
            self.get_logger().error(f"A* call failed: {error}")
            return
        if not result.success:
            self.get_logger().error(result.message)
            return
        self.path_pub.publish(result.path)
        self.get_logger().info(
            f"A* path contains {len(result.path.poses)} poses; "
            f"length={result.path_length:.2f} m"
        )

    def publish_scene(self):
        now = self.get_clock().now().to_msg()
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.header.stamp = now
        grid.info.resolution = self.resolution
        grid.info.width = self.width
        grid.info.height = self.height
        grid.info.origin.orientation.w = 1.0
        grid.data = self.occupancy
        self.map_pub.publish(grid)

        markers = MarkerArray()
        for task_id, (x, y) in enumerate(self.tasks):
            marker = self.make_marker(now, "tasks", task_id, Marker.SPHERE, x, y)
            marker.scale.x = marker.scale.y = marker.scale.z = 0.22
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.95, 0.2, 0.2, 1.0
            markers.markers.append(marker)

        for base_id, (x, y, _) in self.candidates.items():
            marker = self.make_marker(now, "candidates", base_id, Marker.CUBE, x, y)
            marker.scale.x = marker.scale.y = 0.28
            marker.scale.z = 0.12
            if base_id in self.selected:
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 0.9, 0.2, 1.0
                marker.scale.x = marker.scale.y = 0.42
            else:
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.15, 0.35, 1.0, 0.65
            markers.markers.append(marker)

        self.marker_pub.publish(markers)

    @staticmethod
    def make_marker(stamp, namespace, marker_id, marker_type, x, y):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        return marker


def main():
    rclpy.init()
    node = SyntheticPlanningDemo()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
