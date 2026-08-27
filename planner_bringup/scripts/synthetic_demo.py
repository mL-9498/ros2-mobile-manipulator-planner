#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from planner_interfaces.msg import CoverageCandidate
from planner_interfaces.srv import ComputeSetCover, PlanGridPath, PlanUnorderedTasks
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
        self.dijkstra = self.create_client(PlanUnorderedTasks, "plan_unordered_tasks")

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
        self.base_ids = sorted(self.candidates)
        self.set_cover_selected = []
        self.dijkstra_sequence = []
        self.travel_costs = [math.inf] * (len(self.base_ids) ** 2)
        self.paths = {}
        self.pending_pairs = []
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

    def make_candidates(self):
        messages = []
        for base_id in self.base_ids:
            x, y, covered = self.candidates[base_id]
            candidate = CoverageCandidate()
            candidate.base_id = base_id
            candidate.pose.x = x
            candidate.pose.y = y
            candidate.pose.theta = 0.0
            candidate.covered_task_ids = covered
            messages.append(candidate)
        return messages

    def start(self):
        self.publish_scene()
        if self.started:
            return
        if not all((
            self.set_cover.service_is_ready(),
            self.astar.service_is_ready(),
            self.dijkstra.service_is_ready(),
        )):
            self.get_logger().info("Waiting for planner services...")
            return
        self.started = True
        self.timer.cancel()

        cover_request = ComputeSetCover.Request()
        cover_request.task_count = len(self.tasks)
        cover_request.candidates = self.make_candidates()
        self.set_cover.call_async(cover_request).add_done_callback(self.on_set_cover)

        count = len(self.base_ids)
        for i in range(count):
            self.travel_costs[i * count + i] = 0.0
            for j in range(count):
                if i != j:
                    self.pending_pairs.append((i, j))
        self.plan_next_pair()

    def on_set_cover(self, future):
        try:
            result = future.result()
        except Exception as error:
            self.get_logger().error(f"Set Cover call failed: {error}")
            return
        if result.success:
            self.set_cover_selected = list(result.selected_base_ids)
            self.get_logger().info(
                f"Set Cover selected bases: {self.set_cover_selected}")
            self.publish_scene()
        else:
            self.get_logger().error(result.message)

    def plan_next_pair(self):
        if not self.pending_pairs:
            self.request_dijkstra()
            return
        i, j = self.pending_pairs.pop(0)
        request = self.make_astar_request(
            self.candidates[self.base_ids[i]],
            self.candidates[self.base_ids[j]],
        )
        self.astar.call_async(request).add_done_callback(
            lambda future, pair=(i, j): self.on_pair_path(future, pair))

    def make_astar_request(self, start, goal):
        request = PlanGridPath.Request()
        request.width = self.width
        request.height = self.height
        request.resolution = self.resolution
        request.origin_x = 0.0
        request.origin_y = 0.0
        request.occupancy = self.occupancy
        request.start.x, request.start.y = start[0], start[1]
        request.goal.x, request.goal.y = goal[0], goal[1]
        return request

    def on_pair_path(self, future, pair):
        i, j = pair
        try:
            result = future.result()
        except Exception as error:
            self.get_logger().error(f"A* pair call failed: {error}")
            self.plan_next_pair()
            return
        if result.success:
            count = len(self.base_ids)
            self.travel_costs[i * count + j] = result.path_length
            self.paths[(self.base_ids[i], self.base_ids[j])] = result.path
        self.plan_next_pair()

    def request_dijkstra(self):
        request = PlanUnorderedTasks.Request()
        request.task_count = len(self.tasks)
        request.candidates = self.make_candidates()
        request.travel_cost_matrix = self.travel_costs
        request.start_base_id = -1
        self.dijkstra.call_async(request).add_done_callback(self.on_dijkstra)

    def on_dijkstra(self, future):
        try:
            result = future.result()
        except Exception as error:
            self.get_logger().error(f"Dijkstra call failed: {error}")
            return
        if not result.success:
            self.get_logger().error(result.message)
            return

        self.dijkstra_sequence = [visit.base_id for visit in result.visits]
        self.get_logger().info(
            f"Dijkstra base sequence: {self.dijkstra_sequence}; "
            f"cost={result.total_cost:.2f} m; "
            f"expanded_states={result.expanded_states}"
        )
        for step, visit in enumerate(result.visits, start=1):
            self.get_logger().info(
                f"Step {step}: base {visit.base_id}, "
                f"new tasks={list(visit.newly_completed_task_ids)}, "
                f"move={visit.move_cost:.2f} m"
            )
        self.publish_scene()
        self.publish_dijkstra_path()

    def publish_dijkstra_path(self):
        combined = Path()
        combined.header.frame_id = "map"
        combined.header.stamp = self.get_clock().now().to_msg()
        for start, goal in zip(
            self.dijkstra_sequence[:-1], self.dijkstra_sequence[1:]
        ):
            segment = self.paths.get((start, goal))
            if segment is None:
                continue
            poses = segment.poses
            if combined.poses and poses:
                poses = poses[1:]
            combined.poses.extend(poses)
        self.path_pub.publish(combined)

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
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
                0.95, 0.2, 0.2, 1.0)
            markers.markers.append(marker)

        for base_id, (x, y, _) in self.candidates.items():
            marker = self.make_marker(
                now, "candidates", base_id, Marker.CUBE, x, y)
            marker.scale.x = marker.scale.y = 0.28
            marker.scale.z = 0.12
            if base_id in self.set_cover_selected:
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
                    0.1, 0.9, 0.2, 1.0)
                marker.scale.x = marker.scale.y = 0.42
            else:
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
                    0.15, 0.35, 1.0, 0.65)
            markers.markers.append(marker)

        for order, base_id in enumerate(self.dijkstra_sequence, start=1):
            x, y, _ = self.candidates[base_id]
            label = self.make_marker(
                now, "dijkstra_order", base_id, Marker.TEXT_VIEW_FACING, x, y)
            label.pose.position.z = 0.55
            label.text = str(order)
            label.scale.z = 0.35
            label.color.r, label.color.g, label.color.b, label.color.a = (
                1.0, 0.75, 0.0, 1.0)
            markers.markers.append(label)

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
