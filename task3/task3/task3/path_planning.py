#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__("path_planner")

        # Subscribers
        self.initialpose_sub = self.create_subscription(PoseWithCovarianceStamped, "initialpose", self.initialpose_callback, 10)
        self.goalpose_sub = self.create_subscription(PoseStamped, "goal_pose", self.goalpose_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, "map", self.map_callback, 10)

        # Publisher
        self.path_pub = self.create_publisher(Path, "planned_path", 10)

        self.map_data = None
        self.start_pose = None
        self.goal_pose = None

        self.get_logger().info("Path Planner Node has been started")

    def initialpose_callback(self, msg):
        self.start_pose = msg.pose.pose
        self.get_logger().info(f"Start pose received: {self.start_pose}")
        self.create_path_if_ready()

    def goalpose_callback(self, msg):
        self.goal_pose = msg.pose
        self.get_logger().info(f"Goal pose received: {self.goal_pose}")
        self.create_path_if_ready()

    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info("Map data received")
        self.create_path_if_ready()

    def create_path_if_ready(self):
        if self.map_data and self.start_pose and self.goal_pose:
            self.get_logger().info("Creating path...")
            self.create_path()
        else:
            self.get_logger().info("Waiting for map, start, and goal poses to be set")

    def create_path(self):
        start = (int(self.start_pose.position.x), int(self.start_pose.position.y))
        goal = (int(self.goal_pose.position.x), int(self.goal_pose.position.y))

        self.get_logger().info(f"Calculating path from {start} to {goal}")
        path = self.dijkstra(start, goal)
        if path is not None:
            self.publish_path(path)
        else:
            self.get_logger().warn("No path found")

    def dijkstra(self, start, goal):
        width = self.map_data.info.width
        height = self.map_data.info.height
        map_array = np.array(self.map_data.data).reshape((height, width))

        self.get_logger().info(f"Map size: {width} x {height}")

        dist = {start: 0}
        prev = {}
        unvisited = set((x, y) for x in range(width) for y in range(height) if map_array[y, x] == 0)

        while unvisited:
            current = min(unvisited, key=lambda x: dist.get(x, float('inf')))
            if current == goal or dist.get(current, float('inf')) == float('inf'):
                break
            unvisited.remove(current)
            x, y = current

            neighbors = [(x2, y2) for x2, y2 in [(x-1, y), (x+1, y), (x, y-1), (x, y+1)] if 0 <= x2 < width and 0 <= y2 < height]
            for neighbor in neighbors:
                if neighbor in unvisited:
                    alt = dist[current] + 1
                    if alt < dist.get(neighbor, float('inf')):
                        dist[neighbor] = alt
                        prev[neighbor] = current

        path = []
        current = goal
        while current in prev:
            path.append(current)
            current = prev[current]
        path.reverse()

        if path and path[0] == start:
            self.get_logger().info(f"Path found: {path}")
            return path
        else:
            self.get_logger().warn("Path reconstruction failed")
            return None

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for (x, y) in path:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info("Path published")

def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
