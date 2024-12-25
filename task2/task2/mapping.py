#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped

from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster, TransformListener, Buffer 
from tf2_ros import TransformException

import math
import numpy as np
import bresenham

RESOLUTION = 0.05 # [m/cell]

WORLD_WIDTH = 15 # [m]
WORLD_HEIGHT = 15 # [m]

MAP_WIDTH = int(WORLD_WIDTH / RESOLUTION) # [cells]
MAP_HEIGHT= int(WORLD_HEIGHT / RESOLUTION) # [cells]

WORLD_ORIGIN_X = -WORLD_WIDTH / 2 # [m]
WORLD_ORIGIN_Y = -WORLD_HEIGHT / 2 # [m]


class MappingNode(Node):
    def __init__(self):
        super().__init__("mapping_node")
    
        # QoS Profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
    
        # Publisher
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', qos_profile=qos_profile)

        # Subscriber
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.update_map, 10)

        # Init TF Tree (static part)
        tf_map_odom = TransformStamped()
        tf_map_odom.header.stamp = self.get_clock().now().to_msg()
        tf_map_odom.header.frame_id = 'map'
        tf_map_odom.child_frame_id = 'odom'

        # TF Broadcaster
        self.tf_static_publisher = StaticTransformBroadcaster(self)
        self.tf_static_publisher.sendTransform(tf_map_odom)
        self.tf_publisher = TransformBroadcaster(self)

        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Init map
        self.world_width = WORLD_WIDTH
        self.world_height = WORLD_HEIGHT
        self.map_width = MAP_WIDTH
        self.map_height = MAP_HEIGHT
        self.world_origin_x = WORLD_ORIGIN_X
        self.world_origin_y = WORLD_ORIGIN_Y
        self.grid_map = np.full((MAP_WIDTH, MAP_HEIGHT), -1, dtype="int8") # Set all map values to "unknown"

        self.publish_map()

        # Cyclic publishing map every half second
        self.create_timer(0.5, self.publish_map)

    def publish_map(self):
        #############################
        # TODO (not absolutely necessary): Converting map total values in map probability values
        #############################

        # Create map message
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = "map"
        map_msg.info.resolution = RESOLUTION 
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = self.world_origin_x
        map_msg.info.origin.position.y = self.world_origin_y
        map_msg.data = self.grid_map.flatten().tolist()

        # Publish map message
        self.map_pub.publish(map_msg)
        
    def update_map(self, msg): 
        # Query the transformation between "map" and "base_footprint"
        rotation = 0.0
        translation_x = 0.0
        translation_y = 0.0

        try:
            latest_time = self.tf_buffer.get_latest_common_time('map', 'base_footprint')
            tf = self.tf_buffer.lookup_transform("map", "base_footprint", latest_time, rclpy.duration.Duration(seconds=1.0))
            q = tf.transform.rotation
            rotation = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2)) # Rotation 
            translation_x = tf.transform.translation.x
            translation_y = tf.transform.translation.y
            self.get_logger().info(f"\nrotation: {[q.x,q.y,q.z,q.w]}\ntranslation: {[translation_x,translation_y]}")
        except TransformException as ex:
            self.get_logger().error(f"Own Error: {ex}")

        inverse_rotation = -rotation
        inv_translation_x = -translation_x * math.cos(inverse_rotation) - translation_y * math.sin(inverse_rotation)
        inv_translation_y = -translation_x * math.sin(inverse_rotation) + translation_y * math.cos(inverse_rotation)

   
        # Calculate the obstacle position in the world based on the robots position [m]
        world_robot_x = -inv_translation_x
        world_robot_y = -inv_translation_y
        world_obstacles_x = []
        world_obstacles_y = []
        for idx, range in enumerate(msg.ranges):
            angle = msg.angle_min + idx * msg.angle_increment
            if msg.range_min < range < msg.range_max:
                # Calculate obstacle position in the robot's frame
                robot_obstacle_x = range * math.cos(angle)
                robot_obstacle_y = range * math.sin(angle)

                # Convert the obstacle position to the world frame
                world_obstacle_x = robot_obstacle_x * math.cos(inverse_rotation) - robot_obstacle_y * math.sin(inverse_rotation) + world_robot_x
                world_obstacle_y = robot_obstacle_x * math.sin(inverse_rotation) + robot_obstacle_y * math.cos(inverse_rotation) + world_robot_y
            
                world_obstacles_x.append(world_obstacle_x)
                world_obstacles_y.append(world_obstacle_y)

        # Calculate the obstacle position in the map based on the robot position [cell]
        map_robot_x = int((world_robot_x - WORLD_ORIGIN_X) / RESOLUTION)
        map_robot_y = int((world_robot_y - WORLD_ORIGIN_Y) / RESOLUTION)


        for world_obstacle_x, world_obstacle_y in zip(world_obstacles_x, world_obstacles_y):
            map_obstacle_x = int((world_obstacle_x - WORLD_ORIGIN_X) / RESOLUTION)
            map_obstacle_y = int((world_obstacle_y - WORLD_ORIGIN_Y) / RESOLUTION)
            if 0 <= map_obstacle_x < self.grid_map.shape[0] and 0 <= map_obstacle_y < self.grid_map.shape[1]:  # Check bounds
                for point in list(bresenham.bresenham(map_robot_x, map_robot_y, map_obstacle_x, map_obstacle_y)):
                    if 0 <= point[0] < self.grid_map.shape[0] and 0 <= point[1] < self.grid_map.shape[1]:  # Check bounds
                        self.grid_map[point[0], point[1]] = 0 # free space
                self.grid_map[map_obstacle_x, map_obstacle_y] = 100 # obstacle

                         

def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    try:
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        node.get_logger().info(f"Own KeyboardInterrupt")
    # node.destroy_node()


if __name__ == '__main__':
    main()