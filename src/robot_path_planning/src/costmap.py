#!/usr/bin/env python


"""
Costmap generator for a UGV (Unmanned Ground Vehicle) using ROS.
Generates an inflated global costmap from the static occupancy grid map.
"""

import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
from scipy.ndimage import distance_transform_edt

class CostmapGenerator:
    def __init__(self):
        rospy.init_node('costmap_generator', anonymous=True)

        # Subscribers
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # Publishers
        self.costmap_pub = rospy.Publisher('/global_costmap', OccupancyGrid, queue_size=10, latch=True)

        # Internal state
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.05
        self.map_origin = (0, 0)

        # Parameters from YAML configuration
        self.inflation_radius = rospy.get_param("~inflation_radius", 0.2)  # meters
        self.obstacle_range = rospy.get_param("~obstacle_range", 6.0)      # meters
        self.raytrace_range = rospy.get_param("~raytrace_range", 8.0)      # meters
        self.footprint = rospy.get_param("~footprint", [[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]])

    def map_callback(self, msg):
        rospy.loginfo("Map received.")
        
        # Get map metadata
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)

        # Check for valid map data
        if self.map_data is None or self.map_data.size == 0:
            rospy.logwarn("Empty map data received.")
            return

        # Create inflated costmap
        inflated_costmap = self.generate_inflated_costmap()

        # Publish inflated costmap
        self.publish_costmap(inflated_costmap, msg)

    def generate_inflated_costmap(self):
        # Handle unknown values (-1) and set static obstacles
        static_costmap = np.where(self.map_data == -1, -1, 0)  # Preserve unknown cells as -1
        static_costmap[self.map_data == 100] = 100  # Obstacles as 100

        rospy.loginfo("Static costmap generated.")

        # Inflate obstacles
        inflated_costmap = self.apply_inflation(static_costmap)

        rospy.loginfo("Inflated costmap generated.")
        return inflated_costmap

    def apply_inflation(self, costmap):
        # Binary map: obstacles as 1, others as 0 (ignoring -1 for unknown areas)
        binary_map = np.where(costmap == 100, 1, 0)

        # Compute distances from obstacles
        distances = distance_transform_edt(1 - binary_map) * self.map_resolution

        # Inflate obstacles based on inflation radius
        inflated_costmap = np.copy(costmap)  # Start with the static costmap
        inflated_costmap[(distances <= self.inflation_radius) & (costmap != -1)] = 100

        return inflated_costmap

    def publish_costmap(self, costmap, original_map):
        # Create OccupancyGrid message
        costmap_msg = OccupancyGrid()
        costmap_msg.header.stamp = rospy.Time.now()
        costmap_msg.header.frame_id = "map"
        costmap_msg.info = original_map.info

        # Convert costmap to list (required by OccupancyGrid)
        flattened_costmap = costmap.flatten()
        costmap_msg.data = flattened_costmap.tolist()

        # Publish the inflated costmap
        self.costmap_pub.publish(costmap_msg)


    def run(self):
        rospy.spin()


if __name__ == "__main__":
    generator = CostmapGenerator()
    generator.run()

