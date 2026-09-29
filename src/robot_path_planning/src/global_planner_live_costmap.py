#!/usr/bin/env python

"""
Global planner for a UGV (Unmanned Ground Vehicle) using ROS.
Implements an A* algorithm on a live costmap.
Publishes the computed global path for the robot to follow.
"""

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance
from std_msgs.msg import Bool
import numpy as np
import heapq
import math


class AStarGlobalPlanner:
    def __init__(self):
        rospy.init_node('global_planner', anonymous=True)

        # Subscribers
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.inflated_map_sub = rospy.Subscriber('global_costmap', OccupancyGrid, self.inflated_map_callback)
        self.amcl_sub = rospy.Subscriber('/poseupdate', PoseWithCovarianceStamped, self.amcl_callback)
        self.replan_sub = rospy.Subscriber('/replan_path', Bool, self.replan_callback)

        # Publishers
        self.path_pub = rospy.Publisher('/global_path', Path, queue_size=10)
        self.replan_ack_pub = rospy.Publisher('/replan_ack', Bool, queue_size=10)  # Replan acknowledgment

        # Internal state
        self.map_data = None
        self.costmap_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin = (0, 0)
        self.goal = None
        self.robot_pose = None  # To store robot's current position
        self.replan_ack = False



    def amcl_callback(self, msg):
        """Callback to update the robot's current position from HECTOR_SLAM."""
        self.robot_pose = msg.pose.pose
        rospy.loginfo("Received robot pose from HECTOR_SLAM.")

    def is_valid_cell(self, x, y):
        """Check if a cell is valid (not an obstacle or out of bounds)."""
        if 0 <= x < self.map_width and 0 <= y < self.map_height:
            cell_cost = self.costmap_data[y][x]
            return 0 <= cell_cost < 100  # Avoid obstacles and unknown regions
        return False

    def map_callback(self, msg):
        """Callback for the static map."""
        if not msg.data:
            rospy.logwarn("Received empty map data!")
            return

        self.map_data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        rospy.loginfo("Static map received.")

    def inflated_map_callback(self, msg):
        """Callback for the inflated costmap."""
        if not msg.data:
            rospy.logwarn("Received empty costmap data!")
            return

        self.costmap_data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        rospy.loginfo("Inflated costmap received.")

    def goal_callback(self, goal_msg):
        """Callback for setting the goal and computing the global path."""
        if self.map_data is None or self.costmap_data is None or self.robot_pose is None:
            rospy.logwarn("Map, costmap, or robot pose not received yet! Cannot compute path.")
            return

        start = self.world_to_map(self.robot_pose.position.x, self.robot_pose.position.y)
        goal = self.world_to_map(goal_msg.pose.position.x, goal_msg.pose.position.y)

        if not self.is_valid_cell(*goal):
            rospy.logwarn("Goal is in an obstacle! Cannot compute path.")
            return

        rospy.loginfo("Calculating global path from {} to {}.".format(start, goal))
        path = self.a_star(start, goal)

        if path:
            self.publish_path(path)
            self.goal = goal_msg  # Store goal for potential replanning
            self.replan_ack_pub.publish(True)  # Notify that replanning is complete
        else:
            rospy.logwarn("No valid path found!")

    def world_to_map(self, x, y):
        """Convert world coordinates to map indices."""
        mx = int((x - self.map_origin[0]) / self.map_resolution)
        my = int((y - self.map_origin[1]) / self.map_resolution)
        return mx, my

    def map_to_world(self, mx, my):
        """Convert map indices to world coordinates."""
        wx = mx * self.map_resolution + self.map_origin[0]
        wy = my * self.map_resolution + self.map_origin[1]
        return wx, wy

    def heuristic(self, a, b):
        """Heuristic function for A* (Euclidean distance)."""
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def a_star(self, start, goal):
        """A* algorithm for global path planning."""
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 4-connectivity
        open_list = []
        heapq.heappush(open_list, (0, start))
        costs = {start: 0}
        came_from = {}

        while open_list:
            _, current = heapq.heappop(open_list)

            if current == goal:
                return self.reconstruct_path(came_from, start, goal)

            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)

                if self.is_valid_cell(neighbor[0], neighbor[1]):
                    new_cost = costs[current] + 1
                    heuristic_cost = new_cost + self.heuristic(neighbor, goal)

                    if neighbor not in costs or new_cost < costs[neighbor]:
                        costs[neighbor] = new_cost
                        heapq.heappush(open_list, (heuristic_cost, neighbor))
                        came_from[neighbor] = current

        return None

    def reconstruct_path(self, came_from, start, goal):
        """Reconstruct the path from start to goal."""
        current = goal
        path = []
        while current != start:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path

    def replan_callback(self, msg):
        """Callback to trigger re-planning when an obstacle is detected."""
        if msg.data and self.robot_pose and self.goal:
            rospy.logwarn("Replanning global path due to obstacle!")
            self.replan_ack = True
            self.goal_callback(self.goal)  # Use the last stored goal

    def publish_path(self, path):
        """Publish the computed global path."""
        global_path = Path()
        global_path.header.frame_id = "map"
        global_path.header.stamp = rospy.Time.now()

        for cell in path:
            pose = PoseStamped()
            wx, wy = self.map_to_world(cell[0], cell[1])
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            global_path.poses.append(pose)

        self.path_pub.publish(global_path)
        rospy.loginfo("Global path published successfully.")

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    planner = AStarGlobalPlanner()
    planner.run()
