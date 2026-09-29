#!/usr/bin/env python

"""
Global planner for a UGV (Unmanned Ground Vehicle) using ROS.
Implements an A* algorithm on a static occupancy grid map with obstacle inflation.
Publishes the computed global path for the robot to follow.
"""

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool
import heapq
import math
import numpy as np
import scipy.ndimage


class AStarGlobalPlanner:
    def __init__(self):
        rospy.init_node('lightweight_global_planner', anonymous=True)

        # Internal state
        self.map = None
        self.inflated_map = None
        self.map_width = 0
        self.map_height = 0
        self.resolution = 0
        self.origin = (0, 0)
        self.robot_pose = None
        self.goal = None

        self.inflation_radius = 0.35  # meters

        # Subscribers
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.pose_sub = rospy.Subscriber('/poseupdate', PoseWithCovarianceStamped, self.pose_callback)
        self.replan_sub = rospy.Subscriber('/replan_path', Bool, self.replan_callback)

        # Publishers
        self.path_pub = rospy.Publisher('/global_path', Path, queue_size=10)
        self.replan_ack_pub = rospy.Publisher('/replan_ack', Bool, queue_size=10)

  
    def map_callback(self, msg):
        if not msg.data:
            rospy.logwarn("Received empty map.")
            return

        self.map = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)

        self.inflate_obstacles()
        rospy.loginfo("Static map received and inflated.")

    def inflate_obstacles(self):
        #Inflate obstacles by self.inflation_radius (in meters).
        inflation_cells = int(self.inflation_radius / self.resolution)

        # Mark all non-zero (occupied or unknown) as 1, rest as 0
        binary_map = (self.map > 0).astype(np.uint8)

        # Apply binary dilation to inflate obstacles
        structure = np.ones((2 * inflation_cells + 1, 2 * inflation_cells + 1), dtype=np.uint8)
        inflated = scipy.ndimage.binary_dilation(binary_map, structure=structure).astype(np.uint8)

        self.inflated_map = inflated

    def pose_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def world_to_map(self, x, y):
        mx = int((x - self.origin[0]) / self.resolution)
        my = int((y - self.origin[1]) / self.resolution)
        return mx, my

    def map_to_world(self, mx, my):
        x = mx * self.resolution + self.origin[0] + self.resolution / 2
        y = my * self.resolution + self.origin[1] + self.resolution / 2
        return x, y

    def is_valid(self, x, y):
        if 0 <= x < self.map_width and 0 <= y < self.map_height:
            return self.inflated_map[y][x] == 0
        return False

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def a_star(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}

        directions = [(-1, 0), (1, 0), (0, -1), (0, 1),
                      (-1, -1), (-1, 1), (1, -1), (1, 1)]

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                return self.reconstruct_path(came_from, current)

            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                if not self.is_valid(neighbor[0], neighbor[1]):
                    continue

                move_cost = math.hypot(dx, dy)
                tentative_g = g_score[current] + move_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))

        return None

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

    def goal_callback(self, msg):
        if self.map is None or self.robot_pose is None:
            rospy.logwarn("Missing map or pose.")
            return

        start = self.world_to_map(self.robot_pose.position.x, self.robot_pose.position.y)
        goal = self.world_to_map(msg.pose.position.x, msg.pose.position.y)

        if not self.is_valid(*goal):
            rospy.logwarn("Goal is not reachable.")
            return

        rospy.loginfo("Planning path from {} to {}".format(start, goal))
        path = self.a_star(start, goal)

        if path:
            self.publish_path(path)
            self.goal = msg
            self.replan_ack_pub.publish(True)
        else:
            rospy.logwarn("No path found.")

    def replan_callback(self, msg):
        if msg.data and self.goal and self.robot_pose:
            rospy.loginfo("Replanning triggered.")
            self.goal_callback(self.goal)

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        for cell in path:
            pose = PoseStamped()
            x, y = self.map_to_world(cell[0], cell[1])
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        rospy.loginfo("Published path with {} points.".format(len(path)))

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    AStarGlobalPlanner().run()
