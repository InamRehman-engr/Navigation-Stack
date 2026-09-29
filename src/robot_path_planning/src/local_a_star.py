#!/usr/bin/env python

"""
This script implements a local planner for a UGV (Unmanned Ground Vehicle) using ROS.
It subscribes to the robot's pose, a global path, and laser scan data to build a local costmap.
It publishes a local path for the robot to follow, dynamically updating based on obstacles detected in the costmap.
"""

import rospy
import math
import heapq
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from scipy.ndimage import binary_dilation
from time import time

class LocalCostmapBuilder:
    def __init__(self):
        self.grid_size_m = 4.5
        self.grid_resolution = 0.1  # 10cm resolution
        self.grid_cells = int(self.grid_size_m / self.grid_resolution)
        self.costmap = np.zeros((self.grid_cells, self.grid_cells), dtype=np.uint8)
        self.robot_radius = 0.2  # Robot radius in meters
        self.safety_margin = 0.15  # Safety margin in meters
        self.inflation_radius_cells = int((self.robot_radius + self.safety_margin) / self.grid_resolution)

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

    def scan_callback(self, scan_msg):
        self.costmap = np.zeros((self.grid_cells, self.grid_cells), dtype=np.uint8)
        angle = scan_msg.angle_min
        robot_cx = self.grid_cells // 2
        robot_cy = self.grid_cells // 2

        for r in scan_msg.ranges:
            if math.isinf(r) or math.isnan(r) or r > 6.0:
                angle += scan_msg.angle_increment
                continue

            x = r * math.cos(angle)
            y = r * math.sin(angle)

            mx = int(robot_cx + x / self.grid_resolution)
            my = int(robot_cy + y / self.grid_resolution)

            if 0 <= mx < self.grid_cells and 0 <= my < self.grid_cells:
                self.costmap[my][mx] = 1  # mark obstacle

            angle += scan_msg.angle_increment

        self.inflate_obstacles()

    def inflate_obstacles(self):
        """Inflate obstacles by robot radius + safety margin to avoid tight spaces."""
        structure = np.ones((2 * self.inflation_radius_cells + 1, 2 * self.inflation_radius_cells + 1))
        self.costmap = binary_dilation(self.costmap, structure=structure).astype(np.uint8)

    def get_costmap(self):
        return self.costmap


class AStarLocalPlannerDynamic:
    def __init__(self):
        rospy.init_node('dynamic_local_planner')

        self.costmap_builder = LocalCostmapBuilder()
        self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=10)

        self.pose_sub = rospy.Subscriber('/poseupdate', PoseWithCovarianceStamped, self.pose_callback)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.global_path_sub = rospy.Subscriber('/global_path', Path, self.global_path_callback)

        self.robot_pose = None
        self.goal_pose = None
        self.global_path = None

        # Time variables to control path updates
        self.last_path_time = 0
        self.path_update_interval = 5.0  # in seconds

        self.last_planned_path = None
        self.last_goal = None

    def pose_callback(self, msg):
        self.robot_pose = msg.pose.pose
        rospy.loginfo_throttle(5, "Received robot pose.")

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        rospy.loginfo("Received goal pose.")

    def global_path_callback(self, msg):
        self.global_path = msg.poses
        rospy.loginfo_throttle(5, "Received global path with {} points.".format(len(msg.poses)))

    def world_to_local(self, x, y, robot_x, robot_y, robot_yaw):
        dx = x - robot_x
        dy = y - robot_y

        lx = dx * math.cos(-robot_yaw) - dy * math.sin(-robot_yaw)
        ly = dx * math.sin(-robot_yaw) + dy * math.cos(-robot_yaw)

        return lx, ly

    def plan_local_path(self):
        current_time = time()
        if current_time - self.last_path_time < self.path_update_interval:
            return

        if self.robot_pose is None or self.global_path is None:
            rospy.logwarn_throttle(5, "Waiting for robot pose and global path...")
            return

        costmap = self.costmap_builder.get_costmap()
        robot_cx = costmap.shape[1] // 2
        robot_cy = costmap.shape[0] // 2

        quaternion = (
            self.robot_pose.orientation.x,
            self.robot_pose.orientation.y,
            self.robot_pose.orientation.z,
            self.robot_pose.orientation.w
        )
        _, _, yaw = euler_from_quaternion(quaternion)

        closest_idx = self.find_closest_path_index()
        if closest_idx is None:
            rospy.logwarn_throttle(5, "Could not find nearby global path point!")
            return

        # Dynamic goal selection
        max_distance = self.costmap_builder.grid_size_m / 2.0
        rx = self.robot_pose.position.x
        ry = self.robot_pose.position.y

        selected_idx = closest_idx
        for i in range(closest_idx, len(self.global_path)):
            px = self.global_path[i].pose.position.x
            py = self.global_path[i].pose.position.y
            distance = math.hypot(px - rx, py - ry)
            if distance <= max_distance:
                selected_idx = i
            else:
                break

        goal_pose_local = self.global_path[selected_idx].pose
        local_goal_x, local_goal_y = self.world_to_local(
            goal_pose_local.position.x,
            goal_pose_local.position.y,
            self.robot_pose.position.x,
            self.robot_pose.position.y,
            yaw
        )

        goal_mx = int(robot_cx + local_goal_x / self.costmap_builder.grid_resolution)
        goal_my = int(robot_cy + local_goal_y / self.costmap_builder.grid_resolution)

        goal_mx = max(0, min(costmap.shape[1] - 1, goal_mx))
        goal_my = max(0, min(costmap.shape[0] - 1, goal_my))

        rospy.loginfo_throttle(2, "Planning local path to cell ({}, {})".format(goal_mx, goal_my))

        path = self.a_star((robot_cx, robot_cy), (goal_mx, goal_my), costmap)

        if path:
            self.publish_path(path)
            self.last_path_time = current_time
            self.last_planned_path = path
            self.last_goal = goal_pose_local
        else:
            rospy.logwarn("Could not find local path to goal.")

    def find_closest_path_index(self):
        if self.global_path is None or self.robot_pose is None:
            return None

        min_dist = float('inf')
        closest_idx = None
        rx = self.robot_pose.position.x
        ry = self.robot_pose.position.y

        for i, pose_stamped in enumerate(self.global_path):
            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y
            dist = math.hypot(px - rx, py - ry)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        return closest_idx

    def a_star(self, start, goal, grid):
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

                if 0 <= neighbor[0] < grid.shape[1] and 0 <= neighbor[1] < grid.shape[0]:
                    if grid[neighbor[1]][neighbor[0]] != 0:
                        continue  # obstacle

                    # Prevent diagonal through corners
                    if dx != 0 and dy != 0:
                        if grid[current[1]+dy][current[0]] != 0 or grid[current[1]][current[0]+dx] != 0:
                            continue

                    move_cost = math.hypot(dx, dy)
                    tentative_g = g_score[current] + move_cost

                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f_score = tentative_g + math.hypot(goal[0] - neighbor[0], goal[1] - neighbor[1])
                        heapq.heappush(open_set, (f_score, neighbor))

        return None

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y

        quaternion = (
            self.robot_pose.orientation.x,
            self.robot_pose.orientation.y,
            self.robot_pose.orientation.z,
            self.robot_pose.orientation.w
        )
        _, _, yaw = euler_from_quaternion(quaternion)

        for mx, my in path:
            lx = (mx - self.costmap_builder.grid_cells // 2) * self.costmap_builder.grid_resolution
            ly = (my - self.costmap_builder.grid_cells // 2) * self.costmap_builder.grid_resolution

            wx = robot_x + lx * math.cos(yaw) - ly * math.sin(yaw)
            wy = robot_y + lx * math.sin(yaw) + ly * math.cos(yaw)

            pose = PoseStamped()
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        self.local_path_pub.publish(path_msg)
        rospy.loginfo_throttle(2, "Published local path with {} points.".format(len(path)))

    def run(self):
        rate = rospy.Rate(10.0)  # 10 Hz
        while not rospy.is_shutdown():
            self.plan_local_path()
            rate.sleep()


if __name__ == "__main__":
    planner = AStarLocalPlannerDynamic()
    planner.run()
