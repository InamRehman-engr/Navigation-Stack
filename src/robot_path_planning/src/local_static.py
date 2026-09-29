#!/usr/bin/env python

"""
This script implements a local planner for a UGV (Unmanned Ground Vehicle) using ROS.
It subscribes to the robot's pose and a global path, and publishes velocity commands to follow the path.
It also supports pausing the motion based on an external signal.
"""

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion
import math
import numpy as np
from std_msgs.msg import Bool


class LocalPlanner:
    def __init__(self):
        rospy.init_node('local_planner')

        # Subscribers
        rospy.Subscriber('/poseupdate', PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber('/local_path', Path, self.path_callback)
        rospy.Subscriber('/pause_local_planner', Bool, self.pause_callback)

        # Publisher
        self.cmd_vel_pub = rospy.Publisher('/local_cmd_vel', Twist, queue_size=3)

        # Variables
        self.pause_motion = False
        self.robot_pose = None
        self.global_path = []

        # Control Parameters
        self.max_vel_x = 0.50
        self.min_vel_x = -0.50
        self.max_vel_theta = 1.0
        self.xy_goal_tolerance = 0.2  # Stop when within 0.3m of the goal
        self.yaw_goal_tolerance = 0.3  # Stop turning when within 0.2 rad
        self.lookahead_distance = 0.6  # Lookahead distance for path tracking
        self.axis_turn_threshold = math.radians(90)  # 60-degree threshold for axis turn

        rospy.loginfo("Strict Path Following Local Planner initialized.")

    def pause_callback(self, msg):
        self.pause_motion = msg.data


    def pose_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def path_callback(self, msg):
        self.global_path = [pose.pose.position for pose in msg.poses]
        rospy.loginfo("New global path received with {} points!".format(len(self.global_path)))

    def find_nearest_waypoint(self):
        if not self.robot_pose or not self.global_path:
            return None
        
        robot_x, robot_y = self.robot_pose.position.x, self.robot_pose.position.y
        distances = [math.sqrt((wp.x - robot_x) ** 2 + (wp.y - robot_y) ** 2) for wp in self.global_path]
        nearest_index = np.argmin(distances)
        return nearest_index

    def find_target_waypoint(self, nearest_index):
        for i in range(nearest_index, len(self.global_path)):
            wp = self.global_path[i]
            robot_x, robot_y = self.robot_pose.position.x, self.robot_pose.position.y
            distance = math.sqrt((wp.x - robot_x) ** 2 + (wp.y - robot_y) ** 2)
            if distance >= self.lookahead_distance:
                return wp
        return self.global_path[-1]  # Default to the last waypoint

    def compute_velocity_command(self):
        if self.pause_motion:
            return Twist()  # Stop if asked by Lidar Avoider

        if not self.robot_pose or not self.global_path:
            rospy.logwarn("Waiting for pose and global path data...")
            return None

        nearest_index = self.find_nearest_waypoint()
        if nearest_index is None:
            return None

        target_wp = self.find_target_waypoint(nearest_index)
        goal_x, goal_y = target_wp.x, target_wp.y
        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y
        
        # Get robot's current yaw
        quaternion = self.robot_pose.orientation
        _, _, robot_yaw = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        
        # Compute the correct heading error
        goal_angle = math.atan2(goal_y - robot_y, goal_x - robot_x)
        heading_error = self.normalize_angle(goal_angle - robot_yaw)

        # Check if UGV reached the goal
        final_goal = self.global_path[-1]
        goal_distance = math.sqrt((robot_x - final_goal.x) ** 2 + (robot_y - final_goal.y) ** 2)
        if goal_distance <= self.xy_goal_tolerance:
            rospy.loginfo("Goal reached! Stopping UGV.")
            return Twist()  # Stop the robot

        # Create velocity command
        cmd_vel = Twist()
        
        # Perform axis turn if heading error is large
        if abs(heading_error) > self.axis_turn_threshold:
            rospy.loginfo("Performing axis turn due to large heading error.")
            cmd_vel.linear.x = 0.0  # Stop forward motion
            cmd_vel.angular.z = max(-self.max_vel_theta, min(self.max_vel_theta, 2.0 * heading_error))
        else:
            # Normal path following
            cmd_vel.linear.x = max(self.min_vel_x, self.max_vel_x * (1 - abs(heading_error)))
            cmd_vel.angular.z = max(-self.max_vel_theta, min(self.max_vel_theta, 2.0 * heading_error))
        
        return cmd_vel

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            cmd_vel = self.compute_velocity_command()
            if cmd_vel:
                self.cmd_vel_pub.publish(cmd_vel)
            rate.sleep()

if __name__ == "__main__":
    try:
        planner = LocalPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Local Planner node terminated.")



