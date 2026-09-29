#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path

class VelocitySafetyFilter(object):

    def __init__(self):
        rospy.init_node("velocity_safety_filter", anonymous=False)

        self.latest_cmd = Twist()
        self.scan = None

        # ---------- PARAMETERS ----------
        self.ttc_threshold = 1.0        # seconds
        self.robot_radius = 0.30        # meters
        self.front_fov = 0.6            # radians (~35 deg)
        self.max_turn_speed = 0.8       # rad/s
        self.local_path = None

        rospy.Subscriber("/local_path", Path, self.path_cb, queue_size=1)
        rospy.Subscriber("/local_cmd_vel", Twist, self.cmd_cb, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=1)

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        rospy.loginfo("Velocity Safety Filter initialized")

    def path_cb(self, msg):
        self.local_path = msg

    def cmd_cb(self, msg):
        self.latest_cmd = msg

    def scan_cb(self, msg):
        self.scan = msg

    def process(self):
        # No scan yet  passthrough
        if self.scan is None:
            self.cmd_pub.publish(self.latest_cmd)
            return

        v = self.latest_cmd.linear.x
        min_ttc = float("inf")

        angle = self.scan.angle_min

        for r in self.scan.ranges:
            if not np.isfinite(r):
                angle += self.scan.angle_increment
                continue

            # Only consider FRONT beams
            if abs(angle) > self.front_fov:
                angle += self.scan.angle_increment
                continue

            closing_speed = v * math.cos(angle)

            # Ignore non closing beams
            if closing_speed <= 0.0:
                angle += self.scan.angle_increment
                continue

            ttc = (r - self.robot_radius) / closing_speed
            min_ttc = min(min_ttc, ttc)

            angle += self.scan.angle_increment

        cmd = Twist()

        if min_ttc > self.ttc_threshold:
            # SAFE  allow planner command
            cmd = self.latest_cmd
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.5)
        else:
            # UNSAFE  STOP and TURN
            cmd.linear.x = 0.0
            direction = self.choose_turn_direction()
            cmd.angular.z = direction
            self.cmd_pub.publish(cmd)
            rospy.sleep(1.5)
            if self.front_is_clear():
                cmd.linear.x = 0.5
                self.cmd_pub.publish(cmd)
                rospy.sleep(0.5)


    def front_is_clear(self, min_dist=0.8):
        """Returns True if front has enough free space"""
        angle = self.scan.angle_min

        for r in self.scan.ranges:
            if not np.isfinite(r):
                angle += self.scan.angle_increment
                continue

            if abs(angle) < self.front_fov:
                if r < min_dist:
                    return False

            angle += self.scan.angle_increment

        return True

    def choose_turn_direction(self):
        # Default LIDAR-based logic
        angles = np.linspace(
            self.scan.angle_min,
            self.scan.angle_max,
            len(self.scan.ranges)
        )
        ranges = np.array(self.scan.ranges, dtype=np.float32)

        left = ranges[(angles > 0.3) & (angles < 1.5)]
        right = ranges[(angles < -0.3) & (angles > -1.5)]

        left = left[np.isfinite(left)]
        right = right[np.isfinite(right)]

        left_clear = np.mean(left) if len(left) else 0.0
        right_clear = np.mean(right) if len(right) else 0.0

        # Default direction based on LIDAR
        direction = self.max_turn_speed if left_clear > right_clear else -self.max_turn_speed

        # Bias toward local path if available
        if self.local_path and len(self.local_path.poses) > 0:
            # Take first pose in path
            path_pose = self.local_path.poses[0].pose.position
            # Calculate angle to path in robot frame (approx)
            dx = path_pose.x
            dy = path_pose.y
            path_angle = math.atan2(dy, dx)

            # Bias LIDAR decision: prefer turning toward path
            if path_angle > 0:
                direction = max(direction, 0.2)  # slight right bias if path is to left
            else:
                direction = min(direction, -0.2)  # slight left bias if path is to right

        return direction

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.process()
            rate.sleep()


if __name__ == "__main__":
    try:
        VelocitySafetyFilter().run()
    except rospy.ROSInterruptException:
        pass



