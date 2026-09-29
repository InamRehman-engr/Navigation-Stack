#!/usr/bin/env python

"""
Hardware interface for a UGV (Unmanned Ground Vehicle) using ROS.
Converts cmd_vel commands to individual wheel velocities, publishes them to the Arduino,
and integrates wheel velocity feedback to update joint states and odometry.
"""

import rospy
import tf
import math
import time
import tf.transformations
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState


class HardwareInterface:
    def __init__(self):
        rospy.init_node("hardware_interface", anonymous=True)

        # Robot parameters (Adjust for your UGV)
        self.wheel_radius = 0.070  # Wheel radius in meters
        self.wheel_base = 0.406    # Distance between wheels in meters

        # Initialize previous positions
        self.left_wheel_position = 0.0
        self.right_wheel_position = 0.0

        # Subscribe to cmd_vel to get velocity commands
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

        # Subscribe to Arduino wheel velocity feedback
        rospy.Subscriber("/arduino_ugv_vel", Float32MultiArray, self.arduino_wheel_vel_callback)

        # Publish wheel velocities to the Arduino
        self.wheel_vel_pub = rospy.Publisher("/wheel_velocity_controller", Float32MultiArray, queue_size=10)

        # Publish joint states
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

        # Track last time for integration
        self.last_time = rospy.Time.now()

        rospy.loginfo("Hardware Interface Node Initialized")

    def cmd_vel_callback(self, msg):
        """ Convert cmd_vel to wheel velocities and send commands """
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Compute individual wheel velocities
        w_left = (linear_x - (angular_z * self.wheel_base / 2)) / self.wheel_radius
        w_right = (linear_x + (angular_z * self.wheel_base / 2)) / self.wheel_radius

        v_left = w_left * self.wheel_radius
        v_right = w_right * self.wheel_radius

        # Publish wheel velocities
        wheel_vel_msg = Float32MultiArray()
        wheel_vel_msg.data = [v_left, v_right]
        self.wheel_vel_pub.publish(wheel_vel_msg)
    
    def arduino_wheel_vel_callback(self, msg):
        pass
        """
        if len(msg.data) < 2:
            rospy.logwarn("Invalid wheel velocity data received from Arduino")
            return

        # Get the current time
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()  # Time difference in seconds
        self.last_time = current_time  # Update last time

        # Get the velocities from the message
        left_wheel_vel = msg.data[0]  # Linear velocity (m/s)
        right_wheel_vel = msg.data[1]  # Linear velocity (m/s)

        # Convert linear velocity to angular velocity (rad/s)
        left_wheel_angular_vel = left_wheel_vel / self.wheel_radius
        right_wheel_angular_vel = right_wheel_vel / self.wheel_radius

        # Integrate to update wheel positions
        self.left_wheel_position += left_wheel_angular_vel * dt
        self.right_wheel_position += right_wheel_angular_vel * dt

        # Create JointState message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()

        # Define joint names for left and right wheels
        joint_state_msg.name = ['FR-joint', 'BL-joint']

        # Assign the computed positions and velocities
        joint_state_msg.position = [self.right_wheel_position, self.left_wheel_position]
        joint_state_msg.velocity = [right_wheel_angular_vel, left_wheel_angular_vel]
        joint_state_msg.effort = [0.0, 0.0]  # Placeholder effort values (if needed)

        # Publish the joint state message
        self.joint_state_pub.publish(joint_state_msg)
    
    def update_odometry(self, left_wheel_vel, right_wheel_vel):
        # Compute and publish odometry based on Arduino wheel encoder readings
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        v_left = left_wheel_vel 
        v_right = right_wheel_vel

        # Compute robot velocity
        vx = (v_right + v_left) / 2
        vtheta = (v_right - v_left) / self.wheel_base

        # Update pose estimate
        self.x += vx * math.cos(self.theta) * dt
        self.y += vx * math.sin(self.theta) * dt
        self.theta += vtheta * dt

        # Publish transform
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0), quaternion, current_time, "base_link", "odom"
        )
        
        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vtheta

        self.odom_pub.publish(odom)

        self.last_time = current_time  """

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        hardware_interface = HardwareInterface()
        hardware_interface.run()
    except rospy.ROSInterruptException:
        pass






















