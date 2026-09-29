#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry

"""
This used when running the Hector slam node independently (need to uncomment the relevant node in the hector_slam.launch file).
It publishes the transform from the scanmatcher_frame to the odom frame and from the odom frame to the base_link frame.
"""

# Global variables for odometry
odom_x = 0.0
odom_y = 0.0
odom_theta = 0.0

def odometry_callback(msg):
    """Update odometry based on LIDAR-based localization."""
    global odom_x, odom_y, odom_theta

    # Get pose from LIDAR odometry
    odom_x = msg.pose.pose.position.x
    odom_y = msg.pose.pose.position.y

    # Convert quaternion to euler angles
    orientation_q = msg.pose.pose.orientation
    euler = tf.transformations.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    odom_theta = euler[2]  # Extract yaw

def broadcast_tf():
    """Publishes the necessary TFs: scanmatcher_frame->odom and odom->base_link"""
    rospy.init_node('odom_tf_broadcaster')

    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()

    # Subscribe to LIDAR odometry
    rospy.Subscriber("/odom", Odometry, odometry_callback)

    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        try:
            # Get the transform from scanmatcher_frame to odom
            listener.waitForTransform('scanmatcher_frame', 'odom', rospy.Time(0), rospy.Duration(1.0))
            (trans_scanmatcher_to_odom, rot_scanmatcher_to_odom) = listener.lookupTransform('scanmatcher_frame', 'odom', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("Transform scanmatcher_frame->odom not available: %s", e)
            continue

        # Broadcast scanmatcher_frame -> odom
        br.sendTransform(
            trans_scanmatcher_to_odom,
            rot_scanmatcher_to_odom,
            rospy.Time.now(),
            "odom",
            "scanmatcher_frame"
        )

        # Broadcast odom->base_link using LIDAR odometry
        base_link_translation = (odom_x, odom_y, 0.0)
        base_link_rotation = tf.transformations.quaternion_from_euler(0.0, 0.0, odom_theta)

        br.sendTransform(
            base_link_translation,
            base_link_rotation,
            rospy.Time.now(),
            "base_link",
            "odom"
        )

        rate.sleep()

if __name__ == '__main__':
    try:
        broadcast_tf()
    except rospy.ROSInterruptException:
        pass