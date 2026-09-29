#!/usr/bin/env python

"""
Used to copy wheel velocities and efforts from the front wheels to the corresponding rear wheels of a UGV (Unmanned Ground Vehicle).
A janky way of converting diff drive into skid steer drive.
"""

import rospy
from sensor_msgs.msg import JointState

def joint_state_callback(msg):

    #rospy.loginfo("Received joint names: {}".format(msg.name))
    
    # Check if required joints are present
    required_joints = ['FR-joint', 'BL-joint']
    for joint in required_joints:
        if joint not in msg.name:
            rospy.logwarn("Joint {} not found in incoming message. Skipping...".format(joint))
            return
        
    # Create a new JointState message
    new_msg = JointState()

    try:
        # Get indices of the FL and FR joints
        bl_index = msg.name.index('BL-joint')
        fr_index = msg.name.index('FR-joint')

        # Copy positions of FL and FR joints for BL and BR joints
        bl_position = msg.position[bl_index]
        fr_position = msg.position[fr_index]
        fl_position = bl_position
        br_position = fr_position

        # Prepare the new JointState message
        new_msg.header.stamp = rospy.Time.now()
        new_msg.name = ['FR-joint', 'BL-joint', 'BR-joint', 'FL-joint']
        new_msg.position = [fr_position, bl_position, br_position, fl_position]

        # Optionally copy velocity if available
        if len(msg.velocity) > 0:
            fl_velocity = msg.velocity[bl_index]
            fr_velocity = msg.velocity[fr_index]
            bl_velocity = fl_velocity
            br_velocity = fr_velocity
            new_msg.velocity = [fr_velocity, fl_velocity, br_velocity, fl_velocity]

        # Optionally copy effort if available
        if len(msg.effort) > 0:
            fl_effort = msg.effort[bl_index]
            fr_effort = msg.effort[fr_index]
            bl_effort = fl_effort
            br_effort = fr_effort
            new_msg.effort = [fr_effort, bl_effort, br_effort, fl_effort]

        # Publish the modified joint state
        joint_state_pub.publish(new_msg)

    except ValueError as e:
        rospy.logwarn("Joint not found in the incoming JointState message: {}".format(e))


if __name__ == '__main__':
    rospy.init_node('ugv_joint_state_publisher')

    # Create a subscriber to listen to the joint states of the front wheels
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)

    # Create a publisher to publish the joint states for all four wheels
    joint_state_pub = rospy.Publisher('/joint_states_mimic', JointState, queue_size=10)

    rospy.loginfo("UGV Joint State Publisher is running...")
    rospy.spin()
