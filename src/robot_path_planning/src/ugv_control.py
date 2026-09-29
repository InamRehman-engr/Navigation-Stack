#!/usr/bin/env python

"""
This script allows manual control of the UGV (Unmanned Ground Vehicle) using the keyboard.
WASD keys are used for movement, 'z' to increase speed, 'x' to decrease speed, and 'q' to quit.

Think of it as a (teleop_twist_keyboard) but for my custom stack
"""

import rospy
from std_msgs.msg import String
import sys
import select
import tty
import termios

def get_key():
    """Returns the key pressed by the user."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())  # Put the terminal into raw mode
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)  # Wait for input with timeout
        if rlist:
            return sys.stdin.read(1)  # Read one character from stdin
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)  # Restore terminal settings

def ugv_control_publisher():
    # Initialize the ROS node
    rospy.init_node('ugv_state', anonymous=True)
    
    # Create a publisher object that publishes to '/ugv_control'
    ugv_pub = rospy.Publisher('/ugv_control', String, queue_size=10)
    
    print("WASD to control UGV and 'z' to increase speed and 'x' to decrease speed, 'q' to quit.")
    
    while not rospy.is_shutdown():
        # Get the key pressed by the user
        key = get_key()
        
        if key:
            if key == 'w':
                ugv_pub.publish('forward')  # Move UGV forward
                rospy.loginfo("UGV moving forward")
            elif key == 's':
                ugv_pub.publish('reverse')  # Move UGV backward
                rospy.loginfo("UGV moving backward")
            elif key == 'a':
                ugv_pub.publish('left')  # Turn UGV left
                rospy.loginfo("UGV turning left")
            elif key == 'd':
                ugv_pub.publish('right')  # Turn UGV right
                rospy.loginfo("UGV turning right")
            elif key == 'z':
                ugv_pub.publish('speedup')  # Increase speed
                rospy.loginfo("UGV speed increased")
            elif key == 'x':
                ugv_pub.publish('speeddown')  # Decrease speed
                rospy.loginfo("UGV speed decreased")
            elif key == 'q':
                print("Exiting program...")
                break  # Exit the loop if 'q' is pressed
            else:
                rospy.loginfo("Invalid key pressed.")
        else:
            ugv_pub.publish('stop')  # Decrease speed
            rospy.loginfo("UGV stop")

    
        
        # No action is required if no key is pressed.

if __name__ == '__main__':
    try:
        ugv_control_publisher()
    except rospy.ROSInterruptException:
        pass
