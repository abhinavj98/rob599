#!/usr/bin/env python

# A more encapsulated version of navigation for the Fetch.
#
# better_nav.py
#
# Bill Smart
#
# This example shows a more encapsulated version of basic navigation for the Fetch.  The functionality
# is the same as basic_nav.py, but the interface presented to the user is simpler, and specialized to
# the Fetch.


import rospy
import actionlib

# We need the MoveBaseAction and MoveBaseGoal from the move_base_msgs package.
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# We're going to use the quaternion type here.
from geometry_msgs.msg import Quaternion, Twist

# tf includes a handy set of transformations to move between Euler angles and quaternions (and back).
from tf import transformations

import sys
from sensor_msgs.msg import LaserScan

class FetchMove:
    """Move fetch with velocity commands"""
    def __init__(self):
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    def publish_vel(self):
        msg = Twist()
        msg.linear.x = 0.1
        self.publisher.publish(msg)


if __name__ == '__main__':
    rospy.init_node('vel_publisher')
    vel_publisher = FetchMove()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        vel_publisher.publish_vel()
        rate.sleep()

        
