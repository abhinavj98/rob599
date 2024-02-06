#!/usr/bin/env python3

# Example of using a custom message in a topic subscriber.
#
# custom_subscriber.py
#
# Bill Smart
#
# This shows how to use a custom message type with more than one data field in a subscriber.


import rospy
import sys

# Note, as before, we need to import from the rob599_basic.msg
from rob599_basic.msg import Rectangle


# This is called whenever a new message comes in.  The message, of type Rectangle, is passed as the
# function argument.  You cant get to the individual data fields in the usual way.
def callback(msg):
	rospy.loginfo('Got: {0} by {1}'.format(msg.height, msg.width))


if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('rectangled', argv=sys.argv)

	# Set up the subscriber.
	subscriber = rospy.Subscriber('rectangles', Rectangle, callback)

	# Pass control over to ROS.
	rospy.spin()
