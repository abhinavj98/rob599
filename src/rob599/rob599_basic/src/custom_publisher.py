#!/usr/bin/env python3


# An example of how to use a custom message in a publisher.
#
# custom_publisher.py
#
# Bill Smart
#
# This shows how to use a custom message type with more than one data field.


import rospy
import sys

# Import the message definition
from rob599_basic.msg import Rectangle

from random import randint


if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('rectangler', argv=sys.argv)

	# Set up the publisher.
	publisher = rospy.Publisher('rectangles', Rectangle, queue_size=10)

	# Manage the rate.
	rate = rospy.Rate(1)

	while not rospy.is_shutdown():
		# Create the message and fill in the fields.
		message = Rectangle()
		message.height = randint(1, 10)
		message.width = randint(1, 10)

		# We could also construct the message like this.  The data gets filled in according to the
		# message definition.  Don't use this, though, since it's not as clear to the reader as the
		# other ways to do it.
		#message = Rectangle(randint(1, 10), randint(1, 10))

		# This is a better way to do it in the constructor, with named arguments.
		#message = Rectangle(height=randint(1, 10), width=randint(1, 10))

		publisher.publish(message)

		rate.sleep()

