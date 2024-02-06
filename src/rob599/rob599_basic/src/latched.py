#!/usr/bin/env python3

# Basic latched topic publisher example.
#
# latched.py
#
# Bill Smart
#
# This example shows how to use a latched topic.


import rospy
import sys

# We're going to be using Int64 messages, so import them here.
from std_msgs.msg import Int64


if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('latched', argv=sys.argv)

	# Set up the publisher and latch the topics.  This means that, whenever a
	# subscriber connects, it will get the last published message, instead of
	# having to wait for the next message.  This is useful for messages that are
	# not published frequently, like maps.  The last published message is cached
	# by ROS in memory, and then sent out when the new connection is made.
	publisher = rospy.Publisher('counter', Int64, queue_size=10, latch=True)

	# We're going to publish one message every ten seconds.
	rate = rospy.Rate(0.1)

	# A counter, as in the previous example.
	counter = 0

	# Loop until we're shut down.
	while not rospy.is_shutdown():
		# Publish the message.
		publisher.publish(counter)

		# Print out a log message.
		rospy.loginfo('Published {0}'.format(counter))

		# Increment the counter.
		counter += 1

		# Manage the publication rate.
		rate.sleep()

