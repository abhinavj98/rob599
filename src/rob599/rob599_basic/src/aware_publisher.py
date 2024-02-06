#!/usr/bin/env python3

# A connection-aware publisher
#
# aware_publisher.py
#
# Bill Smart
#
# This example shows how to set up a publisher that only publishes a message if a
# subscriber is currently waiting for it.  This is useful if we need to perform
# some large computation in order to assemble the message we want to publish.  A
# good example of this is image processing, which often takes a significant amount
# of time.  We don't want to do this computation if we're not going to actually
# use the results (ie, there are no subscribers). 
#
# You can also call the get_num_connections() function on a subscriber, to see how
# many sources it's subscribing from.


# Import what we need.
import rospy
import sys

from std_msgs.msg import Int64


if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('aware_publisher', argv=sys.argv)

	# Set up a publisher.
	publisher = rospy.Publisher('counter', Int64, queue_size=10)

	# Publish at 10Hz.
	rate = rospy.Rate(10)


	# Set up a counter
	counter = 0

	# Loop until we're shut down.
	while not rospy.is_shutdown():
		# If we have at least one connection, then publish.  This is useful if we
		# need to perform some large computation in order to assemble the message
		# we want to publish.  A good example of this is image processing, which
		# often takes a significant amount of time.  We don't want to do this
		# computation if we're not going to actually use the results (ie, there
		# are no subscribers).  You can also call this function on a subscriber,
		# to see how many sources it's subscribing from.
		if publisher.get_num_connections() > 0:
			publisher.publish(counter)
			rospy.loginfo('Published {0}'.format(counter))

		# Increment the counter.
		counter += 1

		# Limit the publish rate.
		rate.sleep()

