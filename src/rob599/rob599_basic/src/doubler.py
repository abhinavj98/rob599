#!/usr/bin/env python3

# A node that subscribes to a topic, doubles the number it gets, then publishes
# that.
#
# doubler.py
#
# Bill Smart
#
# One of the common uses of ROS nodes is to transform data.  They listen for
# messages on one topic, do some computation on these messages, and then publish
# the result.  This is a simple example of that, where the node listens for an
# integer, doubles it, and then publishes the result.


# Import what we need
import rospy
import sys

from std_msgs.msg import Int64


# The callback that processes any messages received.  This will calculate the
# new value, then publish it out again.  We don't need to explicitly set a rate
# here, since the function will only be called at the rate the incoming messages
# are received.
def callback(msg):
	# Explicitly build a message with the correct data in it.
	new_value = Int64(msg.data * 2)

	# Publish a log message so that we know what's going on.
	rospy.loginfo('Got {0} and published {1}'.format(msg.data, new_value.data))

	# Publish the new value to the output topic.
	publisher.publish(new_value)


if __name__ == '__main__':
	# Initilize the node.
	rospy.init_node('doubler', argv=sys.argv)

	# Set up a publisher.
	publisher = rospy.Publisher('doubled', Int64, queue_size=10)

	# Set up a subscriber.
	subscriber = rospy.Subscriber('number', Int64, callback)

	# Hand over control to ROS.
	rospy.spin()
