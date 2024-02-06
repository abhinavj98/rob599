#!/usr/bin/env python3


# A node that subscribes to a topic, multiplies the number it gets, then publishes
# that.
#
# multiplier.py
#
# Bill Smart
#
# This is a second example of a transformer node.  In this example, we're going
# to encapsulate the implementation of the node in a class, so that we can have
# a simpler set of code in the body of the node.  Since we're using a class, we
# can easily hold information in that class instance that affects the behavior of
# the node.


# Import the stuff that we need
import rospy
import sys

from std_msgs.msg import Int64


class Multiplier:
	def __init__(self, multiplier):
		# Save the multiplier for future use.
		self.multiplier = multiplier

		# Set up a subscriber and assign it to an instance variable.  If we
		# assign this to a local variable, rather than an instance variable,
		# then it will go out of scope when the __init__ function ends.  This
		# will mean that we don't have a subscriber, and things won't work.  To
		# make sure we get the behavior we work, assign it to an instance
		# variable.
		self.subscriber = rospy.Subscriber('number', Int64, self.callback)

		# Set up a publisher and assign it to an instance variable.  Again, this
		# needs to be an instance variable to make sure it doesn't get garbage
		# collected and destroyed.
		self.publisher = rospy.Publisher('multiplied_by_{0}'.format(multiplier),
			Int64, queue_size=10)


	def callback(self, msg):
		# Explicitly build a message with the correct data in it.  We're going
		# to use the instance variable that we used to store the multiplier.
		new_value = Int64(msg.data * self.multiplier)

		# Publish a log message so that we know what's going on.  Since we're 
		rospy.loginfo('Got {0} and published {1}'.format(msg.data,
			new_value.data))

		# Publish the new value to the output topic.  For this version, the
		# publisher is local to the class instance.
		self.publisher.publish(new_value)


if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('multiplier', argv=sys.argv)

	# Let's build a list of publishers.
	publishers = [Multiplier(i) for i in range(1, 4)]

	# Give control over the ROS, as usual.
	rospy.spin()
