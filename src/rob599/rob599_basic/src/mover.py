#!/usr/bin/env python3

# An example of sending movement commands to the Fetch robot.
#
# mover.py
#
# Bill Smart
#
# This shows the basic mechanism for moving the Fetch robot by publishing Twist messages.


# Import what we need from ROS and Python
import rospy
import sys

# The Twist message is used to send velocities to the robot.
from geometry_msgs.msg import Twist


if __name__ == '__main__':
	# Initialize the node, as usual.
	rospy.init_node('mover', argv=sys.argv)

	# Make a Twist message.  We're going to set all of the elements, since we can't depend on them defaulting
	# to safe values.  This is a good practice, in general.
	cmd = Twist()

	# A Twist has three linear velocities (in meters per second), along each of the axes.  For the Fetch, it
	# will only pay attention to the x velocity, since it can't directly move in the y direction (side-to-side)
	# or the z direction (levitating up in the air).
	cmd.linear.x = 0.2
	cmd.linear.y = 0.0
	cmd.linear.z = 0.0

	# A Twist also has three rotational velocities (in radians per second).  The Fetch will only respond to
	# rotations around the z (vertical) axis, and will ignore the others.
	cmd.angular.x = 0.0
	cmd.angular.y = 0.0
	cmd.angular.z = 0.0

	# Make a publisher, as usual.  The Fetch will listen for Twist messages on the cmd_vel topic.
	publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

	# Limit the publication rate.
	rate = rospy.Rate(10)

	# Publish until the node shuts down.  This will drive the roobt forward at 0.2 meters per second.
	while not rospy.is_shutdown():
		publisher.publish(cmd)

		rate.sleep()

	# Zero out the x velocity.  Now, the Twist message corresponds to a zero velocity along all linear and
	# rotational axes.
	cmd.linear.x = 0.0

	# Publish this zero-velocity command a few times.  We do this, since it's not guaranteed that any particular
	# message gets through.  The sleep() gives the publishing thread a bit of time to make sure the nessages get
	# send.  If we didn't include this, then the node might shut down before the messages are sent.  This would
	# mean that the robot would not stop.
	for i in range(10):
		publisher.publish(cmd)
		rate.sleep()

