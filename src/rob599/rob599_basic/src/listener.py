#!/usr/bin/env python

# Example of the basic use of tf in ROS.
#
# listener.py
#
# Bill Smart
#
# This shows the basic functionality of tf, allowing us to take a point in one coordinate frame
# and transform it to some other coordinate frame.


# Import rospy and sys, as usual.
import rospy
import sys

# Import the transform library
import tf

# Import a time-stamped Point.  This is the version of Point with coordinate frame information.
from geometry_msgs.msg import PointStamped


if __name__ == '__main__':
	# Initialize the node, as usual.
	rospy.init_node('listener', argv=sys.argv)

	# Set up a tf listener.  This will subscribe to all of the relevant tf topics, and keep track of the
	# information that comes in over them.  We're going to use this listener as a wrapper and an interface
	# to the tf system.
	listener = tf.TransformListener()

	# We're going to publish information at 1 Hz.
	rate = rospy.Rate(1)

	# We're going to loop until the node is shut down.
	while not rospy.is_shutdown():
		# Try to look up the transform we want.  You should always put this in a try-except block, since it might
		# fail on any single call, due to internal timing issues in the transform publishers.
		try:
			# Look up the transform translation and rotation between odom and map, right now.  Setting the time in
			# the past will let you look up transfroms from the past.  This matters when the robot is moving and the
			# relationship between coordinate frames is not fixed.
			translation, rotation = listener.lookupTransform('odom', 'map', rospy.Time())
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			# Fail silently, and try again.
			continue

		# Print out the transform from the odom frame to the map frame, as atranslation (as a 3-vector) and a
		# rotation (as a quaternion).
		rospy.loginfo('T: {0}\nR: {1}'.format(translation, rotation))

		# Create a point that is at the origin of the odom coordinate frame.  We put the coordinate frame name
		# in the frame_id field, and the current time in the stamp field.
		source = PointStamped()
		source.header.frame_id = 'odom'
		source.header.stamp = rospy.Time()

		# Set the position of the point.
		source.point.x = 0.0
		source.point.y = 0.0
		source.point.z = 0.0

		# Write out the source point to the log stream.
		rospy.loginfo(source)

		# Transform the point to the map coordinate frame.  You can transform between any two frames in the tf
		# tree.  Compare the results of this to that of the raw transform, above.
		destination = listener.transformPoint('map', source)
		rospy.loginfo(destination)

		# Manage the rate that we print things out at.
		rate.sleep()
