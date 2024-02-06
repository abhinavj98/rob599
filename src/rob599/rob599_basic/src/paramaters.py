#!/usr/bin/env python3

# Example of the use of ROS parameters.
#
# parameters.py
#
# Bill Smart
#
# This shows how to get and set parameters in ROS.


import rospy
import sys


if __name__ == '__main__':
	# Initialize the node
	rospy.init_node('parameters', argv=sys.argv)

	# Set a parameter in a relative namespace
	rospy.set_param('foo', 3)

	# Setting a parameter with a dictionary value.
	d = {'p': 2, 'i': 3, 'd': 4}
	rospy.set_param('gains', d)

	# Getting a parameter.  If the parameter doesn't exist, ROS will raise a KeyError.
	rospy.loginfo('foo: {0}'.format(rospy.get_param('foo')))

	# Getting a parameter, with a default value.
	rospy.loginfo('bar: {0}'.format(rospy.get_param('bar', 123)))
