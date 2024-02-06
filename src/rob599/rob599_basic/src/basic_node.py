#!/usr/bin/env python3

# Basic structure of a Python node.
#
# basic_node.py
#
# Bill Smart
#
# This example shows the basic structure of a ROS node in Python.  This node doesn't
# actually do anything, # but does contain all of the parts needed to set up a node.


# This pulls in the basic Python functionality for ROS.  You will need this in every
# ROS Python node.
import rospy

# It's often a good idea to also import sys, so that we can get access to any
# command-line parameters given to the node.
import sys


# Execution of the node starts here, since it's called as an executable.
if __name__ == '__main__':
	# All packages need to be initialized.  The only required argument here is a
	# name for the node.  We can also pass in any command-line parameters that are
	# fed to the node.
	rospy.init_node('basic_node', argv=sys.argv)

	# Give control over to ROS.  The node will sit here until it gets killed,
	# usually with a ctrl-c from the keyboard
	rospy.spin()

