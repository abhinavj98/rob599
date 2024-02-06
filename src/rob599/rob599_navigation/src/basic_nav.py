#!/usr/bin/env python

# Example of using the navigation stack to move the robot.
#
# basic_nav.py
#
# Bill Smart
#
# This is an example of a simple use of the nav stack to move the robot to a fixed position in the map.


import rospy
import actionlib

# We need the MoveBaseAction and MoveBaseGoal from the move_base_msgs package.
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# tf includes a handy set of transformations to move between Euler angles and quaternions (and back).
from tf import transformations

import sys


def active_callback():
	"""
	A simple callback function for when the navigation action becomes active.
	"""
	rospy.loginfo('Action is active.')


def done_callback(status, result):
	"""
	A simple callback function for when the navigation action ends.

	:param status: The action server status.
	:param result: The result of the action, of type MoveBaseResult.
	"""

	# We can check the status of the action, to make sure we are where we thought we would be.
	if status == actionlib.GoalStatus.SUCCEEDED:
		rospy.loginfo('Arrived at destination.')
	else:
		rospy.loginfo('Something went wrong.')


def feedback_callback(feedback):
	"""
	A simple callback to process the intermittent feedback from the action server.
	"""

	# The feedback from the navigation system is the intermittent poses we go through.
	rospy.loginfo('Feedback {0}:\n{1}\n'.format(feedback_callback.count, feedback.base_position.pose))
	feedback_callback.count += 1

# Add an attribute to the function, so that we can keep track of the number of times we've called it.
feedback_callback.count = 0


if __name__ == '__main__':
	rospy.init_node('basic_nav', argv=sys.argv)

	# Set up a client for the navigation action.  On the Fetch, this is called move_base, and has type
	# MoveBaseAction.  Once we make the client, we wait for the server to be ready.
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	client.wait_for_server()
	rospy.loginfo('Made contact with move_base server')

	# Make a goal for the action.  Specify the coordinate frame that we want (map), and set the time to be
	# now.
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time()

	# Set a position in the coordinate frame.
	goal.target_pose.pose.position.x = 4.0
	goal.target_pose.pose.position.y = 0.0
	goal.target_pose.pose.position.z = 0.0

	# Set an angle.  We're going to use a function from tf.transformations to do this, to make sure the
	# quaternion is right.  There isn't a simpler way to do this, unfortunately, unless we write a wrapper
	# function.  The Fetch can only rotate around z, so the first two parameters will no do anything, and
	# should be set to zero.
	q = transformations.quaternion_from_euler(0.0, 0.0, 0.0)
	goal.target_pose.pose.orientation.x = q[0]
	goal.target_pose.pose.orientation.y = q[1]
	goal.target_pose.pose.orientation.z = q[2]
	goal.target_pose.pose.orientation.w = q[3]

	# Send the action to the action server, than wait until it's done before ending the program.
	client.send_goal(goal, done_cb=done_callback, active_cb=active_callback, feedback_cb=feedback_callback)
	client.wait_for_result()
