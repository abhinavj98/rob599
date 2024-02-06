#!/usr/bin/env python

# A more encapsulated version of navigation for the Fetch.
#
# better_nav.py
#
# Bill Smart
#
# This example shows a more encapsulated version of basic navigation for the Fetch.  The functionality
# is the same as basic_nav.py, but the interface presented to the user is simpler, and specialized to
# the Fetch.


import rospy
import actionlib

# We need the MoveBaseAction and MoveBaseGoal from the move_base_msgs package.
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# We're going to use the quaternion type here.
from geometry_msgs.msg import Quaternion

# tf includes a handy set of transformations to move between Euler angles and quaternions (and back).
from tf import transformations

import sys


class FetchNavigation:
	"""
	A simple encapsulation of the navigation stack for a Fetch robot.
	"""
	def __init__(self):
		"""
		Create an instance of the simple navigation interface.
		"""

		# Make an action client, and wait for the server.
		self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.client.wait_for_server()
		rospy.loginfo('{0}: Made contact with move_base server'.format(self.__class__.__name__))

		# Set up some dummy callbacks.  These can be overwritten to actually set up callbacks.
		self.active_callback = None
		self.feedback_callback = None
		self.done_callback = None

		# Make a goal template, and fill in all of the relevant fields.
		self.goal = MoveBaseGoal()
		self.goal.target_pose.header.frame_id = 'map'
		self.goal.target_pose.header.stamp = rospy.Time()

		# Set a position in the coordinate frame.
		self.goal.target_pose.pose.position.x = 0.0
		self.goal.target_pose.pose.position.y = 0.0
		self.goal.target_pose.pose.position.z = 0.0

		self.goal.target_pose.pose.orientation = FetchNavigation._get_quaternion(0.0)

	@staticmethod
	def _get_quaternion(theta):
		"""
		A helper function to build Quaternians from Euler angles.  Since the Fetch lives on the floor, and
		only rotates around z, we can zero out the other angles.

		:param theta: The angle the robot makes with the x-axis.
		"""
		return Quaternion(*transformations.quaternion_from_euler(0.0, 0.0, theta))

	def go_to(self, x, y, theta, wait=False):
		"""
		Drive the robot to a particlar pose on the map.  Since the Fetch lives on the floor, we only need
		(x, y) coordinates and a heading.

		:param x: x coordinate in the map frame.
		:param y: y coordinate in the map frame.
		:param theta: heading (angle with the x-axis in the map frame)
		"""

		rospy.loginfo('{0}: Heading for ({1}, {2}) at {3} radians'.format(self.__class__.__name__,
			x, y, theta))

		# Set the x and y positions
		self.goal.target_pose.pose.position.x = x
		self.goal.target_pose.pose.position.y = y

		# Set the orientation.  This is a quaternion, so we use the helper function.
		self.goal.target_pose.pose.orientation = FetchNavigation._get_quaternion(0.0)

		# Make the action call.  Include the callbacks.  Unless these have been set somewhere else, they are passed
		# as None, which means no callback.
		self.client.send_goal(self.goal, active_cb=self.active_callback, feedback_cb=self.feedback_callback,
			done_cb=self.done_callback)

		# Wait, if asked to.
		if wait:
			self.client.wait_for_result()


if __name__ == '__main__':
	rospy.init_node('better_nav', argv=sys.argv)

	# Make an instance of the navigator.
	nav = FetchNavigation()

	# Go back and forth 5 times.
	for _ in range(5):
		nav.go_to(3.0, 0.0, 0.0, wait=True)
		nav.go_to(3.0, -4.0, 3.14, wait=True)



