#!/usr/bin/env python


# Examples of smach states used in the nav_machine.py example.
#
# nav_states.py
#
# Bill Smart
#
# These are example states used in the navigation example.  Since states should be designed to be reused,
# it's a good idea to store them in their own files and modules, so that you can compose them into new
# robot behaviors in the furure.


import rospy
import smach
import smach_ros

# Some messages
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from tf import transformations

# Some math stuff/
from math import pi
from random import uniform


class Spin(smach.State):
	"""
	A simple smach state that will spin the robot at a certain angular velocity for an approximate fixed duration.
	"""
	def __init__(self, angular=1.0, duration=2.0):
		smach.State.__init__(self, outcomes=['done'])

		# Record the duration
		self.duration = duration

		# Set up a publisher
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

		# Set up a Twist, and make sure it's zeroed out.
		self.twist = Twist()
		self.twist.linear.x = 0.0
		self.twist.linear.y = 0.0
		self.twist.linear.z = 0.0
		self.twist.angular.x = 0.0
		self.twist.angular.y = 0.0
		self.twist.angular.z = angular


	def execute(self, userdata):
		# Publish at 10 Hz
		rate = rospy.Rate(10)

		# Publish for about the right amount of time.
		for _ in range(int(self.duration * 10)):
			self.pub.publish(self.twist)
			rate.sleep()

		# Only one way to end this state.
		return 'done'


class Dance(smach.StateMachine):
	"""
	A simple Smach state machine that does a little wiggle dance.
	"""
	def __init__(self):
		smach.StateMachine.__init__(self, outcomes=['done'])

		with self:
			smach.StateMachine.add('To the left!', Spin(angular=1.0, duration=2.0), transitions={'done':'To the right!'})
			smach.StateMachine.add('To the right!', Spin(angular=-1.0, duration=2.0), transitions={'done':'done'})



class GoTo(smach_ros.SimpleActionState):
	"""
	An example of inheriting from the SimpleActionState
	"""
	def __init__(self, coordinates=None):
		# This is the base class initialization.  Whenever the state is triggered, the goal_cb callback is called
		# and the results are treated as the goal.  If we were using the same goal point location every time, then
		# we should used the fixed goal version of this initialization, not the callback one.
		smach_ros.SimpleActionState.__init__(self, 'move_base', MoveBaseAction, goal_cb=self.get_goal)

		# Save the target coordinates.  If we don't specify these, they'll default to None.
		self.coordinates = coordinates

		# We're gong to use the same goal message every time.
		self.goal = MoveBaseGoal()
		self.goal.target_pose.header.stamp = rospy.Time()
		self.goal.target_pose.header.frame_id = 'map'
		self.goal.target_pose.pose.position.x = 0.0
		self.goal.target_pose.pose.position.y = 0.0
		self.goal.target_pose.pose.position.z = 0
		self.goal.target_pose.pose.orientation = Quaternion(*transformations.quaternion_from_euler(0.0, 0.0, 0.0))

	def get_goal(self, userdata, default_goal):
		"""
		Goal callback.  If the initialization parameter was None, then this picks a different random state each
		time it's called.  If we gave some coordinates, then these are used instead.
		"""
		if self.coordinates:	
			self.goal.target_pose.pose.position.x = self.coorindates[0]
			self.goal.target_pose.pose.position.y = self.coorindates[1]
		else:
			self.goal.target_pose.pose.position.x = uniform(2, 6)
			self.goal.target_pose.pose.position.y = uniform(-4, 2)
			self.goal.target_pose.pose.orientation = Quaternion(*transformations.quaternion_from_euler(0.0, 0.0, uniform(0.0, 2 * pi)))

		return self.goal