#!/usr/bin/env python

# A slightly more complex example of the use of state machines.
#
# state_machine.py
#
# Bill Smart
#
# A more complex example of state machines, simulating a navigation task.


import rospy
import sys

# Import smach and the smach_ros interface.
import smach
import smach_ros

# Some additional imports to make things easier to see in the smach viewer.
from random import random
from time import sleep


class GoTo(smach.State):
	"""
	A simple smach state that simulates faulty navigation to a place in the world.
	"""
	def __init__(self, place):
		# Initialize the base state.  We can either succeed of fail.
		smach.State.__init__(self, outcomes=['success', 'failure'])

		# Save the name of the place we're going to navigate to.
		self.place = place

	def execute(self, userdata):
		"""
		Pretend to navigate to a place in the world.  We've put in a sleep so that we can see the states
		evolve in smach viewer, and added in a 50% chance of the navigation failing.
		"""

		rospy.loginfo('Heading to {0}'.format(self.place))

		# Put in a short sleep, so that we see the progress through the state machine.
		sleep(3)

		# This line simulates navigation failure.  Suppose that we have a terrible robot, and we fail to
		# navigate 50% of the time.
		if random() < 0.5:
			rospy.loginfo('Made it to {0}!'.format(self.place))
			return 'success'
		else:
			rospy.loginfo('Navigation failed for {0}!'.format(self.place))
			return 'failure'


class Charge(smach.State):
	"""
	A simple smach state the simulates the robot charging itself.
	"""
	def __init__(self):
		smach.State.__init__(self, outcomes=['charged'])

	def execute(self, userdata):
		"""
		Pretend to charge the robot.
		"""

		rospy.loginfo('Charging!')

		# Pretend to charge
		sleep(3)

		rospy.loginfo('Charged!')

		return 'charged'


if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('state_machine', argv=sys.argv)

	# Define a set of rooms, with an implicit ordering, for the robot to visit.  We're going to use this
	# to procedurally define the state machine.
	rooms = ['Kitchen', 'Hallway', 'Living Room', 'Bedroom 1', 'Bedroom 2', 'Office']

	# Add "Go to" the state names for rooms to make the diagram more readable.
	rooms = ['Go to {0}'.format(room) for room in rooms]

	# Make a state machine.  This state machine is going to be a loop, so we use an empty list to specify the
	# overall machine outcomes.
	state_machine = smach.StateMachine(outcomes=[])
	with state_machine:
		# Add a destination where the robot can charge itself.  Once it gets there, it transitions to the Charge
		# state.
		smach.StateMachine.add('Go to Charging Station', GoTo('Go to Charging Station'),
			transitions={'success':'Charge', 'failure':'Go to Charging Station'})

		# Add a state to simulate charging.  Note that this has only one outcome, when it transitions to the 
		# navigation state for the first room.
		smach.StateMachine.add('Charge', Charge(),
			transitions={'charged':rooms[0]})

		# After the last room, drive back to the charging station.
		smach.StateMachine.add(rooms[-1], GoTo(rooms[-1]),
			transitions={'success':'Go to Charging Station', 'failure':rooms[-1]})

		# For each of the rooms in the list, make a navigation state for it, and link it to the next room in the
		# list on success.  On failure, we'll just try again.
		for i in range(len(rooms) - 1):
			smach.StateMachine.add(rooms[i], GoTo(rooms[i]),
				transitions={'success':rooms[i + 1], 'failure':rooms[i]})

	# Start up an introspection server, so that we can look at the state machine in smach_viewer.
	sis = smach_ros.IntrospectionServer('room_checker_machine', state_machine, '/STATE_MACHINE_ROOT')
	sis.start()

	# Start the state machine
	final_outcome = state_machine.execute()

	# We're going to put in a spin() just to keep the node alive, so we can look at the state machine structure.
	rospy.spin()	