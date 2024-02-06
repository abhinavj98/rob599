#!/usr/bin/env python

# Simple robot navigation state machine.
#
# nav_machine.py
#
# Bill Smart
#
# This shows the use of SimpleActionState in a state machine that interacts with the ROS navigation stack.


import rospy
import sys

# We're going to use smach and smach_ros in this example.
import smach
import smach_ros


# To keep things clean, we've defined a couple of states in another file, and we're going to include them here.
from nav_states import Dance, GoTo


if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('nav_machine', argv=sys.argv)

	# Make a state machine, and populate it with states.
	state_machine = smach.StateMachine(outcomes=['success', 'failure'])
	with state_machine:
		# Drive to a random place in the world.  If we get there do a dance.  If we abort, then try again.  If the
		# action is preempted (which should not happen), then fail and end the state machine.
		smach.StateMachine.add('Drive', GoTo(),
			transitions={'succeeded':'Dance!', 'preempted':'failure', 'aborted':'Drive'})

		# Dance, and then go find another place to drive to.
		smach.StateMachine.add('Dance!', Dance(), transitions={'done':'Drive'})

	# Set up a visualization server, and start it.
	sis = smach_ros.IntrospectionServer('room_checker_machine', state_machine, '/STATE_MACHINE_ROOT')
	sis.start()

	# We should never get past this line until the machine ends.
	final_outcome = state_machine.execute()
