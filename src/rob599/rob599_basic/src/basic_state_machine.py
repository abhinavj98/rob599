#!/usr/bin/env python


# An example of building a state machine in ROS.
#
# state_machine.py
#
# Bill Smart
#
# This code is an example of how to put together a state machine in ROS using smach.  The
# state machine is defined in code, rather than in some description format, which gives it
# more flexibility.  Note that, although we're setting this up as a node, it doesn't really
# do anything particular to ROS.


import rospy
import sys

# Import the smach, the state machine handler.
import smach

# Import the ROS interface to smach, which includes the visualizer, and some other useful stuff.
import smach_ros


# Smach states are Python classes, which inherit from smach.State.
class PrintState(smach.State):
	"""
	Create state.  This is a very simple example of a state that simply prints a string to the info log
	channel.
	"""
	def __init__(self, text):
		# This line calls the constructor on the base class, eumerating the possible outcomes of the state.
		# If you don't include this line, then the state machine will not work.
		smach.State.__init__(self, outcomes=['printed'])

		# Initialize anything that's specific to your state here.
		self.text = text

	def execute(self, userdata):
		"""
		Function that is called when the state becomes active.  The state ceases to be active when this
		function returns.  Control will pass to the next state in the FSM, according to the return value.
		"""		
		rospy.loginfo(self.text)

		# You have to return one of the strings that's enumerated in the outcomes parameter passed to the
		# smach.State base class.
		return 'printed'


if __name__ == '__main__':
	# Initialize the node, as usual.
	rospy.init_node('basic_state_machine', argv=sys.argv)

	# Make a state machine, and enumerate all of the return values (outcomes) it can have.
	state_machine = smach.StateMachine(outcomes=['done'])

	# Open the state machine context with a with statement.  Everything in the indent block below the with
	# statement will apply to this state machine.  it's possible to have more than one state machine in a
	# single node, although only one can be active at a time.
	with state_machine:
		# Add a state to the state machine, giving a name, a State subclass instance, and a dictionary that
		# specifies what happens on the possible transition values.
		smach.StateMachine.add('State 1', PrintState('First state'), transitions={'printed':'State 2'})

		# Since this State subclass transitions to done (a string passed as an outcome for the state machine),
		# that will end the state machine execution.
		smach.StateMachine.add('State 2', PrintState('Second state'), transitions={'printed':'done'})

	# Start up an introspection server, so that we can look at the state machine in smach_viewer.
	sis = smach_ros.IntrospectionServer('simple_state_machine', state_machine, '/STATE_MACHINE_ROOT')
	sis.start()

	# Start up the state machine.
	final_outcome = state_machine.execute()

	# We're going to put in a spin() just to keep the node alive, so we can look at the state machine structure.
	rospy.spin()
