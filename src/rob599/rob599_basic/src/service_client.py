#!/usr/bin/env python3

# Example of a simple service client.
#
# service_client.py
#
# Bill Smart
#
# This is a simple service client.


import rospy
import sys

# Import the base service message type.  Note that, as with the server, we need the .srv extension on
# the package name (asuming we named the service message directory "srv", which is conventional).
from rob599_basic.srv import Doubler


if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('service_client', argv=sys.argv)

	# This will wait for the service to become available.  It's a good idea to put this in here, since some
	# of the code below will fail if the service does not exist.
	rospy.wait_for_service('doubler')

	# Assuming that the service is up, setting up a service proxy gives us access to it.  We'll need a name
	# and a base service message type.
	doubler = rospy.ServiceProxy('doubler', Doubler)

	# To illustrate the use of a service, we're going to count up from zero, sending the numbers to the
	# service and logging the responses.
	for i in range(5):
		# Once we have the service proxy set up, we can use it just like a function.  The arguments to this
		# function are copied into the fields of a DoublerRequest instance, and then sent over.  The functor
		# returns an instance of DoublerResponse, from which you can access as usual.  It's good practice to
		# wrap this in a try-except block in case something goes wrong.
		try:
			answer = doubler(i)
		except rospy.ServiceException as e:
			rospy.logwarn('Service call failed for {0}: {1}'.format(i, e))

		rospy.loginfo('Sent {0} and got {1}'.format(i, answer.doubled))
