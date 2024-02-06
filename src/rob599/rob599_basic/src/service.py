#!/usr/bin/env python3

# Example of a simple service.
#
# service.py
#
# Bill Smart
#
# This shows how to put together a simple service client in Python.  This depends on the service message
# being defined and compiled first.


# Import what we need.
import rospy
import sys


# Import the service message definitions that we need, from the current package.  Note that we use
# rob599_basic.srv as the package name.  For the server, we need the base message (Doubler) and the
# response (DoublerResponse), which is automatically created for us by catkin.
from rob599_basic.srv import Doubler, DoublerResponse


# This callback is called whenever a service request comes in.  The parameter is of type DoublerRequest, and
# the return, of type DoublerResponse, is returned by the service call mechanism.
def callback(request):
	rospy.loginfo('Got {0}'.format(request.number))

	# The return from the callback is used as the response from the service call.
	return DoublerResponse(request.number * 2)


if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('server')

	# Create the service, specifying the name, the service message base type, and the callback function
	# to be used to fulfil the requests.
	service = rospy.Service('doubler', Doubler, callback)

	rospy.loginfo('Service started')

	# Give control over to ROS.
	rospy.spin()
