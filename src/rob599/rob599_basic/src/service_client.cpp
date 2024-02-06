// Basic C++ service client node.
//
// service_client.cpp
//
// Bill Smart
//
// This example shows the basic use of a service client.


// Include the basic ROS stuff.
#include <ros/ros.h>

// Include the definition of the Doubler message.  This was created for us by catkin_make, and lives in the
// catkin workspace.
#include <rob599_basic/Doubler.h>


int main(int argc, char **argv) {
	// Initialize the node and set up the node handle.
	ros::init(argc, argv, "service_client");
	ros::NodeHandle node;

	// Set up the service client with the NodeHandle.serviceClient function.  This is templated on the service
	// definition type (which is auto-generated from the .srv file by catkin), and takes a service name as the
	// single argument.
	ros::ServiceClient client = node.serviceClient<rob599_basic::Doubler>("doubler");

	// We're going to loop from 0 to 4.
	for(int i = 0; i < 5; ++i) {
		// Make an instance of the Doubler type, and will in the request data.  This idiom differs from Python
		// in that you pass the complete service type to the service client, not just the request.  The service
		// call will fill in the response fields, from which you can then extract the data.
		rob599_basic::Doubler service_data;
		service_data.request.number = i;

		// Make the service call.  If the call returns true, then it succeeded, and you can extract data from
		// the Doubler instance.  If it returns false, the call failed.
		if (client.call(service_data)) {
			ROS_INFO("Sent %i got %d", i, service_data.response.doubled);

		} else {
			ROS_ERROR("Service call failed for %i", i);
		}
	}

	return 0;
}