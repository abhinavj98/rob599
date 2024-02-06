// A basic service server in C++
//
// service.cpp
//
// Bill Smart
//
// This shows how to set up a basic service service in C++


// Include the basic ROS stuff.
#include <ros/ros.h>

// Include the definition of the Doubler message.  This was created for us by catkin_make, and lives in the
// catkin workspace.
#include <rob599_basic/Doubler.h>


// This is the callback for the service.  It takes two arguments, one of the Request type, and one of the
// Response type.  Both of them are (mutable) references.  Unlike Python, the service callback does not
// return the Response.  Instead, set the member variables of the response, and return true.
bool doubler(rob599_basic::Doubler::Request &request, rob599_basic::Doubler::Response &response) {
	// Send a message to the info log channel.
	ROS_INFO("Got %d", request.number);

	// Store the response value in the response argument.
	response.doubled = request.number * 2;

	return true;
}


int main(int argc, char **argv) {
	// Initialize the node and make a node handle.
	ros::init(argc, argv, "server");
	ros::NodeHandle node;

	// Set up the service, using NodeHandle.advertiseService.  This takes a service name, and a callback
	// function.
	ros::ServiceServer service = node.advertiseService("doubler", doubler);
	ROS_INFO("Service started");

	// Give control over to ROS.
	ros::spin();

	return 0;
}