// A simple C++ topic subscriber node.
//
// subscriber.cpp
//
// Bill Smart
//
// This example shows the basic code for a topic subscriber.


// include the basic ROS stuff, and the Int64 definition.
#include <ros/ros.h>
#include <std_msgs/Int64.h>


// This is the callback function.  It should have a void return type, and a single argument that is a reference to a const pointer
// to the message type.  All ROS messages will allow you to use this sort of syntax.
void callback(const std_msgs::Int64::ConstPtr &msg) {
	// Send a message to the info log channel.
	ROS_INFO("Got %d", msg->data);
}


int main(int argc, char **argv) {
	// Initialize the node and set up the node handle.
	ros::init(argc, argv, "subscriber");
	ros::NodeHandle node;

	// Set up a subscriber. This takes a topic name, queue length, and a callback function.
	ros::Subscriber sub = node.subscribe("counter", 10, callback);

	// Give control over to ROS.
	ros::spin();

	return 0;
}