// A basic topic publisher example.
//
// publisher.cpp
//
// Bill Smart
//
// This example shows the basic C++ code for a topic publisher.


// Include the basic ROS stuff and the Int64 message definition.
#include <ros/ros.h>
#include <std_msgs/Int64.h>


int main(int argc, char **argv) {
	// Initialize the node, and create the node handle.
	ros::init(argc, argv, "publisher");
	ros::NodeHandle node;

	// Set up a publisher.  We do this through the NodeHandle advertise function.  This is a templated
	// on the published data type (std_msgs::Int64 in our case), and takes a topic name and a queue size.
	ros::Publisher pub = node.advertise<std_msgs::Int64>("counter", 10);

	// Set a publication rate of 10 Hz.
	ros::Rate rate(1);

	// Initialize a counter.  Note that we have to give it an explicit type.
	int counter = 0;

	// Loop until the node shuts down.  The condition is equivalent to not rospy.is_shutdown() in Python.
	while (ros::ok()) {
		// Create an outgoing message, and fill in the data field.
		std_msgs::Int64 msg;
		msg.data = counter;

		// Publish the message.
		pub.publish(msg);

		// Increment the counter.
		counter += 1;

		// Sleep for a bit to control the publish rate.
		rate.sleep();
	}

	return 0;
}