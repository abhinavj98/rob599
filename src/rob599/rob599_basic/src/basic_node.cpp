// The basic C++ node.
//
// basic_node.cpp
//
// Bill Smart
//
// This is an example of the most basic C++ node in ROS.  It doesn't actually do anything useful,
// but does illustrate some of the boilerplate code that you'll need in all of your other nodes.


// Include the basic ROS stuff.  This is the C++ equivalent to import rospy in Python nodes.
#include <ros/ros.h>


// This is the main function, where program execution starts.  This is similar to the
//  if __name__ == '__main__': line in Python.
int main(int argc, char **argv) {
	// Initualize the node.  Note that the argument order is different in C++ and Python.
	ros::init(argc, argv, "basic_node_cpp");

	// Every node needs to make a NodeHandle.
	ros::NodeHandle node;

	// Give control over to ROS.
	ros::spin();

	// Returning 0 to the operating system, signals that the code ended normally.
	return 0;
}