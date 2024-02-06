#!/usr/bin/env python3

# An example of publishing markers for rviz.
#
# markers.py
#
# Bill Smart
#
# This node will publish a spherical marker above the current robot position, as it drives around
# the world.


# Import rospy and sys, as usual
import rospy
import sys

# Import the Marker message type from the visualization_msgs package.
from visualization_msgs.msg import Marker


if __name__ == '__main__':
	# Initialize the node, as usual
	rospy.init_node('markers', argv=sys.argv)

	# Create a marker.  Markers of all shapes share a common type.
	sphere = Marker()

	# Set the frame ID and type.  The frame ID is the frame in which the position of the marker
	# is specified.  The type is the shape of the marker, detailed on the wiki page.
	sphere.header.frame_id = '/base_link'
	sphere.type = sphere.SPHERE

	# Each marker has a unique ID number.  If you have more than one marker that you want displayed at a
	# given time, then each needs to have a unique ID number.  If you publish a new marker with the same
	# ID number and an existing marker, it will replace the existing marker with that ID number.
	sphere.id = 0

	# Set the action.  We can add, delete, or modify markers.
	sphere.action = sphere.ADD

	# These are the size parameters for the marker.  The effect of these on the marker will vary by shape,
	# but, basically, they specify how big the marker along each of the axes of the coordinate frame named
	# in frame_id.
	sphere.scale.x = 0.5
	sphere.scale.y = 0.5
	sphere.scale.z = 0.5

	# Color, as an RGB triple, from 0 to 1.
	sphere.color.r = 1.0
	sphere.color.g = 0.0
	sphere.color.b = 0.0

	# Alpha value, from 0 (invisible) to 1 (opaque).  If you don't set this and it defaults to zero, then
	# your marker will be invisible.  This is often hard to debug.
	sphere.color.a = 1.0

	# Specify the pose of the marker.  Since spheres are rotationally invarient, we're only going to specify
	# the positional elements.  As usual, these are in the coordinate frame named in frame_id.  Every time the
	# marker is displayed in rviz, ROS will use tf to determine where the marker should appear in the scene.
	# in this case, the position will always be directly above the robot, and will move with it.
	sphere.pose.position.x = 0.0
	sphere.pose.position.y = 0.0
	sphere.pose.position.z = 2.0

	# Set up a publisher.  We're going to publish on a topic called balloon.
	publisher = rospy.Publisher('balloon', Marker, queue_size=10)

	# Set a rate.  10 Hz is a good default rate for a marker moving with the Fetch robot.
	rate = rospy.Rate(10)

	# Publish the marker at 10Hz.
	while not rospy.is_shutdown():
		publisher.publish(sphere)
		rate.sleep()