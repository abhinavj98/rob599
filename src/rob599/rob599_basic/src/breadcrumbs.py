#!/usr/bin/env python

# An example of using markers and tf to drop a set of breadcrumbs.
#
# breadcrubs.py
#
# Bill Smart
#
# This example shows how we might build a simple breadcrumb system for the Fetch robot.  It shows
# how we can autmatically publish a set of breadcrumbs showing where the robot has been.  The code
# for this is encapsulated into a class, so that it can be called easily from the main program loop.

# Import some of the packages that we need.
import rospy
import sys
import tf

# Import visualization markers and stamped poses.
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped


class Breadcrumb:
	"""
	A class that encapsultes a basic breadcrumb functionality for a mobile robot.
	"""
	def __init__(self, frame_id='map'):
		"""
		Initialize the class.

		:param frame_id: The frame in which the breadcrumb is placed.
		"""
		# Set up the frame ID, and make a publisher.  We're going to publish a MarkerArray, which is
		# a slightly more efficient way of publishing a set of markers.
		self.frame_id = frame_id
		self.publisher = rospy.Publisher('breadcrumbs', MarkerArray, queue_size=10)

		# Make a marker array to store the markers in.
		self.breadcrumbs = MarkerArray()

		# Create a tf listener, so that we can interact with tf.
		self.listener = tf.TransformListener()

		# Each marker will need a unique ID, so keep track of the IDs that we've issued.
		self.id = 0

	def drop(self, frame_id='base_link'):
		"""
		Drop a breadcrumb at the current location of the robot.

		:param frame_id: The frame in which to place the breadcrumb.
		"""

		# Make a pose in the base frame.
		pose = PoseStamped()
		pose.header.frame_id = frame_id
		pose.header.stamp = rospy.Time()
		pose.pose.position.x = 0.0
		pose.pose.position.y = 0.0
		pose.pose.position.z = 2.0

		# Transform it to the map frame.  Put this in a try-except block to catch problems.  If we can do the
		# transform, then we make a marker.  If not, then we fail silently.
		try:
			pose = self.listener.transformPose('map', pose)

			# Make a marker.
			crumb = Marker()

			# Each of the markers has to have a unique ID number.
			crumb.id = self.id
			self.id += 1

			# Set the frame ID, type.
			crumb.header.frame_id = 'map'
			crumb.header.stamp = rospy.Time()
			crumb.type = crumb.SPHERE

			crumb.action = crumb.ADD

			crumb.scale.x = 0.1
			crumb.scale.y = 0.1
			crumb.scale.z = 0.1

			crumb.color.r = 0.0
			crumb.color.g = 1.0
			crumb.color.b = 0.0
			crumb.color.a = 1.0

			# Copy the transformed pose.  This is already in the map frame, after the TransformPose call.
			crumb.pose = pose.pose

			# Add the marker to the marker array, and publish it out.
			self.breadcrumbs.markers.append(crumb)
			self.publisher.publish(self.breadcrumbs)
		except:
			pass


if __name__ == '__main__':
	# Initialize the node, as usual.
	rospy.init_node('breadcrumbs')

	# Create an instance of the breadcrumb class.
	crumbs = Breadcrumb()

	# Loop at 10 Hz, dropping breadcrumbs on each loop.
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		crumbs.drop()

		rate.sleep()

