#!/usr/bin/env python3


import rospy
import sys

from std_msgs.msg import Int64

class PeriodicRepublisher:
	def __init__(self, topic, message_type, republish_topic, period):
		self.last_msg = None
		self.sub = rospy.Subscriber(topic, message_type, self.subscribe_callback, queue_size=1)
		self.pub = rospy.Publisher(republish_topic, message_type, queue_size=10)

		self.timer = rospy.Timer(period, self.publish_callback)

	def subscribe_callback(self, msg):
		self.last_msg = msg

	def publish_callback(self, event):
		if self.last_msg:
			self.pub.publish(self.last_msg)


if __name__ == '__main__':
	rospy.init_node('periodic_republish')

	republish = PeriodicRepublisher('flood', Int64, 'trickle', rospy.Duration(1))

	rospy.spin()