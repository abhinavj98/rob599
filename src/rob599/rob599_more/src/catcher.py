#!/usr/bin/env python3


import rospy
import sys

from std_msgs.msg import Int64


class Counter:
	def __init__(self):
		self.count = 0
		sub = rospy.Subscriber('flood', Int64, self.callback, queue_size=1)

	def callback(self, msg):
		self.count += 1


if __name__ == '__main__':
	rospy.init_node('catcher', argv=sys.argv)

	counter = Counter()

	rospy.spin()

	print('\nReceived {0} mesages'.format(counter.count))