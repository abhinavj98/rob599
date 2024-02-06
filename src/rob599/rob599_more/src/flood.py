#!/usr/bin/env python3


import rospy
import sys

from std_msgs.msg import Int64


if __name__ == '__main__':
	rospy.init_node('flood', argv =sys.argv)

	pub = rospy.Publisher('flood', Int64, queue_size=1)

	counter = 0
	while not rospy.is_shutdown():
		pub.publish(counter)
		counter += 1

	print('\nSent {0} messages'.format(counter))