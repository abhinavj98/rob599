#!/usr/bin/env python3


import rospy
import sys

from std_msgs.msg import String


if __name__ == '__main__':
	rospy.init_node('keyboard', argv=sys.argv)

	pub = rospy.Publisher('keyboard', String, queue_size=10)

	while not rospy.is_shutdown():
		text = input('> ')

		pub.publish(text)
