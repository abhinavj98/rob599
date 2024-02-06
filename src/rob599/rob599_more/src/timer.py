#!/usr/bin/env python3


import rospy
import sys


def callback(event):
	rospy.loginfo('Callback fired')


if __name__ == '__main__':
	rospy.init_node('timer', argv=sys.argv)

	timer = rospy.Timer(rospy.Duration(1), callback)

	rospy.spin()
