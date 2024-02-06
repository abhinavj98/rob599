#!/usr/bin/env python3


import rospy
import sys

from sensor_msgs.msg import Image

import message_filters


def callback(msg1, msg2):
	rospy.loginfo('Got a pair of messages')


if __name__ == '__main__':
	rospy.init_node('sync', argv=sys.argv)

	sub_1 = message_filters.Subscriber('usb_cam/image_raw', Image)
	sub_2 = message_filters.Subscriber('picture', Image)

	sync = message_filters.TimeSynchronizer([sub_1, sub_2], queue_size=1)
	sync.registerCallback(callback)

	rospy.spin()
