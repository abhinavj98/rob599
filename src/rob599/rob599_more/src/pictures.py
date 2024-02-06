#!/usr/bin/env python


import rospy
import sys

import cv2

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError


class ImageTransformer:
	def __init__(self):
		self.bridge = CvBridge()
	
		self.sub = rospy.Subscriber('usb_cam/image_raw', Image, self.callback, queue_size=1)
		self.pub = rospy.Publisher('picture', Image, queue_size=1)

	def callback(self, msg):
		try:
			image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
		except CvBridgeError, e:
			rospy.logwarn('CV Bridge error: {0}'.format(e))

		image = cv2.Canny(image, 100, 200)

		image_msg = self.bridge.cv2_to_imgmsg(image, 'passthrough')
		image_msg.header = msg.header
		self.pub.publish(image_msg)


if __name__ == '__main__':
	rospy.init_node('pictures', argv=sys.argv)

	transformer = ImageTransformer()

	rospy.spin()

