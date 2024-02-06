#!/usr/bin/env python3


import rospy
import sys


if __name__ == '__main__':
	rospy.init_node('parameters', argv=sys.argv)

	text = rospy.get_param('foo', 'Nothing set')
	rospy.loginfo('Text: {0}'.format(text))

	count = rospy.get_param('count', 0)
	count += 1
	rospy.set_param('count', count)
