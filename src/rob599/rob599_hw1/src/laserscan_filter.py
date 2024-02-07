#!/usr/bin/env python3
import rospy
import sys
from sensor_msgs.msg import LaserScan
import numpy as np
from typing import List
import copy

class LaserScanFilter:
    """Take in raw laser scan and filter to points just in front of fetch"""
    def __init__(self):
        self.laser_scan_subscriber = rospy.Subscriber('base_scan', LaserScan, self.filter_callback)
        self.laser_scan_publisher = rospy.Publisher('base_scan_filtered', LaserScan, queue_size = 10)
        
    def filter_callback(self, msg):
        filtered_msg = copy.deepcopy(msg)
        ranges_cartesian = self.polar_to_cartesian(msg.ranges, msg.angle_min, msg.angle_max, msg.angle_increment)
        polar_filtered_ranges = list(map(self.is_point_in_front, ranges_cartesian))
        filtered_msg.ranges = polar_filtered_ranges
        self.laser_scan_publisher.publish(filtered_msg)
        
    @staticmethod
    def polar_to_cartesian(ranges: List[float], angle_min: float, angle_max: float, angle_increment: float):
        angle = np.arange(angle_min, angle_max, angle_increment)
        x = ranges * np.cos(angle)
        y = ranges * np.sin(angle)
        return list(zip(x, y))


    def is_point_in_front(self, range):
        fetch_points = [0., 0.] #Laser link coordinates

        x_min = fetch_points[0] 
        x_max = fetch_points[0] + 1
        y_min = fetch_points[1] - 0.5
        y_max = fetch_points[1] + 0.5
        x = range[0]
        y = range[1]
        if x_min < x < x_max and y_min < y < y_max:
            return np.sqrt(np.square(x)+np.square(y))
        else:
            return np.inf

if __name__ == '__main__':
    # Initialize the node.
    rospy.init_node('laserscan_filter')
    laserscan_filter = LaserScanFilter()
    rospy.spin()
