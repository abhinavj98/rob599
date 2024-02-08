#!/usr/bin/env python3
import rospy
import sys
from sensor_msgs.msg import LaserScan
import numpy as np
from typing import List
import copy
from sklearn.linear_model import LinearRegression
from visualization_msgs.msg import Marker, Point


class DetectWall:
    def __init__(self):
        #subscribe to filtered scan
        self.laser_scan_subscriber = rospy.Subscriber('base_scan_filtered', LaserScan, self.laser_scan_callback)
        self.marker_pub = rospy.Publisher('wall_marker', Marker, queue_size=10)
        
        self.model = None

    def laser_scan_callback(self, msg):
        x, y = self.polar_to_cartesian(msg.ranges, msg.angle_min,
                                            msg.angle_max, msg.angle_increment)
        
        X = x.reshape(-1, 1)  # Reshape x to a column vector for scikit-learn

        # Fit a line using scikit-learn's LinearRegression
        self.model = LinearRegression()
        self.model.fit(X, y)
        self.visualize_fit(X)
    
    def visualize_fit(self, X):
        y_pred = self.model.predict(X)
        # Publish the original points as markers
        marker = Marker()
        marker.header.frame_id = 'laser_link'
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.color.r = 1.0
        marker.color.a = 1.0

        for i in range(len(X)):
            point = Point()
            point.x = x[i]
            point.y = y_pred[i]
            point.z = 0.0
            marker.points.append(point)

        self.marker_pub.publish(marker)
        
    @staticmethod
    def polar_to_cartesian(ranges: List[float], angle_min: float, angle_max: float, angle_increment: float):
        angle = np.arange(angle_min, angle_max, angle_increment)
        x = ranges * np.cos(angle)
        y = ranges * np.sin(angle)
        return x, y