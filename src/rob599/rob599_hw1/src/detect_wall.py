#!/usr/bin/env python3
import rospy
import sys
from sensor_msgs.msg import LaserScan
import numpy as np
from typing import List
import copy
from sklearn.linear_model import LinearRegression
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
import tf


class DetectWall:
    def __init__(self):
        #subscribe to filtered scan
        self.laser_scan_subscriber = rospy.Subscriber('base_scan_filtered', LaserScan, self.laser_scan_callback)
        self.wall_angle_publisher = rospy.Publisher('wall_angle', Int32, queue_size=10)
        self.marker_pub = rospy.Publisher('wall_marker', Marker, queue_size=10)
        self.text_publisher = rospy.Publisher('wall_text', Marker, queue_size=10)
        
        self.model = None

    def laser_scan_callback(self, msg):
        x, y = self.polar_to_cartesian(msg.ranges, msg.angle_min,
                                            msg.angle_max, msg.angle_increment)
        
        X = x[abs(x)!=np.inf].reshape(-1, 1)
        Y = y[abs(x)!=np.inf].reshape(-1, 1)
        # Fit a line using scikit-learn's LinearRegression
        self.model = LinearRegression()
        self.model.fit(X, Y)
        slope = self.model.coef_[0]
        angle_radians = np.arctan(slope)
        self.wall_angle_publisher.publish(angle_radians)
        self.visualize_fit(X, angle_radians)
        
        
    def visualize_fit(self, X, angle):
        y_pred = [0,0]
        x_pred = [X[0], X[-1]]

        y_pred[0] = self.model.predict(X[0].reshape(-1,1))
        y_pred[1] = self.model.predict(X[-1].reshape(-1,1))
        # Publish the original points as markers
        marker = Marker()
        marker.header.frame_id = 'laser_link'
        marker.type = Marker.LINE_LIST 
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.color.r = 1.0
        marker.color.a = 1.0
        for i in range(2):
            point = Point()
            point.x = x_pred[i]
            point.y = y_pred[i]
            point.z = 0.0
            marker.points.append(point)

        self.marker_pub.publish(marker)

        marker = Marker()
        marker.header.frame_id = "laser_link"  # Change the frame ID if needed
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.scale.z = 0.1  # Text size

        marker.color.r = 1.0  # Red
        marker.color.g = 1.0  # Green
        marker.color.b = 1.0  # Blue
        marker.color.a = 1.0  # Alpha (transparency)

        marker.pose.position.x = 0.0  # Text position
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.0

        marker.text = str(angle)  # Text content

        self.text_publisher.publish(marker)

    @staticmethod
    def polar_to_cartesian(ranges: List[float], angle_min: float, angle_max: float, angle_increment: float):
        angle = np.arange(angle_min, angle_max, angle_increment)
        x = ranges * np.cos(angle)
        y = ranges * np.sin(angle)
        return x, y
    

if __name__ == "__main__":
    rospy.init_node('wall_detect')
    fetch = DetectWall()
    rospy.spin()
