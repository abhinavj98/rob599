#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Quaternion, Twist
from sensor_msgs.msg import LaserScan
from typing import List
import numpy as np
import tf


class VelocityController:
    """Velocity controller for the fetch robot"""
    def __init__(self):
        self.vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    def publish_vel(self, vel: List[float, float]):
        msg = Twist()
        msg.linear.x = vel[0]
        msg.angular.z = vel[1]

        self.vel_publisher.publish(msg)

    def stop(self):
        self.publish_vel([0, 0])

    def forward_polar(self, ranges: List[float]):
        if min(ranges) < 1:
            self.stop()
        else:
            min_dist = np.min(ranges)
            velocity = 1*(min_dist - 1)
            #clip velocity to 0 and 0.5
            velocity = max(0, min(1, velocity))
            self.publish_vel([velocity, 0])
    
    def forward_cartesian(self, ranges):
        if ranges is None:
            self.stop()
        dist = np.linalg.norm(ranges)
        if min(ranges) < 1:
            self.stop()
        else:
            min_dist = np.min(dist)
            velocity = 1*(min_dist - 1)
            #clip velocity to 0 and 0.5
            velocity = max(0, min(1, velocity))
            self.publish_vel([velocity, 0])

class FetchMove:
    """Move fetch with velocity commands"""
    def __init__(self):
        self.laser_scan_subscriber = rospy.Subscriber('base_scan', LaserScan, self.laser_scan_callback)
        self.vel_controller = VelocityController()
        self.size = 1
        self.tf_listener = tf.TransformListener()

    # def get_fetch_pose(self):
    #     try:
    #         (trans,rot) = self.tf_listener.lookupTransform('/base_footprint', '/CameraTop_optical_frame', rospy.Time(0))
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         continue
    
    def laser_scan_callback(self, msg):
        ranges_cartesian = self.polar_to_cartesian(msg.ranges, msg.angle_min, msg.angle_max, msg.angle_increment)
        #filter ranges_cartesian if it is in front
        self.cartesian_in_front = filter(self.is_point_in_front, ranges_cartesian)

    def polar_to_cartesian(self, ranges: List[float], angle_min: float, angle_max: float, angle_increment: float):
        angle = np.arange(angle_min, angle_max, angle_increment)
        x = ranges * np.cos(angle)
        y = ranges * np.sin(angle)
        return x, y

    def is_point_in_front(self, range: float):
        fetch_points = [0., 0.]#get transform from world to laser
        #Will need robot orientation to get the correct min and max

        x_min = fetch_points[0] - 0.5
        x_max = fetch_points[0] + 0.5
        y_min = fetch_points[1]
        y_max = fetch_points[1] + 1
        x = range[0]
        y = range[1]
        return x_min < x < x_max and y_min < y < y_max
    
    def move(self):
        self.vel_controller.forward_cartesian(self.is_point_in_front)



if __name__ == '__main__':
    rospy.init_node('vel_publisher')
    fetch = FetchMove()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        fetch.move()
        rate.sleep()

        
