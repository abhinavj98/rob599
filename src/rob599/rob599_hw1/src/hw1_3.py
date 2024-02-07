#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Quaternion, Twist
from sensor_msgs.msg import LaserScan
from typing import List
import numpy as np


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

    def forward(self, ranges: List[float]):
        if min(ranges) < 1:
            self.stop()
        else:
            min_dist = np.min(ranges)
            velocity = 0.5*(min_dist - 1)
            #clip velocity to 0 and 0.5
            velocity = max(0, min(0.5, velocity))
            self.publish_vel([velocity, 0])

class FetchMove:
    """Move fetch with velocity commands"""
    def __init__(self):
        self.laser_scan_subscriber = rospy.Subscriber('base_scan', LaserScan, self.laser_scan_callback)
        self.vel_controller = VelocityController()
        self.size = 1
    
    def laser_scan_callback(self, msg):
        ranges_cartesian = self.polar_to_cartesian(msg.ranges, msg.angle_min, msg.angle_max, msg.angle_increment)


    def polar_to_cartesian(self, ranges: List[float], angle_min: float, angle_max: float, angle_increment: float):
        angle = np.arange(angle_min, angle_max, angle_increment)
        x = ranges * np.cos(angle)
        y = ranges * np.sin(angle)
        return x, y

    def is_point_in_front(self, range: float):
        fetch_points = [0.5, 0.5]#get transform from world to laser
        #Will need robot orientation to get the correct min and max

        x_min = fetch_points[0] - 0.5
        x_max = fetch_points[0] + 0.5
        y_min = fetch_points[1]
        y_max = fetch_points[1] + 1
        x = range[0]
        y = range[1]
        return x_min < x < x_max and y_min < y < y_max



if __name__ == '__main__':
    rospy.init_node('vel_publisher')
    vel_publisher = FetchMove()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        vel_publisher.publish_vel()
        rate.sleep()

        
