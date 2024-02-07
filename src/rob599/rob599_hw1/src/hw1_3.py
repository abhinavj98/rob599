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

    def publish_vel(self, vel: List[float]):
        msg = Twist()
        msg.linear.x = vel[0]
        #msg.angular.z = vel[1]
        print(msg)
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
    
class FetchMove:
    """Move fetch with velocity commands"""
    def __init__(self):
        self.laser_scan_subscriber = rospy.Subscriber('base_scan_filtered', LaserScan, self.laser_scan_callback)
        self.vel_controller = VelocityController()
        self.size = 1
        
    def laser_scan_callback(self, msg):
        self.move(msg.ranges)
    
    def move(self, ranges):
        self.vel_controller.forward_polar(ranges=ranges)



if __name__ == '__main__':
    rospy.init_node('vel_publisher')
    fetch = FetchMove()
    rospy.spin()
    #rate = rospy.Rate(10)
    #while not rospy.is_shutdown():
     #   #fetch.move()
      #  rate.sleep()

        
