#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Quaternion, Twist
from sensor_msgs.msg import LaserScan
from typing import List
import numpy as np
import actionlib
from rob599_hw1.srv import StoppingDist, StoppingDistResponse
from rob599_hw1.msg import StoppingAction, StoppingGoal, StoppingFeedback, StoppingResult

class VelocityController:
    """Velocity controller for the fetch robot"""
    def __init__(self):
        self.vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.set_stopping_dist_srv = rospy.Service('set_stopping_dist', StoppingDist, self.set_stopping_dist)
        self.stopping_distance = 1.
        self.max_velocity = 1.

    def set_stopping_dist(self, request):
        if request.dist < 0:
            return StoppingDistResponse(False)
        self.set_stopping_dist = request.dist
        return StoppingDistResponse(True) 
    
    def publish_vel(self, vel: List[float]):
        msg = Twist()
        msg.linear.x = vel[0]
        self.vel_publisher.publish(msg)

    def stop(self):
        self.publish_vel([0, 0])

    def forward_polar(self, ranges: List[float]):
        if min(ranges) < self.stopping_distance:
            self.stop()
        else:
            min_dist = np.min(ranges)
            velocity = (min_dist - self.stopping_distance)
            #clip velocity to 0 and 0.5
            velocity = max(0, min(self.max_velocity, velocity))
            self.publish_vel([velocity, 0])
        return min(ranges)
    
class FetchMove:
    """Move fetch with velocity commands"""
    def __init__(self):
        self.laser_scan_subscriber = rospy.Subscriber('base_scan_filtered', LaserScan, self.laser_scan_callback)
        self.vel_controller = VelocityController()
        self.action_server = actionlib.SimpleActionServer('move_to_wall', StoppingAction, self.action_callback, False)
        self.ranges = None
        self.size = 1
        
    def action_callback(self, goal):
        while True:
            min_dist = self.move(self.ranges)
            self.action_server.publish_feedback(StoppingFeedback(dist = min_dist))
            if self.action_server.is_new_goal_available():
                self.action_server.set_preempted(StoppingResult(success=False))
                return
            if goal >= min_dist:
                self.action_server.set_succeeded(StoppingResult(success=True))
                break


    def laser_scan_callback(self, msg):
        self.ranges = msg.ranges
    
    def move(self, ranges):
        return self.vel_controller.forward_polar(ranges=ranges)



if __name__ == '__main__':
    rospy.init_node('vel_publisher')
    fetch = FetchMove()
    rospy.spin()
    #rate = rospy.Rate(10)
    #while not rospy.is_shutdown():
     #   #fetch.move()
      #  rate.sleep()

        
