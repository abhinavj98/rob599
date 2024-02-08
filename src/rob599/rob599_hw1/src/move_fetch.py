#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Quaternion, Twist
from sensor_msgs.msg import LaserScan
from typing import List
import numpy as np
import actionlib

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from rob599_hw1.srv import StoppingDist, StoppingDistResponse
from rob599_hw1.msg import StoppingAction, StoppingGoal, StoppingFeedback, StoppingResult
import time
action_in_progress = True

class VelocityController:
    """Velocity controller for the fetch robot"""
    def __init__(self):
        self.vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.set_stopping_dist_srv = rospy.Service('set_stopping_dist', StoppingDist, self.set_stopping_dist)
        self.stopping_distance = 1.
        self.max_velocity = 1.

    def set_stopping_dist(self, request):
        global action_in_progress
        if action_in_progress:
            rospy.logwarn("Service calls are disabled.")
            return StoppingDistResponse(False)
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
        self.line_publisher = rospy.Publisher('visualization_line', Marker, queue_size=10)
        self.text_publisher = rospy.Publisher('visualization_text', Marker, queue_size=10)
        self.action_server = actionlib.SimpleActionServer('move_to_wall', StoppingAction, self.action_callback, False)
        self.ranges = None
        self.size = 1
        self.msg = None
        self.action_server.start()
        rospy.loginfo('action server started')
        
    def action_callback(self, goal):
        global action_in_progress
        action_in_progress = True
        while True:
            min_dist = self.move(self.ranges)
            self.rviz_publish_line()
            self.action_server.publish_feedback(StoppingFeedback(dist = min_dist))
            if self.action_server.is_new_goal_available():
                self.action_server.set_preempted(StoppingResult(success=False))
                return
            if goal.goal >= min_dist:
                self.action_server.set_succeeded(StoppingResult(success=True))
                action_in_progress = False
                break

            time.sleep(0.5)
    
    @staticmethod
    def polar_to_min_cartesian(ranges: List[float], angle_min: float, angle_max: float, angle_increment: float):
        angle = np.arange(angle_min, angle_max, angle_increment)
        min_range_index = ranges.index(min(ranges))
        x = ranges[min_range_index]*np.cos(angle[min_range_index])
        y = ranges[min_range_index]*np.sin(angle[min_range_index])
        return x, y, ranges[min_range_index]


    def rviz_publish_line(self):
        marker = Marker()
        marker.header.frame_id = "laser_link"  
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # Line width

        marker.color.r = 1.0  # Red
        marker.color.a = 1.0  # Alpha (transparency)

        # Define the points for the lines
        p1 = Point()
        p1.x = 0.0
        p1.y = 0.0
        p1.z = 0.0

        p2 = Point()
        min_x, min_y, min_range = self.polar_to_min_cartesian(self.msg.ranges, self.msg.angle_min, self.msg.angle_max, self.msg.angle_increment)
        p2.x = min_x
        p2.y = min_y
        p2.z = 0.0

        marker.points.append(p1)
        marker.points.append(p2)

        self.line_publisher.publish(marker)

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

        marker.text = str(min_range)  # Text content

        self.text_publisher.publish(marker)


    def laser_scan_callback(self, msg):
        self.ranges = msg.ranges
        self.msg = msg
    
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

        
