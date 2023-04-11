#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CompressedImage, Image
from moveit_msgs.msg import DisplayTrajectory

from geometry_msgs.msg import TwistStamped

# Python 2/3 compatibility imports
import sys
import copy
import rospy
import geometry_msgs.msg
import moveit_commander

import cv2
import numpy as np
import time

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String

# Things to do:
# Movement
# Music coordinate
    # Slow beat music
# Framework

# Maybe try this for testing for integration in exisiting code.


reset  = [0, 0, 0, 0]
lean_forward = [40, 0, 0, 0]
lean_back = [-40, 0, 0, 0]
look_left = [0, -40, 0, 0]
look_right = [0, 40, 0, 0]
look_up = [0, 0, 0, -40]
look_down = [0, 0, 0, 40]
tilt_left = [0, 0, -40, 0]
tilt_right = [0, 0, 40, 0]

routine = [        
    {"time":0, "positions": [reset,reset,reset,reset]},
    {"time":10, "positions": [lean_forward, lean_forward, lean_back, lean_back]},
    {"time":20, "positions": [lean_back, lean_back, lean_forward, lean_forward]}]

class MultiRobotBehavior(object):
    """
    Multi Robot Behavior class.
    """
    def __init__(self):
        self.publishers = []
        for i in range(4):
            topic_name = f"/sb_{i}_cmd_state"
            self.publishers.append(rospy.Publisher(
                topic_name, TwistStamped, queue_size=10))
        rate = rospy.Rate(10)  # 10hz

        self.twist = TwistStamped()

    def execute_action(self, time_inter, positions):
        for num in range(len(positions)):
            move = positions[num]
            self.twist.twist.linear.x = move[0]
            self.twist.twist.linear.y = move[1]
            self.twist.twist.linear.z = move[2]
            self.twist.twist.angular.x = move[3]

            self.publishers[num].publish(self.twist)
        

        time.sleep(time_inter)

    def sb_callback(self):
        prev_time = 0
        
        for step in routine:
            step_time = step["time"]
            step_position = step["positions"]
            if step_time == 0:
                time_elapsed = 10
                prev_time = step_time
            self.execute_action(time_elapsed, step_position)
    	
if __name__ == '__main__':
    rospy.init_node("lab_4_node")
    
    ##############################
    # YOUR CODE HERE             #
    # Call the behavior(s)       #
    ##############################
    
    multi = MultiRobotBehavior()
    while not rospy.is_shutdown():
        multi.sb_callback()
    #while not rospy.is_shutdown():
    #    multi.sb_callback()

    rospy.spin()