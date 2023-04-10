#!/usr/bin/env python3
from blissful_basics import singleton, LazyDict, Warnings, FS
Warnings.disable() # all python warnings
from math import pi, tau, dist, fabs, cos
from moveit_msgs.msg import DisplayTrajectory
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import std_msgs
import cv2
import rospy
import sys
import copy
import rospy
import geometry_msgs.msg
import moveit_commander
from blissful_basics import singleton, LazyDict, Warnings
import numbers
import numpy
import numpy as np
import math
from time import sleep
import rospy
from geometry_msgs.msg import TwistStamped
from statistics import median, stdev

from tools import bb, Event, JointPositions, EmotionalPositionGenerators, get_faces, convert_args, time_since_prev, clip_value, now, convert_routine_to_function
from image_tools import Image

should_log = bb.countdown(20) # 20 = every 20 times its called return true
config = LazyDict(
    send_to_rviz=False,
    enable_simulation=True,
)

@singleton
class Robot:
    status = LazyDict(
    )
    percepts = LazyDict(
    )
    
    def __init__(self):
        Robot = self.__class__
        # rospy.init_node("lab_2_node")
        rospy.init_node('test_joint', anonymous=True)
        
        self.joint_publisher = rospy.Publisher(
            "/sb_cmd_state",
            TwistStamped,
            queue_size=20
        )
        self.face_publisher = rospy.Publisher(
            "/face/do_something",
            std_msgs.msg.String,
            queue_size=5,
        )
        if config.send_to_rviz:
            self.movement_publisher = rospy.Publisher(
                "/move_group/display_planned_path",
                DisplayTrajectory,
                queue_size=20
            )
        rospy.loginfo("Node started.")

        self.twist_obj = TwistStamped()
        Robot.previous_joint_positions = JointPositions([0,0,0,0])
        if config.send_to_rviz:
            self.robot = moveit_commander.RobotCommander()
            self.scene = moveit_commander.PlanningSceneInterface()

            self.group_name = "survivor_buddy_head"
            self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        
            Robot.previous_joint_positions = JointPositions(self.move_group.get_current_joint_values())
            self.display_trajectory = DisplayTrajectory()
        
        Robot.previous_joint_positions = JointPositions([0,0,0,0])
    
    def tell_face(self, data):
        import json
        self.face_publisher.publish(
            std_msgs.msg.String(
                json.dumps(data)
            )
        )
    
    torso_change_limiter = 45 # degrees 
    def move_towards_positions(self, joint_goals, *, wait=False):
        torso_change = Robot.previous_joint_positions.torso_joint - joint_goals.torso_joint
        if abs(torso_change) > Robot.torso_change_limiter:
            if torso_change > 0:
                joint_goals.torso_joint = Robot.previous_joint_positions.torso_joint - Robot.torso_change_limiter
            else:
                joint_goals.torso_joint = Robot.previous_joint_positions.torso_joint + Robot.torso_change_limiter
        
        if not config.send_to_rviz:
            self.twist_obj.twist.linear.x  = -joint_goals[0]
            self.twist_obj.twist.linear.y  = -joint_goals[1]
            self.twist_obj.twist.linear.z  =  joint_goals[2]
            self.twist_obj.twist.angular.x = -joint_goals[3]
            self.joint_publisher.publish(self.twist_obj)
            Robot.previous_joint_positions = joint_goals
        else:
            joint_current = self.move_group.get_current_joint_values()
            
            for index, each in enumerate(joint_goals):
                joint_current[index] = each
            
            joint_current = tuple(joint_current)
            print(f'''joint_current = {JointPositions(joint_current)}''')
            # this gets rid of standing-still "jitter"
            if all(prev == current for prev, current in zip(Robot.previous_joint_positions, joint_current)):
                return
            else:
                Robot.previous_joint_positions = JointPositions(joint_current)
                print(f'''Robot.previous_joint_positions = {Robot.previous_joint_positions}''')
            
            self.move_group.go(joint_current, wait=wait)
            plan = self.move_group.plan()
            self.move_group.stop()
            
            if config.enable_simulation: # its a lot faster/smoother movement when not enabling simulation
                self.display_trajectory = DisplayTrajectory()
                self.display_trajectory.trajectory_start = self.robot.get_current_state()
                self.movement_publisher.publish(self.display_trajectory)

            # execute plan
            self.move_group.execute(plan[1], wait=wait)
    
    @property
    def previous_joint_positions(self):
        return self._previous_joint_positions
    
    @previous_joint_positions.setter
    def previous_joint_positions(self, value):
        self._previous_joint_positions = [ round(each, ndigits=3) for each in value ]
        
# 
# 
# Percepts
# 
# 
if True:
    # 
    # audio percept (used as a time percept)
    # 
    Robot.percepts.audio_event = Event()
    audio_subscriber = rospy.Subscriber(
        "/audio",
        Float32MultiArray,
        callback=lambda *args, **kwargs: Event.trigger(Robot.percepts.audio_event, *args, **kwargs),
        queue_size=1
    )
    
# 
# 
# Behaviors
# 
# 
if True:
    @singleton
    class Dance:
        @Event.when(Robot.percepts.audio_event)
        def when_new_timestep(data):
            Robot.tell_face(dict(sendMessage=f"found gesture: {gesture}"))
            
            joint_goals = JointPositions()
            joint_goals.torso_joint = 0
            joint_goals.neck_swivel = 0
            joint_goals.head_tilt   = 0
            joint_goals.head_nod    = 0
            Robot.move_towards_positions(joint_goals, wait=False):
            
            
    
if __name__ == '__main__':
    # 
    # load arguments
    # 
    # grab all the kwargs, put them in config
    args, kwargs = convert_args(sys.argv)
    for each_key, each_value in kwargs.items():
        each_key = each_key.replace("--","")
        config[each_key] = each_value
    
    
    moveit_commander.roscpp_initialize(sys.argv)
    ##############################
    # YOUR CODE HERE             #
    # Call the behavior(s)       #
    ##############################
    rospy.spin()