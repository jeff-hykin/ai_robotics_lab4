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

import math
import math

from playsound import playsound
from threading import Thread

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String

from blissful_basics import LazyDict

# Things to do:
# Movement
# Music coordinate
    # Slow beat music
# Framework

# Maybe try this for testing for integration in exisiting code.



def jointDictToVec(d):
    return np.array([d.torso_joint, d.neck_swivel, d.head_nod, d.head_tilt])
def jointVec(torso_joint, neck_swivel, head_nod, head_tilt):
    return np.array([torso_joint, neck_swivel, head_nod, head_tilt])
def jointVecToDict(v):
    if v is None:
        return None
    return dict(torso_joint=v[0]   , neck_swivel=v[1]   , head_nod=v[2]   , head_tilt=v[3]) 

reset        = jointVec(torso_joint=0   , neck_swivel=0   , head_tilt=0   , head_nod=0   ) 
lean_forward = jointVec(torso_joint=20, neck_swivel=0, head_tilt=0, head_nod=0)
lean_back    = jointVec(torso_joint=-20   , neck_swivel=0   , head_tilt=0   , head_nod=0   ) 
look_left    = jointVec(torso_joint=0   , neck_swivel=-40   , head_tilt=0   , head_nod=0   ) 
look_right   = jointVec(torso_joint=0   , neck_swivel=40   , head_tilt=0   , head_nod=0   ) 
look_up      = jointVec(torso_joint=0   , neck_swivel=0   , head_tilt=0   , head_nod=-40   ) 
look_down    = jointVec(torso_joint=0   , neck_swivel=0   , head_tilt=0   , head_nod=40   ) 
tilt_left    = jointVec(torso_joint=0   , neck_swivel=0   , head_tilt=-40   , head_nod=0   ) 
tilt_right   = jointVec(torso_joint=0   , neck_swivel=0   , head_tilt=40   , head_nod=0   ) 

# reset        = dict(torso_joint=0   , neck_swivel=0   , head_tilt=0   , head_nod=0   ) 
# lean_forward = dict(torso_joint=40  , neck_swivel=None, head_tilt=None, head_nod=None)
# lean_back    = dict(torso_joint=-40 , neck_swivel=None, head_tilt=None, head_nod=None) 
# look_left    = dict(torso_joint=None, neck_swivel=-40 , head_tilt=None, head_nod=None) 
# look_right   = dict(torso_joint=None, neck_swivel=40  , head_tilt=None, head_nod=None) 
# look_up      = dict(torso_joint=None, neck_swivel=None, head_tilt=None, head_nod=-40 ) 
# look_down    = dict(torso_joint=None, neck_swivel=None, head_tilt=None, head_nod=40  ) 
# tilt_left    = dict(torso_joint=None, neck_swivel=None, head_tilt=-40 , head_nod=None) 
# tilt_right   = dict(torso_joint=None, neck_swivel=None, head_tilt=40  , head_nod=None) 


def allDo(motion):
    return [motion, motion, motion, motion]

def delayed_sine_single(time, motion, period, start, end, delay):
    t = time - delay
    if t < start:
        t = 0
    if t > end:
        t = end
    val = math.sin(t * (2 * math.pi / period))
    return val * motion

def delayed_sine(time, motion, period, start, end, delay1, delay2, delay3, delay4):
    return [delayed_sine_single(time, motion, period, start, end, delay1), 
                delayed_sine_single(time, motion, period, start, end, delay2), 
                delayed_sine_single(time, motion, period, start, end, delay3), 
                delayed_sine_single(time, motion, period, start, end, delay4)]

def sine_single(time, motion, period, phase):
    t = time + phase
    val = math.sin(t * (2 * math.pi / period))
    return val * motion

# def sine(time, motion, period, phase1, phase2, phase3, phase4):
#     return [motion, motion, motion, motion]

# a half wave
def delayed_sine_there_and_back(time, motion, delay, speed):
    return delayed_sine(time=time, motion=motion, period=4/speed, start=0, end=2/speed, delay1=0, delay2=delay, delay3=delay*2, delay4=delay*3)

# a quarter wave
def delayed_sine_there(time, motion, delay, speed):
    return delayed_sine(time=time, motion=motion, period=4/speed, start=0, end=1/speed, delay1=0, delay2=delay, delay3=delay*2, delay4=delay*3)

# a full wave
# speed 1 takes 4 seconds, speed 2 takes 2s, etc.
def delayed_sine_full(time, motion, delay, speed):
    return delayed_sine(time=time, motion=motion, period=4/speed, start=0, end=4/speed, delay1=0, delay2=delay, delay3=delay*2, delay4=delay*3)

routine = [
    # time is in seconds
    { "time":     0, "positions": [                    reset,                    reset,                    reset,                    reset, ], },
    { "time":     1, "positions": [                    look_left,                    look_left,                    look_right,                    look_right, ], },
    { "time":     2, "positions": [                    look_right,                    look_right,                    look_left,                    look_left, ], },
    { "time":     3, "positions": [                    look_left,                    look_left,                    look_right,                    look_right, ], },
    { "time":     4, "positions": [                    look_right,                    look_right,                    look_left,                    look_left, ], },
    { "time":     5, "positions": [                    look_left,                    look_left,                    look_right,                    look_right, ], },
    { "time":     6, "positions": [                    look_right,                    look_right,                    look_left,                    look_left, ], },
    { "time":     7, "positions": [                    look_left,                    look_left,                    look_right,                    look_right, ], },
    { "time":     8, "positions": [                    look_right,                    look_right,                    look_left,                    look_left, ], },
    { "time":     9, "positions": [                    look_left,                    look_left,                    look_right,                    look_right, ], },
    { "time":     10, "positions": [                    look_right,                    look_right,                    look_left,                    look_left, ], },
    { "time":     11, "positions": [                    look_left,                    look_left,                    look_right,                    look_right, ], },
    { "time":     12, "positions": [                    look_right,                    look_right,                    look_left,                    look_left, ], },
    { "time":     13, "positions": [                    look_right,                    look_left,                    look_left,                    look_left, ], },
    { "time":     14, "positions": [                    look_left,                    look_left,                    look_left,                    look_left, ], },
    { "time":     15, "positions": [                    look_left,                    look_right,                    look_left,                    look_right, ], },
    { "time":     16, "positions": [                    look_right,                    look_left,                    look_right,                    look_left, ], },
    { "time":     17, "positions": [                    look_left,                    look_right,                    look_left,                    look_right, ], },
    { "time":     18, "positions": [                    look_right,                    look_left,                    look_right,                    look_left, ], },
    { "time":     19, "positions": [                    look_left,                    look_right,                    look_left,                    look_right, ], },
    { "time":     20, "positions": [                    look_right,                    look_left,                    look_right,                    look_left, ], },
    { "time":     21, "positions": [                    look_right,                    look_right,                    look_right,                    look_right, ], },
    { "time":     22, "positions": [                    look_left,                    look_right,                    look_right,                    look_left, ], },
    { "time":     23, "positions": [                    look_right,                    look_left,                    look_left,                    look_right, ], },
    { "time":     24, "positions": [                    look_left,                    look_right,                    look_right,                    look_left, ], },
    { "time":     25, "positions": [                    look_right,                    look_left,                    look_left,                    look_right, ], },
    { "time":     26, "positions": [                    look_left,                    look_right,                    look_right,                    look_left, ], },
    { "time":    27, "positions": [             1.0 * lean_forward,                     reset,                     None,                     None, ], },
    { "time":    28, "positions": [                     None,             1.0 * lean_forward,                     reset,                     None, ], },
    { "time":    29, "positions": [                     None,                     None,             1.0 * lean_forward,                     reset, ], },
    { "time":    30, "positions": [                     None,                     None,                     None,             1.0 * lean_forward, ], },
    { "time":    33, "positions": allDo( lean_forward ) },
    { "time":    34, "positions": allDo( reset ) },
    { "time":    36, "positions": allDo( tilt_left + lean_back ) },
    { "time":    38, "positions": allDo( tilt_right + lean_back ) },
    { "time":    40, "positions": allDo( reset ) },
    { "time":    41, "positions": allDo( reset ) },
]
routine.extend([
    { "time":    41.00 + t, "positions": delayed_sine_there_and_back(time=t, motion=lean_back, delay=0.25, speed=1) } 
    for t in np.arange(0.25, 2+4*0.25, 0.25)])
routine.extend([
    { "time":    45.00, "positions": allDo( reset ) },
])
routine.extend([
    { "time":    45.00 + t, "positions": delayed_sine_there(time=t, motion=look_left, delay=0.25, speed=2) } 
    for t in np.arange(0.125, 1+4*0.125, 0.125)])
routine.extend([
    { "time":    45.50, "positions": allDo( reset ) },
])
routine.extend([
    { "time":    45.50 + t, "positions": delayed_sine_there(time=t, motion=look_right, delay=0.25, speed=2) } 
    for t in np.arange(0.125, 1+4*0.125, 0.125)])
routine.extend([
    { "time":    47.00, "positions": allDo( reset ) },
])
routine.extend([
    { "time":    47.00 + t, "positions": delayed_sine_full(time=t, motion=lean_forward, delay=0.25, speed=1) } 
    for t in np.arange(0.25, 4+4*0.25, 0.25)])
routine.extend([
    { "time":    52.00, "positions": allDo( reset ) },
])

routine.extend([
    { "time":    52.50, "positions": allDo( look_up ) },
])
routine.extend([
    { "time":    52.75, "positions": allDo( look_down ) },
])
routine.extend([
    { "time":    53.00, "positions": allDo( look_up ) },
])
routine.extend([
    { "time":    53.25, "positions": allDo( look_down ) },
])
routine.extend([
    { "time":    53.50, "positions": allDo( look_up ) },
])
routine.extend([
    { "time":    53.75, "positions": allDo( look_down ) },
])

routine.extend([
    { "time":    53.75 + t, "positions": delayed_sine_there_and_back(time=t, motion=lean_back, delay=0.25, speed=1) } 
    for t in np.arange(0.25, 2+4*0.25, 0.25)])
routine.extend([
    { "time":    57.75, "positions": allDo( reset ) },
])
routine.extend([
    { "time":    57.75 + t, "positions": delayed_sine_there(time=t, motion=look_left, delay=0.25, speed=2) } 
    for t in np.arange(0.125, 1+4*0.125, 0.125)])
routine.extend([
    { "time":    58.25, "positions": allDo( reset ) },
])
routine.extend([
    { "time":    58.25 + t, "positions": delayed_sine_there(time=t, motion=look_right, delay=0.25, speed=2) } 
    for t in np.arange(0.125, 1+4*0.125, 0.125)])
routine.extend([
    { "time":    59.75, "positions": allDo( reset ) },
])
routine.extend([
    { "time":    64.50 + t, "positions": delayed_sine_full(time=t, motion=lean_forward, delay=0.25, speed=1) } 
    for t in np.arange(0.25, 4+4*0.25, 0.25)])

routine.extend([
    { "time":    65.25, "positions": allDo( lean_forward)},]) 
routine.extend([
    { "time":    66.00, "positions": allDo( lean_back )},])
routine.extend([
    { "time":    67.00, "positions": allDo( lean_forward)},]) 
routine.extend([
    { "time":    68.00, "positions": allDo( lean_back)},])
routine.extend([
    { "time":    69.00, "positions": allDo( look_up )},])
routine.extend([
    { "time":    70.00, "positions": allDo( look_down )},])
routine.extend([
    { "time":    71.00, "positions": allDo( look_up )},])
routine.extend([
    { "time":    72.00, "positions": allDo( look_down )},])
routine.extend([
    { "time":    73.00, "positions": allDo( look_up )},])
routine.extend([
    { "time":    74.00, "positions": allDo( look_down )},])
routine.extend([
    { "time":    75.00, "positions": allDo( lean_forward )},]) 
routine.extend([
    { "time":    76.00, "positions": allDo( lean_back )},])
routine.extend([
    { "time":    78.00, "positions": allDo( lean_forward )},]) 
routine.extend([
    { "time":    79.00, "positions": allDo( lean_back )},])
routine.extend([
    { "time":    80.00, "positions": allDo( look_up )},])
routine.extend([
    { "time":    80.25, "positions": allDo( look_down )},])
routine.extend([
    { "time":    80.50, "positions": allDo( look_up )},])
routine.extend([
    { "time":    80.75, "positions": allDo( look_down )},])
routine.extend([
    { "time":    81.00, "positions": allDo( look_up )},])
routine.extend([
    { "time":    81.25, "positions": allDo( look_down )},])
routine.extend([
    { "time":    82.00, "positions": [look_left, look_left, look_right, look_right]},])
routine.extend([
    { "time":    82.00, "positions": [look_right, look_right, look_left, look_left]},])
routine.extend([
    { "time":    83.00, "positions": [look_left, look_left, look_right, look_right]},])
routine.extend([
    { "time":    84.00, "positions": [look_right, look_right, look_left, look_left]},])
routine.extend([
    { "time":    85.00, "positions": [look_left, look_left, look_right, look_right]},])
routine.extend([
    { "time":    86.00, "positions": [look_right, look_right, look_left, look_left]},])
routine.extend([
    { "time":    87.00, "positions": [look_left, look_left, look_right, look_right]},])
routine.extend([
    { "time":    88.00, "positions": [look_right, look_right, look_left, look_left]},])
routine.extend([
    { "time":    89.00, "positions": [look_left, look_left, look_right, look_right]},])
routine.extend([
    { "time":    90.00, "positions": [look_right, look_right, look_left, look_left]},])
routine.extend([
    { "time":    91.00, "positions": [look_left, look_left, look_right, look_right]},])
routine.extend([
    { "time":    92.00, "positions": [look_right, look_right, look_left, look_left]},])
routine.extend([
    { "time":    93.00, "positions": [look_left, look_left, look_right, look_right]},])
routine.extend([
    { "time":    94.00, "positions": [look_right, look_right, look_left, look_left]},])
routine.extend([
    { "time":    95.00, "positions": [look_left, look_left, look_right, look_right]},])
routine.extend([
    { "time":    96.00, "positions": [look_right, look_left, look_right, look_left]},])
routine.extend([
    { "time":    97.00, "positions": [look_left, look_right, look_left, look_right]},])
routine.extend([
    { "time":    98.00, "positions": [look_right, look_left, look_right, look_left]},])
routine.extend([
    { "time":    99.00, "positions": [look_left, look_right, look_left, look_right]},])
routine.extend([
    { "time":    100.00, "positions": [look_left, look_right, look_right, look_left]},])
routine.extend([
    { "time":    101.00, "positions": [look_right, look_left, look_left, look_right]},])
routine.extend([
    { "time":    102.00, "positions": [look_left, look_right, look_right, look_left]},])
routine.extend([
    { "time":    103.00, "positions": [look_right, look_left, look_left, look_right]},])
routine.extend([
    { "time":    104.00, "positions": [look_left, look_right, look_right, look_left]},])
routine.extend([
    { "time":    105.00, "positions": [look_right, look_left, look_left, look_right]},])
routine.extend([
    { "time":    106.00, "positions": [look_left, look_right, look_right, look_left]},])
routine.extend([
    { "time":    107.00, "positions": [look_right, look_left, look_left, look_right]},])
routine.extend([
    { "time":    107.25, "positions": allDo(look_down)},])
routine.extend([
    { "time":    107.50, "positions": allDo(look_up)},])
routine.extend([
    { "time":    107.75, "positions": allDo(look_down)},])
routine.extend([
    { "time":    108.00, "positions": allDo(look_up)},])

routine.extend([
    { "time":    108.75 + t, "positions": delayed_sine_there_and_back(time=t, motion=lean_back, delay=0.25, speed=1) } 
    for t in np.arange(0.25, 2+4*0.25, 0.25)])
routine.extend([
    { "time":    112.75, "positions": allDo( reset ) },
])
routine.extend([
    { "time":    112.75 + t, "positions": delayed_sine_there(time=t, motion=look_left, delay=0.25, speed=2) } 
    for t in np.arange(0.125, 1+4*0.125, 0.125)])
routine.extend([
    { "time":    113.25, "positions": allDo( reset ) },
])
routine.extend([
    { "time":    113.25 + t, "positions": delayed_sine_there(time=t, motion=look_right, delay=0.25, speed=2) } 
    for t in np.arange(0.125, 1+4*0.125, 0.125)])
routine.extend([
    { "time":    114.75, "positions": allDo( reset ) },
])
routine.extend([
    { "time":    114.75 + t, "positions": delayed_sine_full(time=t, motion=lean_forward, delay=0.25, speed=1) } 
    for t in np.arange(0.25, 4+4*0.25, 0.25)])

routine.extend([
    { "time":    115.00, "positions": allDo(look_up)},])
routine.extend([
    { "time":    116.00, "positions": allDo(look_down)},])
routine.extend([
    { "time":    117.00, "positions": allDo(look_up)},])
routine.extend([
    { "time":    118.00, "positions": allDo(look_down)},])
routine.extend([
    { "time":    119.00, "positions": allDo(look_up)},])
routine.extend([
    { "time":    120.00, "positions": allDo(look_down)},])
routine.extend([
    { "time":    121.00, "positions": allDo(look_up)},])
routine.extend([
    { "time":    121.25, "positions": allDo(look_down)},])
routine.extend([
    { "time":    121.50, "positions": allDo(look_up)},])
routine.extend([
    { "time":    121.75, "positions": allDo(look_down)},])
routine.extend([
    { "time":    122.00, "positions": allDo(look_up)},])
routine.extend([
    { "time":    122.25, "positions": allDo(reset)},])




routine.extend([
    { "time":    1000, "positions": [                    reset,                    reset,                    reset,                    reset, ], },
])

# translate back from numpy arrays to dicts for rest of jeffs stuff
for i in range(len(routine)):
    positions = routine[i]["positions"]
    positions_dicts = [jointVecToDict(pos) for pos in positions]
    routine[i]["positions"] = positions_dicts


# routine = [
#     # time is in seconds
#     { "time":     0, "positions": [                    reset,                    reset,                    reset,                    reset, ], },
#     { "time":    10, "positions": [             lean_forward,                    reset,                     None,                     None, ], },
#     { "time":    11, "positions": [                     None,             lean_forward,                    reset,                     None, ], },
#     { "time":    12, "positions": [                     None,                     None,             lean_forward,                    reset, ], },
#     { "time":    13, "positions": [                     None,                     None,                     None,             lean_forward, ], },
#     { "time":    16, "positions": [             lean_forward,             lean_forward,             lean_forward,             lean_forward, ], },
#     { "time":    16, "positions": [                    reset,                    reset,                    reset,                    reset, ], },
#     { "time":    18, "positions": [ [ tilt_left, lean_back ], [ tilt_left, lean_back ], [ tilt_left, lean_back ], [ tilt_left, lean_back ], ], },
#     { "time":  1000, "positions": [                    reset,                    reset,                    reset,                    reset, ], },
# ]

def shift_towards(*, new_value, old_value, proportion):
    if proportion == 1:
        return new_value
    if proportion == 0:
        return old_value
    
    difference = new_value - old_value
    amount = difference * proportion
    return old_value+amount

def convert_routine_to_function(routine):
    '''
    Example:
        reset        = dict(torso_joint=0   , neck_swivel=0   , head_tilt=0   , head_nod=0   ) 
        lean_forward = dict(torso_joint=1   , neck_swivel=2   , head_tilt=3   , head_nod=4   )
        lean_back    = dict(torso_joint=5   , neck_swivel=6   , head_tilt=7   , head_nod=8   )
        look_left    = dict(torso_joint=9   , neck_swivel=10   , head_tilt=11   , head_nod=12   )
        look_right   = dict(torso_joint=13   , neck_swivel=14   , head_tilt=15   , head_nod=16   )
        look_up      = dict(torso_joint=17   , neck_swivel=18   , head_tilt=19   , head_nod=20   )
        look_down    = dict(torso_joint=21   , neck_swivel=22   , head_tilt=23   , head_nod=24   )
        tilt_left    = dict(torso_joint=25   , neck_swivel=26   , head_tilt=27   , head_nod=28   )
        tilt_right   = dict(torso_joint=29   , neck_swivel=30   , head_tilt=31   , head_nod=32   )
        routine = [
            # time is in seconds
            { "time":     0, "positions": [                    reset,                    reset,                    reset,                    reset, ], },
            { "time":    10, "positions": [             lean_forward,                     None,                     None,                     None, ], },
            { "time":    11, "positions": [                     None,             lean_forward,                     None,                     None, ], },
            { "time":    12, "positions": [                     None,                     None,             lean_forward,                     None, ], },
            { "time":    13, "positions": [                     None,                     None,                     None,             lean_forward, ], },
            { "time":    16, "positions": [             lean_forward,                    reset,                    reset,                    reset, ], },
            { "time":    16, "positions": [                    reset,                    reset,                    reset,                    reset, ], },
            { "time":    18, "positions": [ [ tilt_left, lean_back ], [ tilt_left, lean_back ], [ tilt_left, lean_back ], [ tilt_left, lean_back ], ], },
            { "time":  1000, "positions": [                    reset,                    reset,                    reset,                    reset, ], },
        ]
        
        # create a converter function
        time_to_positions = convert_routine_to_function(routine)
        time_to_positions(11.21321) # returns the location of all the joints for all the bots at time 10
    '''
    if len(routine) == 0:
        raise Exception(f'''The convert_routine_to_function() function got a routine with a length of 0, there needs to be at least one frame in the routine''')
    
    # standarize the routine values (LazyDict, and flatten the keypoint lists so they're all dict or None)
    for index, each_timestep in enumerate(routine):
        routine[index] = LazyDict(each_timestep)
        new_positions_list = []
        for each_bot_keypoint in routine[index].positions:
            if isinstance(each_bot_keypoint, (dict, LazyDict, type(None))):
                keypoint = each_bot_keypoint
            elif not isinstance(each_bot_keypoint, list):
                raise Exception(f'''got an unknown value as a keypoint (expected list, dict, or None): {each_bot_keypoint}\nroutine index:{index}''')
            else: # must be a list
                keypoint = LazyDict()
                for each_keypoint in each_bot_keypoint:
                    # override any non-None keys
                    for each_key, each_value in each_keypoint.items():
                        if each_value != None:
                            keypoint[each_key] = each_value
            
            # convert to LazyDict
            if not isinstance(each_bot_keypoint, type(None)):
                keypoint = LazyDict(keypoint)
                
            new_positions_list.append(keypoint)
        routine[index].positions = new_positions_list
    
    # validate start/end
    for start_or_end_positions in [routine[0].positions, routine[-1].positions]:
        for each_bot in start_or_end_positions:
            # make sure start/end of routine specifies values for all joint positions
            assert each_bot.torso_joint != None
            assert each_bot.neck_swivel != None
            assert each_bot.head_tilt != None
            assert each_bot.head_nod != None
    
    # create a joint-centric representation of the data
    number_of_bots = len(routine[0].positions)
    bot_time_mappings = []
    for bot_index in range(number_of_bots):
        time_mappings = LazyDict(
            torso_joint=LazyDict(),
            neck_swivel=LazyDict(),
            head_tilt=LazyDict(),
            head_nod=LazyDict(),
        )
        for each_frame in routine:
            time_value = each_frame.time
            position_updates = each_frame.positions[bot_index]
            if position_updates != None:
                for each_joint_name, each_value in position_updates.items():
                    # only add an entry if the value is not None
                    if each_value != None:
                        time_mappings[each_joint_name][time_value] = each_value
        bot_time_mappings.append(time_mappings)

    def joint_at_time(bot_index, joint_name, time):
        key_times = tuple(bot_time_mappings[bot_index][joint_name].items())
        prev_time_and_value = list(key_times[0])
        next_time_and_value = list(key_times[0])
        # by the end of the loop prev_time and next_time will correctly be < and > the time argument
        for next_time, next_value in key_times:
            next_time_and_value = [ next_time, next_value ]
            # found the true "next" point
            if time <= next_time:
                break
            prev_time_and_value = next_time_and_value
        
        # just need to interpolate the values
        prev_time, prev_value = prev_time_and_value
        next_time, next_value = next_time_and_value
        duration = next_time - prev_time
        if duration == 0:
            return prev_value # prev_value should == next_value at this point
        assert duration > 0
        relative_time_progress = time - prev_time
        progress_between_points = relative_time_progress/duration
        interpolated_value = shift_towards(new_value=next_value, old_value=prev_value, proportion=progress_between_points)
        return interpolated_value
    
    
    from copy import deepcopy
    def time_to_positions(time):
        # out-of-bound times
        if time <= routine[0].time:
            return routine[0].positions
        if time >= routine[-1].time:
            return routine[-1].positions
        
        # in-bound times
        positions_per_bot = []
        for bot_index, each_bot_mapping in enumerate(bot_time_mappings):
            positions = LazyDict()
            for joint_name in each_bot_mapping.keys():
                positions[joint_name] = joint_at_time(bot_index=bot_index, joint_name=joint_name, time=time)
            positions_per_bot.append(positions)
        
        return positions_per_bot
    
    return time_to_positions

time_to_positions = convert_routine_to_function(routine)
start_time = None

class MultiRobotBehavior(object):
    """
    Multi Robot Behavior class.
    """
    def __init__(self):
        self.publishers = []
        for i in range(4):
            topic_name = f"/sb_{i}_cmd_state"
            self.publishers.append(rospy.Publisher(
                topic_name, TwistStamped, queue_size=1))
        # rate = rospy.Rate(10)  # 10hz

        self.twist = TwistStamped()

    def execute_action(self, positions):
        for bot_index, position in enumerate(positions):
            self.twist.twist.linear.x  = position["torso_joint"] 
            self.twist.twist.linear.y  = position["neck_swivel"]
            self.twist.twist.linear.z  = position["head_tilt"]
            self.twist.twist.angular.x = position["head_nod"]
            
            self.publishers[bot_index].publish(self.twist)
        
    def sb_callback(self):
        global start_time
        if start_time == None:
            start_time = time.time()
        
        time_progress = time.time() - start_time
        self.execute_action(time_to_positions(time_progress))
        print("T =",time_progress)


    	
if __name__ == '__main__':
    rospy.init_node("lab_4_node")
    rate = rospy.Rate(10)  # 10hz
    
    ##############################
    # YOUR CODE HERE             #
    # Call the behavior(s)       #
    ##############################
    
    multi = MultiRobotBehavior()

    # play music
    t = Thread(target=lambda: playsound('music.mp3'), args=[])
    t.start()

    while not rospy.is_shutdown():
        multi.sb_callback()
        rate.sleep()
    #while not rospy.is_shutdown():
    #    multi.sb_callback()

    rospy.spin()