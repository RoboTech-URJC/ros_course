#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import sys
import math
import time
import random

# Const:

MinLaserMeasure = 0.3
GoForward = 0
GoBackward = 1
Turn = 2
ForwardVel = 0.06
BackwardVel = -0.03
TurningVel = math.pi / 4.5
BackTime = 1.5
TurningTime = 2.0

laser_measure = 8.0
state = 0
t0 = time.time()

# I need to know I command to the TB3 to turn by first time
#after other state or not

is_first_turn = True
yaw_vel = 0.0

def laser_callback(msg):
    # Save the laser measure that correspond to the front:

    global laser_measure

    laser_measure = msg.ranges[0]

def step(pub):
    vel_msg = Twist()

    x = yaw = 0.0
    global state, t0, is_first_turn, yaw_vel
    if (state == GoForward):
        x = ForwardVel
        yaw = 0.0
        if (laser_measure < MinLaserMeasure):
            state = GoBackward
    elif (state == GoBackward):
        x = BackwardVel
        yaw = 0.0
        tf= time.time()
        if (tf - t0 >= BackTime):
            state = Turn
            t0 = tf
    else:
        x = 0.0
        turning_vels = [-1, 1]
        if (is_first_turn):
            yaw_vel = random.choice(turning_vels)
            is_first_turn = False
        yaw = yaw_vel * TurningVel   # Turn to left or right randomly
        tf = time.time()
        if (tf - t0 >= TurningTime):
            state = GoForward
            is_first_turn = True
            t0 = tf


    vel_msg.linear.x = x
    vel_msg.angular.z = yaw

    pub.publish(vel_msg)

def main(args):
    # Initialize the node:

    rospy.init_node("bump_go_node")

    # Create the subscriber:

    laser_sub = rospy.Subscriber("/scan", LaserScan, laser_callback, queue_size=1)
    vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    loop_rate = rospy.Rate(5)   # 2Hz
    try:
        while (not rospy.is_shutdown()):
            step(vel_pub)
            loop_rate.sleep()
        
        rospy.spin()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main(sys.argv)