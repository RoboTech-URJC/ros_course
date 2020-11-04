#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import sys
import math
import time
import random

class Walker():
    def __init__(self):
        self.initParams_()

        self.laser_sub_ = rospy.Subscriber("/scan", LaserScan, self.laserCallback_, queue_size=1)
        self.vel_pub_ = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def step(self):
        vel_msg = Twist()

        x = yaw = 0.0
        if (self.laser_measure_ == None):
            return

        if (self.state_ == self.go_forward_st_):
            x = self.forward_vel_
            yaw = 0.0
            if (self.laser_measure_ < self.min_laser_measure_):
                self.state_ = self.go_backward_st_
        elif (self.state_ == self.go_backward_st_):
            x = self.backward_vel_
            yaw = 0.0
            tf= time.time()
            if (tf - self.t0_ >= self.back_time_):
                self.state_ = self.turn_st_
                self.t0_ = tf
        else:
            x = 0.0
            turning_vels = [-1, 1]
            if (self.is_first_turn_):
                self.yaw_vel_ = random.choice(turning_vels)
                self.is_first_turn_ = False
            yaw = self.yaw_vel_ * self.turning_vel_   # Turn to left or right randomly
            tf = time.time()
            if (tf - self.t0_ >= self.turning_time_):
                self.state_ = self.go_forward_st_
                self.is_first_turn_ = True
                self.t0_ = tf


        vel_msg.linear.x = x
        vel_msg.angular.z = yaw

        self.vel_pub_.publish(vel_msg)

    def initParams_(self):
        self.min_laser_measure_ = 0.3
        self.forward_vel_ = 0.06
        self.backward_vel_ = -0.03
        self.turning_vel_ = math.pi / 4.5
        self.back_time_ = 1.5
        self.turning_time_ = 2.0
        self.laser_measure_ = None
        self.state_ = 0
        self.is_first_turn_ = True
        self.yaw_vel_ = 0.0
        self.t0_ = time.time()

        # States:

        self.go_forward_st_ = 0
        self.go_backward_st_ = 1
        self.turn_st_ = 2

    def laserCallback_(self, msg):
        # Save the laser measure that correspond to the front:

        self.laser_measure_ = msg.ranges[0]

def main(args):
    # Initialize the node:

    rospy.init_node("bump_go_poo_node")

    # Create the 'walker' object of the class 'Walker'

    walker = Walker()

    loop_rate = rospy.Rate(5)   # 5Hz
    try:

        while (not rospy.is_shutdown()):
            walker.step()
            loop_rate.sleep()

        rospy.spin()

    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main(sys.argv)