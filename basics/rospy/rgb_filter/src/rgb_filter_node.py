#! /usr/bin/env python3

import rospy

import sys

class RGBFilter():
    def __init__(self):
        pass

    def step(self):
        rospy.loginfo("Step!")

def main(args=None):
    rospy.init_node("rgb_filter_node")

    filter = RGBFilter()

    loop_rate = rospy.Rate(4)   # 4Hz
    try:
        while (not rospy.is_shutdown()):
            filter.step()
            loop_rate.sleep()

    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main(sys.argv)
