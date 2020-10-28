#! /usr/bin/env python3

import sys
import rospy
from std_msgs.msg import String

def callback(msg):
    rospy.loginfo("I received: " + msg.data)

def main(args=None):
    rospy.init_node("simple_sub_node")

    sub = rospy.Subscriber("/talk", String, callback)

    rospy.spin()

if __name__ == "__main__":

    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass
