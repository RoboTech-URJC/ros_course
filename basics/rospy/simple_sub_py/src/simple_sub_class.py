#! /usr/bin/env python3

import sys
import rospy
from std_msgs.msg import Int32

class Subscriber():
    def __init__(self):
        self.sub_ = rospy.Subscriber("/talk", Int32, self.callback_)

    def callback_(self, msg):
        rospy.loginfo("I received: " + str(msg.data))

def main(args=None):
    rospy.init_node("simple_sub_node")

    sub = Subscriber()

    rate = rospy.Rate(1)
    while(not rospy.is_shutdown()):
        rospy.loginfo("I'm in the loop")
        rate.sleep()

    rospy.spin()

if __name__ == "__main__":

    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass
