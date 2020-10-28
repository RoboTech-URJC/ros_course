#! /usr/bin/env python3

import rospy
from std_msgs.msg import Int32

class Publisher():
    def __init__(self):
        self.pub_ = rospy.Publisher("/talk", Int32, queue_size=1)
        self.msg_ = Int32()
        self.msg_.data = 0

    def doWork(self):
        self.pub_.publish(self.msg_)
        self.msg_.data += 1

def main():
    rospy.init_node("simple_pub_iterative_class_node")

    publisher = Publisher()

    # 2Hz Rate:

    rate = rospy.Rate(2)
    while(not rospy.is_shutdown()):
        publisher.doWork()
        rate.sleep()

if __name__ == "__main__":

    try:
        main()
    except rospy.ROSInterruptException:
        pass
