#! /usr/bin/env python3

import rospy
from std_msgs.msg import String

class Publisher():
    def __init__(self):
        self.pub_ = rospy.Publisher("/talk", String, queue_size=1)

    def doWork(self):
        msg = String()
        msg.data = "Hello World"
        self.pub_.publish(msg)

def main():
    rospy.init_node("simple_pub_class_node")

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
