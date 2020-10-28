#! /usr/bin/env python3

import rospy
from std_msgs.msg import String

def main():
    rospy.init_node("simple_pub_node")

    pub = rospy.Publisher("/talk", String, queue_size=1)

    msg = String()
    msg.data = "Hello World"

    # 2Hz Rate:

    rate = rospy.Rate(2)
    while(not rospy.is_shutdown()):
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":

    try:
        main()
    except rospy.ROSInterruptException:
        pass
