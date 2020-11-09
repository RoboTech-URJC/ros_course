#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2
import sys

class RGBFilter():
    def __init__(self):
        self.initParams_()
        self.rgb_img_ = None
        self.cv_bridge = CvBridge()

        self.image_sub_ = rospy.Subscriber(self.image_topic_, Image, self.imageCb_)

    def __del__(self):
        cv2.destroyAllWindows()

    def step(self):
        if (self.rgb_img_ == None):
            return

        # sensor_msgs/Image to OpenCV image:

        try:
            cv_img = self.cv_bridge.imgmsg_to_cv2(self.rgb_img_, self.rgb_img_.encoding)
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Resize the image:

        img_resized = cv2.resize(cv_img, (640, 480), interpolation = cv2.INTER_AREA)

        # Show the image:

        cv2.imshow("RGB Image", img_resized)
        cv2.waitKey(1)

    def imageCb_(self, msg):
        self.rgb_img_ = msg

    def initParams_(self):
        self.image_topic_ = rospy.get_param("~image_sub", "camera/rgb/image_raw")

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
