#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import Image
import cv2
import math

from cv_bridge import CvBridge, CvBridgeError


class Subscribe():
    def __init__(self):

        self.subscriber = rospy.Subscriber('/camera/color/image_raw', Image, self.callback)
        self.bridge = CvBridge()

    def callback(self, msg):

        depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        depth_array = np.array(depth_image, dtype=np.float32)
        # print depth_array[200, 100]
        print depth_image[0,0]
        print "*******************************************"

        # print msg.width ,msg.height
        # for i in range(614400):
        #     print ord(msg.data[i])

def main():

    rospy.init_node('analysis_Image')

    sub = Subscribe()

    rospy.spin()

if __name__ == '__main__':
    main()
