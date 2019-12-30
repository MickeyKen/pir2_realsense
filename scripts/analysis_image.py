#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import Image
import cv2
import math


class Subscribe():
    def __init__(self):

        self.subscriber = rospy.Subscriber('/camera/color/image_raw', Image, self.callback)

    def callback(self, msg):

        for i in range(614400):
            print ord(msg.data[i])

def main():

    rospy.init_node('analysis_Image')

    sub = Subscribe()

    rospy.spin()

if __name__ == '__main__':
    main()
