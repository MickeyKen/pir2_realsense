#!/usr/bin/env python

import rospy, rospkg
import numpy as np
import math
import struct
import cv2

import tf

import message_filters
from sensor_msgs.msg import Image
from people_msgs.msg import HeadPoseStamped, HeadPose
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Quaternion

from cv_bridge import CvBridge, CvBridgeError


class Publishers():

    def make_image(self, u,v,in_img):

        img_msg = Image()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(in_img, "32FC1")
        except CvBridgeError as e:
            print(e)

        # pixel = (v-1)*depth_msg.width + u
        cv_image = cv2.circle(cv_image,(u,v), 10, (0,0,0), -1)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)
        # img_msg = self.bridge.cv2_to_imgmsg(cv_image, "32FC1")
        #
        # self.image_pub.publish(img_msg)


    def make_pose(self, x,y,z,roll,pitch,yaw):
        pose_msg = Pose()
        pose_msg.position.x = x
        pose_msg.position.y = y
        pose_msg.position.z = z

        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        #type(pose) = geometry_msgs.msg.Pose
        pose_msg.orientation.x = quaternion[0]
        pose_msg.orientation.y = quaternion[1]
        pose_msg.orientation.z = quaternion[2]
        pose_msg.orientation.w = quaternion[3]

        print pose_msg

        self.pose_pub.publish(pose_msg)


class Subscribe(Publishers):
    def __init__(self):
        self.pose_pub = rospy.Publisher('/pir2/human_pose', Pose, queue_size=10)
        self.image_pub = rospy.Publisher("/image_topic_2",Image, queue_size=10)
        self.bridge = CvBridge()

        self.pcl_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image)
        self.hp_sub = message_filters.Subscriber('/ros_openvino_toolkit/headposes_estimation', HeadPoseStamped)

        ts = message_filters.ApproximateTimeSynchronizer([self.pcl_sub, self.hp_sub],10, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)

    def callback(self, depth_msg, hp_msg):
        # print len(depth_msg.data)
        pixel = 0
        u = hp_msg.headposes[0].roi.x_offset + (hp_msg.headposes[0].roi.width/2)
        v = hp_msg.headposes[0].roi.y_offset + (hp_msg.headposes[0].roi.height/2)
        r = hp_msg.headposes[0].roll
        p = hp_msg.headposes[0].pitch
        yaw = hp_msg.headposes[0].yaw
        pixel = (v-1)*depth_msg.width + u
        # distance = depth_msg.data[pixel]
        # print pixel
        if pixel < 148346:
            self.make_image(u,240,depth_msg)
            print ord(depth_msg.data[0][0])
        # x,y,z = self.pixelTo3DPoint(pcl_msg, u, v)
        # self.make_pose(x,y,z,r,p,yaw)


if __name__ == '__main__':
    rospy.init_node('calcurate_XYZ_publisher')

    Subscribe = Subscribe()

    rospy.spin()
