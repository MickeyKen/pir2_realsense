#!/usr/bin/env python

import rospy, rospkg
import numpy as np
import math

import tf

import message_filters
from sensor_msgs.msg import PointCloud2
from people_msgs.msg import HeadPoseStamped, HeadPose
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Quaternion

class Publishers():

    def euler_to_quaternion(self, euler):
        pass
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

        self.pose_pub(pose_msg)


class Subscribe(Publishers):
    def __init__(self):
        self.pose_pub = pub = rospy.Publisher('/pir2/human_pose', Pose, queue_size=10)

        self.pcl_sub = message_filters.Subscriber('/points', PointCloud2)
        self.hp_sub = message_filters.Subscriber('/ros_openvino_toolkit/headposes_estimation', HeadPoseStamped)

        ts = message_filters.TimeSynchronizer([self.pcl_sub, self.hp_sub], 10)
        ts.registerCallback(self.callback)

    def callback(self, pcl_msg, hp_msg):
        u = hp.headposes[0].roi.x_offset
        v = hp.headposes[0].roi.y_offset
        r = hp.headposes[0].roll
        p = hp.headposes[0].pitch
        yaw = hp.headposes[0].yaw
        x,y,z = self,pixelTo3DPoint(cloud, u, v)
        self.make_pose(x,y,z,r,p,yaw)

    def pixelTo3DPoint(self, pcl_msg, u, v):
        width = cloud.width
        height = cloud.height
        point_step = cloud.point_step
        row_step = cloud.row_step

        array_pos = v*row_step + u*point_step

        bytesX = [ord(x) for x in cloud.data[array_pos:array_pos+4]]
        bytesY = [ord(x) for x in cloud.data[array_pos+4: array_pos+8]]
        bytesZ = [ord(x) for x in cloud.data[array_pos+8:array_pos+12]]

        byte_format=struct.pack('4B', *bytesX)
        X = struct.unpack('f', byte_format)[0]

        byte_format=struct.pack('4B', *bytesY)
        Y = struct.unpack('f', byte_format)[0]

        byte_format=struct.pack('4B', *bytesZ)
        Z = struct.unpack('f', byte_format)[0]

        return [X, Y, Z]

if __name__ == '__main__':
    rospy.init_node('calcurate_XYZ_publisher')

    Subscribe = Subscribe()

    rospy.spin()
