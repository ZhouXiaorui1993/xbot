#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division  # python2的 / 默认是整除
from cv_bridge import CvBridge
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from geometry_msgs.msg import Point, Quaternion
import PyKDL
import nav_msgs.msg
import numpy as np
import rospy
import cv2
import tf
import base64


class Base64Map(object):
    def __init__(self):

        # init node
        rospy.init_node("map_to_base64", anonymous=False)

        # base frame
        self.robot_base_frame = rospy.get_param("/slam_gmapping/base_frame", default="base_footprint")

        # init a tf listener
        self.tf_listener = tf.TransformListener()

        # give tf some time to fill the buffer
        rospy.sleep(2)

        # get map from /map
        self.map_data = None
        self.robot_pose = None

        # Publisher
        self.str_img_pub = rospy.Publisher('/base64_img/map_img', String, queue_size=1)

        # publish rate
        self.pub_rate = rospy.Rate(10)

        # waiting for /map_metadata be available
        rospy.loginfo("waiting for /map_metadata...")
        self.map_metadata = rospy.wait_for_message('/map_metadata', MapMetaData)

        # get map metadata
        self.map_resolution = self.map_metadata.resolution
        self.map_width = self.map_metadata.width
        self.map_height = self.map_metadata.height
        self.map_origin_x = self.map_metadata.origin.position.x
        self.map_origin_y = self.map_metadata.origin.position.y

        # waiting for /map be available
        rospy.loginfo("waiting for /map")
        rospy.wait_for_message('/map_metadata', MapMetaData)
        # subscriber
        rospy.Subscriber('map', OccupancyGrid, self.map_cb)

        while not rospy.is_shutdown():
            if self.map_data is not None:
                map_arr = np.random.random((self.map_height, self.map_width, 3))
                arr = np.reshape(self.map_data, (self.map_height, self.map_width))

                # -1 is unknown. If we cast that to a uint8, it won't look right.
                # map_saver uses 205 for unknown values, so let's change all -1 cells to 205
                # unknown space
                map_arr[:, :, 0][arr == -1] = 100
                map_arr[:, :, 1][arr == -1] = 100
                map_arr[:, :, 2][arr == -1] = 100

                # obstacles
                map_arr[:, :, 0][arr == 100] = 0
                map_arr[:, :, 1][arr == 100] = 0
                map_arr[:, :, 2][arr == 100] = 0

                # known space
                map_arr[:, :, 0][arr == 0] = 121
                map_arr[:, :, 1][arr == 0] = 213
                map_arr[:, :, 2][arr == 0] = 117

                # throw a robot into map
                self.add_robot(map_arr)

                img = np.uint8(np.flipud(map_arr))

                img_encode = cv2.imencode('.png', img)[1]
                data_encode = np.array(img_encode)

                # str_encode = data_encode.tostring()
                str_encode = base64.b64encode(data_encode)

                # pub string encode map
                self.str_img_pub.publish(str_encode)
                self.pub_rate.sleep()

    def get_now_pos(self):
        try:
            # get now robot pos from listen the transform between map and base frame
            (trans, rot) = self.tf_listener.lookupTransform('map', 'base_footprint', rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf Exception")
            return

        return Point(*trans), self.quat_to_angle(Quaternion(*rot))

    def map_cb(self, map_msg):
        """
        get map data
        """
        self.map_data = map_msg.data

    def add_robot(self, arr):
        # get now robot position
        position, orientation = self.get_now_pos()
        self.robot_pose = position
        # x = int(round((self.robot_pose.x - self.map_origin_x) / self.map_resolution - int(self.map_width / 2)))
        # y = int(round((self.robot_pose.y - self.map_origin_y) / self.map_resolution - int(self.map_height / 2)))
        x = int(round((self.robot_pose.x - self.map_origin_x) / self.map_resolution))
        y = int(round((self.robot_pose.y - self.map_origin_y) / self.map_resolution))
        arr[y - 3:y + 3, x - 3:x + 3, 0] = 0
        arr[y - 3:y + 3, x - 3:x + 3, 1] = 0
        arr[y - 3:y + 3, x - 3:x + 3, 2] = 255

        print("robot in world:%s" % [position.x, position.y])
        print("robot in ad map:%s" % [x, y])

    @staticmethod
    def quat_to_angle(quat):
        rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
        return rot.GetRPY()[2]


if __name__ == '__main__':
    try:
        Base64Map()
        rospy.spin()
    except rospy.ROSException:
        rospy.loginfo("map_to_base64 node terminated.")
