#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32, Float32MultiArray, Int16
from sensor_msgs.msg import Image as imgemsg
# from _41combine import *
from Timer import *
from sensor_msgs.msg import CompressedImage
import numpy as np
from _33Warpimg import *


class compress:
    def __init__(self):
        # 创建cv_bridge，声明图像的发布者和订阅者
        self.image_pub = rospy.Publisher("img_compressed", CompressedImage, queue_size=1)
        # self.image_pub = rospy.Publisher("line_control_img", imgemsg, queue_size=1)
        # self.control_pub = rospy.Publisher("line_number_direction", Float32, queue_size=5)
        self.bridge = CvBridge()
        self.start_line_sub = rospy.Subscriber("cv_control", Int16, self.callback_start)
        self.image_sub = rospy.Subscriber("csi_stereo/left/image_raw", imgemsg, self.callback)

        self.start_line = 1
        # self.control_dir = 0

    def callback_start(self, data):
        try:
            print('judge line start or not')
            cv_con = data
        except ValueError as e:
            print(e)
        if cv_con == 2:
            self.start_line = 1
        else:
            self.start_line = 0

    def callback(self, data):
        if self.start_line == 1:
            time_compress = timer(6)
            # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
            try:
                time_msg = timer(5)
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                time_msg('msg-cv')
            except CvBridgeError as e:
                print(e)

            # cv_image = warpimg(cv_image)
            cv_image = cv2.resize(cv_image, (160, 120))

            time_com = timer(5)
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tostring()
            print(cv_image.shape)
            time_com('compress')
            # 再将opencv格式额数据转换成ros image格式的数据发布
            try:
                # self.image_pub.publish(self.bridge.cv2_to_imgmsg(out_net, "bgr8"))
                # time_press_msg = timer(5)
                self.image_pub.publish(msg)

                # fi = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
                # time_press_msg('press to msg')
                # time_wait = timer(5)
                # cv2.imshow('press-msfg', fi)
                # cv2.waitKey(1)
                # time_wait('waitkey')
                time_compress('msg-compress-img-took')
                # self.control_pub.publish(control_pubnum)
            except CvBridgeError as e:
                print(e)


if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("compress")
        rospy.loginfo("Starting compress node")
        compress()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down find line node.")
        cv2.destroyAllWindows()
