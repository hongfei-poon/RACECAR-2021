#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32, Float32MultiArray, Int16
from sensor_msgs.msg import Image as imgemsg
from _41combine import *
from Timer import *
import numpy as np
from sensor_msgs.msg import CompressedImage


class find_line:
    def __init__(self):
        # 创建cv_bridge，声明图像的发布者和订阅者
        # self.image_pub = rospy.Publisher("line_control_img", imgemsg, queue_size=1)
        self.control_pub = rospy.Publisher("line_number_direction", Float32MultiArray, queue_size=1)
        self.bridge = CvBridge()
        self.start_line_sub = rospy.Subscriber("cv_control", Int16, self.callback_start)
        self.image_sub = rospy.Subscriber("img_compressed", CompressedImage, self.callback, queue_size=1,buff_size=2 ** 24)

        self.array = [1, 0]
        # arr[0]=1,run
        self.start_line = 1
        self.control_dir = 0
        self.count_lowret = 0

    def callback_start(self, data):
        try:
            print('judge line start or not')
            cv_con = data
        except ValueError as e:
            print(e)
        if cv_con == 2:
            self.start_line = 1
            self.array[0] = 1
        else:
            self.start_line = 0
            self.array[0] = 0

    def callback(self, data):
        if self.start_line == 1:
            time_all = timer(6)
            time_msgcv = timer(5)
            # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
            try:
                # cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)

            time_msgcv('on linux commsg_cv')
            print("Processing frame | Delay:%6.3f" % (rospy.Time.now() - data.header.stamp).to_sec())

            time_line = timer(6)
            # cv2.imshow('sub-img', cv_image)
            # cv2.waitKey(2)

            cv_image = cv2.resize(cv_image, (640, 480))
            control_direction, out_net, ret = follow_line(cv_image, 0.5)

            if ret < 20:
                self.count_lowret = self.count_lowret + 1
            if ret > 30:
                self.count_lowret = 0
            else:
                self.count_lowret = self.count_lowret

            if self.count_lowret > 30:
                self.array[0] = 0
                self.start_line=0
                print('not find line kill line_info---------------')

            self.control_dir = control_direction
            self.array[1] = control_direction
            control_pubnum = Float32MultiArray(data=self.array)
            time_line('time_line_all')

            time_show = timer(5)
            out_net = cv2.resize(out_net, (int(out_net.shape[1] / 2), int(out_net.shape[0] / 2)))
            cv2.imshow("outnet window", out_net)
            cv2.waitKey(1)
            time_show('time_show')
            # 再将opencv格式额数据转换成ros image格式的数据发布

            try:
                # self.image_pub.publish(self.bridge.cv2_to_imgmsg(out_net, "bgr8"))
                self.control_pub.publish(control_pubnum)
                time_all('all thing ginish------')
                print("Processing pub the control | Delay:%6.3f" % (rospy.Time.now() - data.header.stamp).to_sec())
            except CvBridgeError as e:
                print(e)


if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("find_line")
        rospy.loginfo("Starting find_line node")
        find_line()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down find line node.")
        cv2.destroyAllWindows()
