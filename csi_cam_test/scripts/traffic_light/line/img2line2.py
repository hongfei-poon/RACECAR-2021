#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32, Float32MultiArray, Int16
from sensor_msgs.msg import Image as imgemsg
from _41combine import *
from Timer import *


class find_line:
    def __init__(self):
        # 创建cv_bridge，声明图像的发布者和订阅者
        # self.image_pub = rospy.Publisher("line_control_img", imgemsg, queue_size=1)
        self.control_pub = rospy.Publisher("line_number_direction", Float32, queue_size=5)
        self.bridge = CvBridge()
        self.start_line_sub = rospy.Subscriber("cv_control", Int16, self.callback_start)
        self.image_sub = rospy.Subscriber("csi_stereo/left/image_raw", imgemsg, self.callback)

        self.start_line = 0
        self.control_dir = 0

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
            # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)

            # 在opencv的显示窗口中绘制一个圆，作为标记
            # (rows,cols,channels) = cv_image.shape
            # if cols > 60 and rows > 60 :
            #     cv2.circle(cv_image, (60, 60), 30, (0,0,255), -1)
            time_line = timer(6)
            control_direction, out_net = follow_line(cv_image, 0.5)
            print(control_direction)
            self.control_dir = control_direction
            control_pubnum = Float32(data=self.control_dir)
            time_line('time_line')
            # control_direction=1
            # 显示Opencv格式的图像
            # print('cv_image type',type(cv_image))
            # print('out_net type',type(out_net))
            cv2.imshow("outnet window", out_net)
            cv2.waitKey(3)
            print('show img success')
            # 再将opencv格式额数据转换成ros image格式的数据发布
            try:
                # self.image_pub.publish(self.bridge.cv2_to_imgmsg(out_net, "bgr8"))
                self.control_pub.publish(control_pubnum)
            except CvBridgeError as e:
                print(e)
        else:
            print('wait for control')


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
