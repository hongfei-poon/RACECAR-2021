#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from __future__ import print_function
from numpy import float32, int32
import numpy
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float32MultiArray, Int16
from _01Main_ros import *


class find_light:
    def __init__(self):
        # 创建cv_bridge，声明图像的发布者和订阅者
        # 使用新的数据类型需要import
        # self.image_pub = rospy.Publisher("control_light", Image, queue_size=1)
        # self.control_pub = rospy.Publisher("light_dist", Float32MultiArray, queue_size=5)
        self.control_pub = rospy.Publisher("light_control", Float32, queue_size=3)

        self.bridge = CvBridge()
        self.start_light_sub = rospy.Subscriber("cv_control", Int16, self.callback_start)
        self.image_sub = rospy.Subscriber("csi_stereo/left/image_raw", Image, self.callback_img, queue_size=1,
                                          buff_size=2 ** 24)

        # 统计坐标为空的次数，到达10，停止这部分视觉任务/暂时停用
        self.count_composition = 0
        self.stop_car = 0
        self.start_light = 1
        self.count_red = 0
        self.count_green = 0

        self.num = 0

    def callback_start(self, data):
        try:
            print('judge start light or not', end='  ')
            cv_con = data.data
            print(cv_con, end=' ')
        except ValueError as e:
            print(e)
        if cv_con == 1:
            print('open traffic light')
            self.start_light = 1
        else:
            self.start_light = 0
            print('close traffic light')

    def callback_img(self, data):
        # cv_mode =1,in area
        if self.start_light == 1:

            # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)
            # 判断图片大小是否正常
            (rows, cols, channels) = cv_image.shape
            if cols > 60 and rows > 60:
                # cv2.circle(cv_image, (60, 60), 30, (0,0,255), -1)
                ret_judge = 1
                print('receive the img and go for judge')
            else:
                ret_judge = 0

            # 坐标位置，交通灯颜色 012红黄绿，添加相关信息的图片
            composition, light_color, img_from_light = img2ligth(ret_judge, cv_image)

            self.count_red = max(self.count_red, 0)
            self.count_green = max(self.count_green, 0)
            self.count_red = min(self.count_red, 10)
            self.count_green = min(self.count_green, 10)

            if composition is None:
                print('none of composition-----------')
                self.count_composition = self.count_composition + 1
                if self.count_composition > 6:
                    self.stop_car = 0
                    self.count_red = 0
                    self.count_green = 0

            else:
                # print('com is ok')
                self.count_composition = 0
                # print('find_the_composition,now_sen
                if composition is not None:
                    self.stop_car = self.stop_car
                    # print('look at color and stop or not')
                if not len(light_color):
                    print('none in color，set to 0 -1')
                    light_color = [-1]
                    self.count_red = self.count_red
                    self.count_green = self.count_green + 0.5

                if light_color[0] == 0:  # 红灯停车
                    print('red---------------------------------------------------------')
                    self.count_red = self.count_red + 5
                    self.count_green = self.count_green - 5

                if light_color[0] == 2:
                    self.count_green = self.count_green + 5
                    self.count_red = self.count_red - 3

                if light_color[0] == 1:  # 黄灯，绿灯进行行车,-1
                    print('yellow-------------------')
                    self.count_red = self.count_red + 5
                    self.count_green = self.count_green - 5

                print('final red', self.count_red)  # 发布前获得最终的结果
                print('final green', self.count_green)

                if self.count_red >= 5:
                    self.stop_car = 1
                if self.count_green >= 6:
                    self.stop_car = 0

            fi = Float32(data=self.stop_car)
            print('this is final light------------------------------------------', fi)
            # print('count none of compo', self.count_composition)
            # 再将opencv格式额数据转换成ros image格式的数据发布
            try:
                # print('now_try_to_send_img')
                # self.image_pub.publish(self.bridge.cv2_to_imgmsg(img_from_light, "bgr8"))
                self.control_pub.publish(fi)
                print(fi)
                # cv2.imshow("img_to_node", img_from_light)
                # cv2.waitKey(3)
            except CvBridgeError as e:
                print(e)

            # else:
            #     self.stop_car = 0
            #     # not in area ,keep moving
            #     fi = Float32(data=self.stop_car)
            #     print('not in the area', fi)
            #     self.control_pub.publish(fi)
        if self.start_light == 0:
            self.stop_car = 0
            # not in area ,keep moving
            fi = Float32(data=self.stop_car)
            print('not in the area', fi)
            self.control_pub.publish(fi)


if __name__ == '__main__':
    try:
        # print("OK")
        # 初始化ros节点
        rospy.init_node("traffic_light")
        rospy.loginfo("Starting traffic_light node")
        find_light()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down traffic_light node.")
        cv2.destroyAllWindows()
