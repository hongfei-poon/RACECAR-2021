#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
# from _01Main_ros import *

class find_line:
    def __init__(self):    
        # 创建cv_bridge，声明图像的发布者和订阅者
        self.image_pub = rospy.Publisher("image_control_light", Image, queue_size=1)
        # self.control_pub=rospy.Publisher("control_light",int8,queue_size=5)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("csi_stereo/left/image_raw", Image, self.callback)

    def callback(self,data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print (e)

        # 在opencv的显示窗口中绘制一个圆，作为标记
        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            cv2.circle(cv_image, (60, 60), 30, (0,0,255), -1)

        # a, b = img2ligth(ret_judge, cv_image)

        # 显示Opencv格式的图像
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)



        # 再将opencv格式额数据转换成ros image格式的数据发布
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            # self.control_pub.publish(control_direction)
        except CvBridgeError as e:
            print (e)

if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("find_line")
        rospy.loginfo("Starting find_line node")
        find_line()
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down cv_bridge_test node.")
        cv2.destroyAllWindows()
