#!/usr/bin/env python
# -*- coding: utf-8 -*-
# import os
# import tty
# import termios
import sys
import socket
# import threading
# import multiprocessing
# import time
# from time import sleep
import rospy
#from sensor_msgs.msg import Image   #获取摄像头图像
#import cv2
#from cv_bridge import CvBridge
# from std_msgs.msg import String
from geometry_msgs.msg import Pose
# from geometry_msgs.msg import PoseArray
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist
# import numpy as np
address = ('192.168.3.109', 8888)
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
def SpeedCallback(msg):
    data_l=msg.twist.twist.linear.x
    data_a=msg.twist.twist.argular.z
    rospy.loginfo("Subcribe Info: linear=%d  argular=%d", data_l,data_a)
    return(data_l,data_a)
def Speed_sub():
    rospy.init_node('car_speed',anonymous=True)
    rospy.Subscriber("/odom",Pose,SpeedCallback)
    rospy.spin()
def PoseCallback(msg):
    data_x = msg.pose.pose.position.x
    data_y = msg.pose.pose.position.y
    data_w = msg.pose.pose.orientation.w
    rospy.loginfo("Subcribe Info: x=%d  y=%d  w=%d", data_x,data_y,data_w)
    return(data_x,data_y,data_w)
def Pose_sub():
    rospy.init_node('car_pose',anonymous=True)
    rospy.Subscriber("/odom",Pose,PoseCallback)
    rospy.spin()
if __name__ == "__main__":
    try:
        Pose_sub()
        Speed_sub()
    except:
        rospy.loginfo("获取小车数据失败")
    try:
        s.connect(address)  # 尝试连接服务端
    except Exception:
        print('[!] Server not found ot not open')
        sys.exit()
    while True:
        trigger1 = Speed_sub()
        trigger2 = Pose_sub()
        print(trigger1,trigger2)
        s.sendall(trigger1.encode())
        s.sendall(trigger2.encode())
        data = s.recv(1024)
        data = data.decode()
        print('[Recieved]', data)
        if trigger1 == '' or trigger2 == '' :  # 自定义结束字符串
            break