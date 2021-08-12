#!/usr/bin/env python3
# -*- coding:utf-8 _*-
"""
@author:YuTao
@time: 2019/07/01
"""
import numpy as np
import cv2
import matplotlib.pyplot as plt
from _02CalculatePositon import *
from _03TrafficLight import *



def img2ligth(ret_judge, img):
    np.set_printoptions(suppress=True, precision=4)
    # 持续读取
    Img = img
    ret = ret_judge
    if ret:
        # %% 由Marker计算小车的相对位置，同时得到marker图像区域
        CamPosition, MarkerROI = DealMarker(Img)  # 02当中的函数 CamPosition：(x,y,z)
        # %% 实现交通灯颜色检测
        LightColors = TrafficLight(MarkerROI, Img)  # 03LightColors：0-'Red', 1-'Yellow', 2-'Green'
        
        # if CamPosition is not None:
        # print('Frame:', Frame)
        print(CamPosition, LightColors, 'composition and light')  # 输出小车位置和交通灯颜色
        # Img = cv2.resize(Img, (160,120))
        # cv2.imshow('Video', Img)
        # key = cv2.waitKey(5)
        # if key != -1:
        # 	exit()
        # if CamPosition is not None:
        # 	Point = ax.scatter(CamPosition[0], CamPosition[1], s=5)
        # 	plt.pause(0.01)
        # Point.remove()
        # if CamPosition is None:
        # 	print('wrong in	composition')
        return CamPosition, LightColors, Img
    else:
        print('fail to open camera')


# plt.show()
# plt.pause(5)
if __name__ == '__main__':
    # Video = cv2.VideoCapture(0)#打开电脑摄像头
    Video = cv2.VideoCapture("12.mp4")
    # Video = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    while (1):
        ret_judge, Img = Video.read()
        if not ret_judge:
            print("finish")
            break
        Camposition, b, img = img2ligth(ret_judge, Img)
        print('type_of_img', type(img))
        if Camposition is not None:
            cam_send = Camposition[0]
        print(cam_send, b)

    Video.release()
