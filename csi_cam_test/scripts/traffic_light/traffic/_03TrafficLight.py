# -*- coding:utf-8 _*-
"""
@author:YuTao
@time: 2019/07/01
"""
import cv2
import numpy as np

Red = np.array([123, 125, 205.])

Green = np.array([97, 172, 77.])

Yellow = np.array([110, 140, 200.])

Colors = (Red, Yellow, Green)
ColorsName = ('Red', 'Yellow', 'Green')
DistThreshold = 3000  # 颜色距离阈值#2000 5000


def JudgeLightColor(Light):
    Dist = np.empty((0,))
    print('light_in_color', Light)
    # a继续添加   w+
    for Color in Colors:
        Dist = np.append(Dist, np.sum(abs(Color - Light) ** 2))  #
    print('dist-color', Dist)
    # print('color to return', np.argmin(Dist), np.min(Dist))
    return np.argmin(Dist), np.min(Dist)


# 比较获取出来的light与颜色color的dist差值，最小最接近
# argmin给出水平方向最小值的下标


def TrafficLight(MarkerROI, Img):
    LightColors = []

    if MarkerROI is None:
        print('wrong in MarkerROI')

    if MarkerROI is not None:  # 如果检测到Marker，CamPosition和MarkerROI就不是None
        W = MarkerROI[2] - MarkerROI[0]
        H = MarkerROI[3] - MarkerROI[1]
        # MinY = max(MarkerROI[1] - int(2.2 * H), 0)
        # MaxY = min(MarkerROI[3] - H, Img.shape[0])
        minx = max(MarkerROI[0] + int(0.26 * W), 0)
        maxx = min(MarkerROI[2] - int(0.26 * W), Img.shape[1])
        MinY = max(MarkerROI[1] - int(2.5 * H), 0)
        MaxY = min(MarkerROI[3] - int(1.3 * H), Img.shape[0])
        # a[0,1]   b[2,1]
        # c[0,3]    d[2,3]
        if MaxY <= MinY + 10:
            return LightColors
        LightImg = Img[MinY:MaxY, minx:maxx, :]  # 提取交通灯的小块区域图像
        LightImg = cv2.resize(LightImg, (2 * LightImg.shape[1], 2 * LightImg.shape[0]))

        # 提取亮点中心轮廓
        LightImgGray = cv2.cvtColor(LightImg, cv2.COLOR_BGR2GRAY)
        th, MaskImg = cv2.threshold(LightImgGray, 180, 255, cv2.THRESH_TOZERO)  # cv2.threshold (源图片, 阈值, 填充色, 阈值类型)
        # 第一个retVal（得到的阈值值（在后面一个方法中会用到）），第二个就是阈值化后的图像
        MaskImg = cv2.morphologyEx(MaskImg, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        # 各类形态学的变化 src传入的图片op进行变化的方式kernel表示方框的大小  open开运算，指的是先进行腐蚀操作，再进行膨胀操作
        i, contours, hierarchy = cv2.findContours(MaskImg, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)  # 轮廓
        # 轮廓本身，每条轮廓对应的属性。
        # a = cv2.findContours(MaskImg, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        # print(a)
        # exit()
        sel_contours = []

        # 根据面积筛选轮廓
        for index, contour in enumerate(contours):

            Area = cv2.contourArea(contour)  # 求轮廓面积
            Hull = cv2.convexHull(contour, False)  # 获取突出点节点
            HullArea = cv2.contourArea(Hull)  # 计算突出点节点面积
            print('inside counter', 'A-----------------', Area, Area / HullArea)
            # https://www.cnblogs.com/jclian91/p/9728488.html
            if Area * 10 > 155 and Area < 1800 and Area / HullArea > 0.85:
                print('go_for_counter_for_color')
                sel_contours.append(contour)
                # 形态学提取外轮廓区域
                MaskImg = np.zeros_like(LightImgGray)  # 初始化一个0矩阵大小类似lightimg
                cv2.drawContours(MaskImg, [contour], -1, 255, cv2.FILLED)  # 绘画出狂选的区域
                kernel = np.ones((int(H / 8), int(H / 8)), np.uint8)  # uint8是专门用于存储各种图像的（包括RGB，灰度图像等），范围是从0–255
                dilation = cv2.dilate(MaskImg, kernel, iterations=1)  # 膨胀src表示输入的图片， kernel表示方框的大小， iteration表示迭代的次数
                MaskImg = dilation - MaskImg
                MaskImg = cv2.cvtColor(MaskImg, cv2.COLOR_GRAY2BGR)
                OutSide = LightImg & MaskImg
                Index = np.argwhere(np.sum(OutSide, axis=2) > 0)
                GrayLevel = OutSide[Index[:, 0], Index[:, 1], :]
                Light = np.mean(GrayLevel, axis=0)
                Color, Dist = JudgeLightColor(Light)  # argmin and dist
                # print('dist',Dist)
                # print('color', Color)
                if Dist < DistThreshold:  # 颜色空间L2距离足够小，完成颜色判断
                    # LightColors=Color
                    LightColors.append(Color)

        # %% 显示交通灯小块区域
        #cv2.drawContours(LightImg, sel_contours, -1, (255, 0, 0), 3)
        #LightImg = cv2.resize(LightImg, (40, 30))
        #cv2.imshow('LightImg', LightImg)
        #cv2.waitKey(2)

    return LightColors
