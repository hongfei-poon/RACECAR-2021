# -*- coding:utf-8 _*-
"""
@author:YuTao
@time: 2019/07/01
"""
import numpy as np
import cv2
import matplotlib.pyplot as plt
from _02CalculatePositon import *
#from _03TrafficLight import *

Red = np.array([38, 28, 230.])
Yellow = np.array([11, 81, 178.])
Green = np.array([35, 128, 10.])
Colors = (Red, Yellow, Green)
ColorsName = ('Red', 'Yellow', 'Green')
DistThreshold = 2000    # 颜色距离阈值
def stop():
	num = input("please enter a number:")
	print(num)

def JudgeLightColor(Light):
	Dist = np.empty((0,))
	for Color in Colors:
		print('color',Color)
		Dist = np.append(Dist, np.sum(abs(Color - Light) ** 2))#
	return np.argmin(Dist), np.min(Dist)
	#argmin给出水平方向最小值的下标


def TrafficLight(MarkerROI, Img):
	LightColors = []
	if MarkerROI is  None:
		print('wrong in MarkerROI')
	if MarkerROI is not None:  # 如果检测到Marker，CamPosition和MarkerROI就不是None
		W = MarkerROI[2] - MarkerROI[0]
		H = MarkerROI[3] - MarkerROI[1]
		MinY = max(MarkerROI[1] - int(2.2 * H), 0)
		MaxY = min(MarkerROI[3] - H, Img.shape[0])
		print('W,H,MinY,MaxY',W,H,MinY,MaxY)
		if MaxY <= MinY + 10:
			return LightColors
		LightImg = Img[MinY:MaxY, MarkerROI[0]:MarkerROI[2], :]  # 提取交通灯的小块区域图像

		# 提取亮点中心轮廓
		LightImgGray = cv2.cvtColor(LightImg, cv2.COLOR_BGR2GRAY)
		th, MaskImg = cv2.threshold(LightImgGray, 200, 255, cv2.THRESH_TOZERO)#cv2.threshold (源图片, 阈值, 填充色, 阈值类型)
		#第一个retVal（得到的阈值值（在后面一个方法中会用到）），第二个就是阈值化后的图像
		print('MaskImg-threshole')
		cv2.imshow('m-threshole',MaskImg)
		cv2.waitKey(0)
		MaskImg = cv2.morphologyEx(MaskImg, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
		#各类形态学的变化 src传入的图片op进行变化的方式kernel表示方框的大小  open开运算，指的是先进行腐蚀操作，再进行膨胀操作
		cv2.imshow('m-morpho', MaskImg)
		cv2.waitKey(0)
		contours, hierarchy = cv2.findContours(MaskImg, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)#轮廓
		#轮廓本身，每条轮廓对应的属性。
		# a = cv2.findContours(MaskImg, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		# print(a)
		# exit()
		print('draw contours without change')
		#cv2.drawContours(MaskImg, contours, -1, (255, 0, 0), 3)
		sel_contours = []

		# 根据面积筛选轮廓
		for index, contour in enumerate(contours):
			Area = cv2.contourArea(contour)#求轮廓面积
			Hull = cv2.convexHull(contour, False)#寻找物体的凸包并绘制凸包的轮廓,hull为突出点点集合
			#https://www.cnblogs.com/jclian91/p/9728488.html
			HullArea = cv2.contourArea(Hull)
			if Area > 20 and Area < 1000 and Area / HullArea > 0.9:
				sel_contours.append(contour)
				# 形态学提取外轮廓区域
				MaskImg = np.zeros_like(LightImgGray)
				cv2.drawContours(MaskImg, [contour], -1, 255, cv2.FILLED)
				kernel = np.ones((int(H / 8), int(H / 8)), np.uint8)
				dilation = cv2.dilate(MaskImg, kernel, iterations=1)  # 膨胀
				MaskImg = dilation - MaskImg
				MaskImg = cv2.cvtColor(MaskImg, cv2.COLOR_GRAY2BGR)
				OutSide = LightImg & MaskImg
				print('outside',type(OutSide))
				Index = np.argwhere(np.sum(OutSide, axis=2) > 0)
				GrayLevel = OutSide[Index[:, 0], Index[:, 1], :]
				Light = np.mean(GrayLevel, axis=0)
				cv2.imshow('light', Light)
				cv2.waitKey(0)
				Color, Dist = JudgeLightColor(Light) #argmin and dist

				if Dist < DistThreshold:    # 颜色空间L2距离足够小，完成颜色判断
					LightColors.append(Color)

		# %% 显示交通灯小块区域
		cv2.drawContours(LightImg, sel_contours, -1, (255, 0, 0), 3)
		##画出轮廓：temp是白色幕布，contours是轮廓，-1表示全画，然后是颜色，厚度
		print('drawcontour')
		cv2.putText(Img, str([ColorsName[LightColor] for LightColor in LightColors]), (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 3)
		cv2.imshow('LightImg', LightImg)
		cv2.waitKey(0)
	return LightColors
#print('finish3______________________________________________________________')

Img=cv2.imread('3.png',1)
#cv2.imshow('3,jpg',Img)
plt.pause(5)

EMap = np.loadtxt('EMap.txt')
plt.ion()## 打开交互模式,实现同时打开多个照片
 # 创建一个三维的绘图工程

CamPosition, MarkerROI = DealMarker(Img)
LightColors = TrafficLight(MarkerROI, Img)
print('composition light',CamPosition, LightColors)  # 输出小车位置和交通灯颜色
Img = cv2.resize(Img, (int(Img.shape[1] / 2), int(Img.shape[0] / 2)))
cv2.imshow('Img', Img)
key = cv2.waitKey(5)
if key != -1:
	exit()
if CamPosition is not None:
	ax = plt.subplot()
	Point = ax.scatter(CamPosition[0], CamPosition[1], s=5)
	print('show scatter')
	plt.pause(10)
	Point.remove()