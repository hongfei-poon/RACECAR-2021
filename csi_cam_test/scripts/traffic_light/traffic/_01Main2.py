# -*- coding:utf-8 _*-
"""
@Author  : Yu Tao
@Time    : 2020/3/19 11:37 
"""
import numpy as np
np.set_printoptions(suppress=True, precision=4)
import cv2, time, socket, json
import multiprocessing as mp
from _02GStreamer import *

def ImgRead(ImgQueue):
	# %% 从摄像头读取数据
	cam = cv2.VideoCapture(0)
	#cam = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
	if not cam.isOpened():
		print("Unable to open camera")
	else:
		print('Open camera success!')
		while True:
			ret, Img = cam.read()
			if not ret:
				break
			while not ImgQueue.empty():
				ImgQueue.get()
			ImgQueue.put(Img)
			cv2.imshow('ImgRead', Img)
			key = cv2.waitKey(5)
			if key == 27:
				break
		cam.release()

def vision():
	Frame = 0
	ImgQueue = mp.Queue()  # 先进先出队列，实现不同进程数据交互

	Mps = []
	Mps.append(mp.Process(target=ImgRead, args=(ImgQueue,)))
	[Mp.start() for Mp in Mps]
	# Mps[0].join()
	while ImgQueue.empty():
		pass
	while True:
		Key = input('Press s or S to save image:')
		if Key == 's' or Key == 'S':
			Img = ImgQueue.get()
			cv2.imwrite('%04d.jpg' % Frame, Img)
			#cv2.imwrite('%05d.jpg'%Frame,Img)
			print('Save image %04d.jpg success!' % Frame)
			#print('save image %05d.jpg successs'%Frame)
			Frame = Frame + 1
		elif Key == 'Q' or Key == 'q':
			break
	[Mp.terminate() for Mp in Mps]
	#terminate():强制终止进程;不会进行任何清理操作，如果p创建了子进程，该子进程就成了僵尸进程，使用此方法需要小心

if __name__ == '__main__':
	vision()
