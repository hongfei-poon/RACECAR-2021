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
import torch, os
from Timer import *
from _02PipeDatasetLoader import *
from _03Unet import *
from _21CalEvaluationIndicator import *
from _31runpt import *


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

def initialize_net(ModelFolder):
	Device = torch.device("cpu")#调试cpu or gpu
	#Device = torch.device("cuda")
	ModelFolder=ModelFolder
	Unet = UNet(in_channels=3, out_channels=1, init_features=4, WithActivateLast=True,
				ActivateFunLast=torch.sigmoid).to(
		Device)
	Unet.load_state_dict(torch.load(os.path.join(ModelFolder, 'checkpoint.pt'), map_location=Device))
	Unet.eval()  # 测试模式
	torch.set_grad_enabled(False)
	OutputS = []  # 存储检测数据，用于指标计算
	LabelS = []
	Imgpaths='d:\My Documents\桌面\视觉ros\深度学习-图像分割训练\WarpedImg'
	for Imgpath in Imgpaths:
		print(Imgpath)

	for Iter, (Input, Label, SampleName) in enumerate(ValDataLoader):
		end = timer(8)
		print(SampleName)
		InputImg = Input.float().to(Device)
		OutputImg = Unet(InputImg)
		Output = OutputImg.cpu().numpy()[0]
		Label = Label.detach().cpu().numpy()[0]
		cv2.waitKey(0)
		OutputS.append(Output)
		LabelS.append(Label)
		end('5555')
		# 生成效果图
		OutputImg = OutputImg.cpu().numpy()[0, 0]
		OutputImg = (OutputImg * 255).astype(np.uint8)
		Input = Input.numpy()[0][0]
		Input = (Normalization(Input) * 255).astype(np.uint8)
		ResultImg = cv2.cvtColor(Input, cv2.COLOR_GRAY2RGB)
		ori_result = ResultImg.copy()
		ResultImg[..., 2] = OutputImg
		plt.subplot(1, 4, 1), plt.imshow(Label[0])  # Label[0]为输入网络的标记图片
		plt.subplot(1, 4, 2), plt.imshow(ori_result)  # ori_result为输入图片的灰度化后原图
		plt.subplot(1, 4, 3), plt.imshow(ResultImg)  # ResultImg为神经网络标注结果与原图合成出来的图片
		plt.subplot(1, 4, 4), plt.imshow(OutputImg)  # OutputImg为网络的直接输出
		# plt.savefig(os.path.join(SaveFolder+'/'+SampleName[0] + '/'+ 'label.png'))
		plt.show()

def cahge_sezie(img):
	height, width = img.shape[0:2]
	if width > 800:
		new_width = 800
		new_height = int(new_width / width * height)
		img = cv2.resize(img, (new_width, new_height))
		return img

if __name__ == '__main__':
	# ModelFolder1 = './Output'
	#
	# initialize_net(ModelFolder1)

	vision()
