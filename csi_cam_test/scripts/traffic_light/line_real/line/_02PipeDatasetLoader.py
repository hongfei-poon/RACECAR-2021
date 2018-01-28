# -*- coding:utf-8 _*-
"""
@Author  : Xiaoqi Cheng Yutao
@Time    : 2019/10/23 19:40
"""
import torch, os, cv2
from torch.utils.data import Dataset, DataLoader
from torchvision.transforms import transforms
import matplotlib.pyplot as plt
from PIL import Image
import numpy as np
import random

unloader = transforms.ToPILImage()
def tensor_to_PIL(tensor):
	image = tensor.cpu().clone()
	image = image.squeeze(0)
	image = unloader(image)
	return image
InputImgSize=(128,128)
# %% 训练过程图片的变换
TrainImgTransform = transforms.Compose([
	# transforms.RandomOrder,
	# transforms.ColorJitter(),
	transforms.RandomAffine(degrees=(-10, 10), translate=(0.1, 0.1), scale=(0.5, 2.), shear=10),
	transforms.RandomHorizontalFlip(),
	# transforms.RandomVerticalFlip(),
	transforms.RandomRotation(20),
	transforms.RandomResizedCrop(InputImgSize, scale=(1., 1.), interpolation=Image.BILINEAR),
	transforms.ToTensor(),
	transforms.Normalize(mean=[0.46], std=[0.10]),
	#transforms.Normalize(mean=[0.19838604,0.19140925,0.18471918], std=[0.1604704,0.13926657,0.1375487 ]),
])
TrainLabelTransform = transforms.Compose([
	# transforms.RandomOrder,
	# transforms.ColorJitter(),
	transforms.RandomAffine(degrees=(-10, 10), translate=(0.1, 0.1), scale=(0.5, 2.), shear=10),
	transforms.RandomHorizontalFlip(),
	# transforms.RandomVerticalFlip(),
	transforms.RandomRotation(20),
	transforms.RandomResizedCrop(InputImgSize, scale=(1., 1.), interpolation=Image.NEAREST),
	# transforms.RandomResizedCrop(InputImgSize, scale=(1., 1.)),
	transforms.ToTensor(),
])

# %% 测试过程图片变换
ValImgTransform = transforms.Compose([
	transforms.Resize(InputImgSize),
	transforms.ToTensor(),
	transforms.Normalize(mean=[0.46], std=[0.10]),
])
ValLabelTransform = transforms.Compose([
	transforms.Resize(InputImgSize, interpolation=Image.NEAREST),
	transforms.ToTensor(),
])

class PipeDataset(Dataset):
	def __init__(self, DatasetFolderPath, ImgTransform, LabelTransform, ShowSample=False):#ShowSample=False
		self.DatasetFolderPath = DatasetFolderPath
		self.ImgTransform = ImgTransform
		self.LabelTransform = LabelTransform
		self.ShowSample = ShowSample
		self.SampleFolders = os.listdir(self.DatasetFolderPath)#返回指定路径下的文件和文件夹列表

	def __len__(self):
		return len(self.SampleFolders)

	def __getitem__(self, item):
		SampleFolderPath = os.path.join(self.DatasetFolderPath, self.SampleFolders[item])  #[存储样本文件夹地址]+[文件名字] 样本文件夹路径
		FusionImgPath = os.path.join(SampleFolderPath, 'img.png')
		LabelImgPath = os.path.join(SampleFolderPath, 'label.png')
		FusionImg = Image.open(FusionImgPath)
		LabelImg = Image.open(LabelImgPath)
		LabelImg = np.array(LabelImg)*255
		LabelImg = Image.fromarray(LabelImg)#array到image的转换
		FusionImg2=FusionImg.copy()
		#print('1fusion',FusionImg.size)
		# 保证样本和标签具有相同的变换
		# seed = np.random.randint(2147483647)
		# random.seed(seed)
		# FusionImg = self.ImgTransform(FusionImg)
		# # plt.subplot(1,2,1)
		# # plt.imshow(FusionImg)
		# # plt.show()
		# random.seed(seed)
		# LabelImg = self.LabelTransform(LabelImg)
		# plt.imshow(tensor_to_PIL(LabelImg))
		# # plt.show()

		#修正结果
		torch.manual_seed(2147483647)
		FusionImg = self.ImgTransform(FusionImg)
		torch.manual_seed(2147483647)
		LabelImg = self.LabelTransform(LabelImg)

		# %% 显示Sample
		if self.ShowSample:
			plt.figure(self.SampleFolders[item])
			Img = FusionImg.numpy()[0]
			Label = LabelImg.numpy()[0]

			Img = (Normalization(Img) * 255).astype(np.uint8)
			Label = (Normalization(Label) * 255).astype(np.uint8)
			Img = cv2.cvtColor(Img, cv2.COLOR_GRAY2RGB)
			Img[..., 2] = Label
			plt.subplot(1, 2, 1)
			plt.imshow(FusionImg2)
			plt.subplot(1, 2, 2)
			plt.imshow(Img)
			plt.show()

		#print('shape of fusion',FusionImg.shape)
		return FusionImg, LabelImg, self.SampleFolders[item]


def PipeDatasetLoader(FolderPath, BatchSize=1, ShowSample=False):
	TrainFolderPath = os.path.join(FolderPath, 'Train')
	TrainDataset = PipeDataset(TrainFolderPath, TrainImgTransform, TrainLabelTransform, ShowSample)
	TrainDataLoader = DataLoader(TrainDataset, batch_size=BatchSize, shuffle=True, drop_last=False, num_workers=0, pin_memory=True)

	ValFolderPath = os.path.join(FolderPath, 'Val')
	ValDataset = PipeDataset(ValFolderPath, ValImgTransform, ValLabelTransform, ShowSample)
	ValDataLoader = DataLoader(ValDataset, batch_size=1, shuffle=False, drop_last=False, num_workers=0, pin_memory=True)
	return TrainDataset, TrainDataLoader, ValDataset, ValDataLoader


def Normalization(Array):  # 数组归一化到0~1
	min = np.min(Array)
	max = np.max(Array)
	if max - min == 0:
		return Array
	else:
		return (Array - min) / (max - min)


if __name__ == '__main__':
	FolderPath = '..\Dataset'
	TrainDataset, TrainDataLoader, ValDataset, ValDataLoader = PipeDatasetLoader(FolderPath, BatchSize=1, ShowSample=False)
	for epoch in range(1):
		for i, (Img, Label, SampleName) in enumerate(TrainDataLoader):
			print('samplename',SampleName)
			print('imgshape',Img.shape)
			print(Label.max())

