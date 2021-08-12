#-*- coding:utf-8 _*-
"""
@Author  : Xiaoqi Cheng Yutao
@Time    : 2019/12/20 9:52
"""
import torch, os
from Timer import *
from _02PipeDatasetLoader import *
from _03Unet import *
from _21CalEvaluationIndicator import *

Device = torch.device("cpu")

# %% 载入数据、模型
FolderPath = '..\Dataset'
#FolderPath = 'E:\\Python\\2019.05.13 159YT\\2019.11.11SRoadSegmentation\\Dataset'
TrainDataset, TrainDataLoader, ValDataset, ValDataLoader = PipeDatasetLoader(FolderPath, 1)
# Unet_BCELoss_Adam
ModelFolder = './Output'
SaveFolder = './Dataset/Val'
Unet = UNet(in_channels=3, out_channels=1, init_features=4, WithActivateLast = True, ActivateFunLast = torch.sigmoid).to(Device)
Unet.load_state_dict(torch.load(os.path.join(ModelFolder, 'checkpoint.pt'), map_location = Device))#700

# %% 测试
Unet.eval()         # 测试模式
torch.set_grad_enabled(False)
OutputS = []        # 存储检测数据，用于指标计算
LabelS = []
for Iter, (Input, Label, SampleName) in enumerate(ValDataLoader):
	print(Input.shape)
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
	OutputImg = (OutputImg*255).astype(np.uint8)
	Input = Input.numpy()[0][0]
	Input = (Normalization(Input) * 255).astype(np.uint8)
	ResultImg = cv2.cvtColor(Input, cv2.COLOR_GRAY2RGB)
	ori_result=ResultImg.copy()
	ResultImg[...,2] = OutputImg

	cv2.imwrite(os.path.join(ModelFolder,SampleName[0] + '.png'), ResultImg)#, SampleName[0] + '/' + SampleName[0] + '.png'
	plt.subplot(1,4,1),plt.imshow(Label[0])#Label[0]为输入网络的标记图片
	plt.subplot(1, 4, 2),plt.imshow(ori_result)#ori_result为输入图片的灰度化后原图
	plt.subplot(1,4,3),plt.imshow(ResultImg)#ResultImg为神经网络标注结果与原图合成出来的图片
	plt.subplot(1, 4, 4),plt.imshow(OutputImg)#OutputImg为网络的直接输出
	#plt.savefig(os.path.join(SaveFolder+'/'+SampleName[0] + '/'+ 'label.png'))
	#plt.show()

# %% 计算指标
OutputFlatten = np.vstack(OutputS).ravel()
LabelFlatten = np.vstack(LabelS).ravel()# np.vstack垂直方向堆叠，ravel变为一维，修改原矩阵
#%% ROC, AUC  #根据IOU计算TP,FP
fpr, tpr, AUC = ROC_AUC(LabelFlatten, OutputFlatten, ShowROC = True)
print('AUC:', AUC)#AUC用于反应泛化能力
recall, precision, MF, AP = PRC_AP_MF(LabelFlatten, OutputFlatten, ShowPRC = True)
print('MF:', MF)#MF综合反应recall与precision
print('AP:', AP)#AP反应含有目标的图片数，精确度
plt.show()



