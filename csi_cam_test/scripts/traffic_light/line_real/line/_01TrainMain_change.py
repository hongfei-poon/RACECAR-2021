# -*- coding:utf-8 _*-
"""
@Author  : Xiaoqi Cheng Yutao
@Time    : 2019/11/28 9:33
"""
import logging, os, torch
from Timer import *
from _02PipeDatasetLoader import *
from _03Unet import *
from _04Loss import *
from pytorchtool import EarlyStopping

WeightCoefficient = 2
Lr = 0.01#0.01
Epochs = 300#700
LrDecay = 0.1#0.1
BatchSize = 16  #30
LrDecayPerEpoch = 150  # 学习率调整的epoch500
ValidPerEpoch = 10  # 50测试的epoch
SavePerEpoch = 25  # 保存结果的epoch（原700）
torch.cuda.set_device(-1)  # 选用GPU设备  -1cpu，0gpu

# %% TODO:载入数据，初始化网络，定义目标函数
FolderPath = '../Dataset'
TrainDataset, TrainDataLoader, ValDataset, ValDataLoader = PipeDatasetLoader(FolderPath, BatchSize)
# %% Unet_BCELoss_Adam
Unet = UNet(in_channels=3, out_channels=1, init_features=4, WithActivateLast=True, ActivateFunLast = torch.sigmoid).to('cpu')
SaveFolder = 'Output'
Criterion = nn.BCELoss().to('cpu')
Optimizer = torch.optim.Adam(Unet.parameters(), lr=Lr)
os.makedirs(SaveFolder, exist_ok=SaveFolder)
logging.basicConfig(filename=os.path.join(SaveFolder, 'log.txt'), filemode='w', level=logging.WARNING, format='%(asctime)s %(message)s', datefmt='%Y-%m-%d-%H:%M:%S')
# Unet.load_state_dict(torch.load(os.path.join(SaveFolder, 'PreTrained.pt'), map_location = 'cuda'))
logging.warning('WeightCoefficient:{0:03d}'.format(WeightCoefficient))#不是很懂为什么要设置一个warning1

# initialize the early_stopping object
early_stopping = EarlyStopping(patience=30, verbose=True)
# %% TODO:开始循环训练
#LrScheduler = torch.optim.lr_scheduler.StepLR(Optimizer, step_size=LrDecayPerEpoch, gamma=LrDecay)  # 设置学习率策略
LrScheduler=torch.optim.lr_scheduler.ReduceLROnPlateau(Optimizer,mode='min',factor=LrDecay,patience=15,verbose=True,
													   threshold=0.0001, threshold_mode='rel', cooldown=0, min_lr=0, eps=1e-08)
for Epoch in range(1, Epochs + 1):
	# %% 训练
	Unet.train()  # 训练模式
	# torch.cuda.empty_cache()  # 释放缓存占用, 耗时大概0.05s
	# 训练一个Epoch
	TrainLoss = 0
	#print('Epoch:%d, LR:%.8f ' % (Epoch, LrScheduler.get_lr()[0]), end='>> ', flush=True)
	# print('Epoch:%d, LR:%.8f ' % (Epoch, LrScheduler.get_last_lr()[0]), end='>> ', flush=True)

	print('Epoch:%d' % (Epoch), end='>> ', flush=True)
	for Iter, (InputImg, Label, SampleName) in enumerate(TrainDataLoader):
		print(Iter, end=' ', flush=True)#iter类似于 epoch里面的轮次
		#print(SampleName)
		InputImg = InputImg.float().to('cpu')
		Label = Label.float().to('cpu')
		Weight = Label * (WeightCoefficient-1) + 1
		#print(Weight)
		Criterion.weight = Weight#loss下的参数
		Optimizer.zero_grad()#设置梯度为none
		with torch.set_grad_enabled(True):#是否有梯度参数
			OutputImg = Unet(InputImg)
			BatchLoss = Criterion(OutputImg, Label)
			BatchLoss.backward()#向前传递
			Optimizer.step()#参数优化
			TrainLoss += BatchLoss.item()
	print('len:',TrainDataset.__len__(),BatchSize)
	AveTrainLoss = TrainLoss / TrainDataset.__len__() * BatchSize  # 平均每幅图像的loss
	print(", Total loss is: %.6f" % float(AveTrainLoss))
	#logging.warning('\tTrain\tEpoch:{0:04d}\tLearningRate:{1:08f}\tLoss:{2:08f}'.format(Epoch, LrScheduler.get_last_lr()[0], AveTrainLoss))
	#logging.warning('\tTrain\tEpoch:{0:04d}\tLoss:{2:08f}'.format(Epoch,AveTrainLoss))
	# %% 测试
	if Epoch % ValidPerEpoch == 0 or Epoch == 1:
		Unet.eval()  # 训练模式
		torch.cuda.empty_cache()  # 释放缓存占用
		ValLoss = 0
		print('Validate:', end='>>', flush=True)
		for Iter, (InputImg, Label, SampleName) in enumerate(ValDataLoader):
			print(Iter, end=' ', flush=True)
			InputImg = InputImg.float().to('cpu')
			Label = Label.float().to('cpu')
			Weight = Label * (WeightCoefficient - 1) + 1
			Criterion.weight = Weight
			with torch.set_grad_enabled(False):  # 等同于torch.no_grad()
				OutputImg = Unet(InputImg)
				BatchLoss = Criterion(OutputImg, Label)  # CrossEntropyLoss的Target必须没有通道的维度，即(BatchSize, W, H)
				#比较计算output与label之间的差距
				ValLoss += BatchLoss.item()
		AveValLoss = ValLoss / ValDataset.__len__()
		print("Total loss is: %.6f" % float(AveValLoss))
		#logging.warning('\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tValid\tEpoch:{0:04d}\tLearningRate:{1:08f}\tLoss:{2:08f}'.format(Epoch, LrScheduler.get_last_lr()[0], AveValLoss))

	# %% 保存
	if Epoch % SavePerEpoch == 0:
		torch.save(Unet.state_dict(), os.path.join(SaveFolder, '{0:04d}.pt'.format(Epoch)))

	# %% 每隔一定epoch后更新一次学习率
	LrScheduler.step(AveValLoss)
	valid_loss=AveValLoss
	model=Unet
	# early_stopping needs the validation loss to check if it has decresed,
	# and if it has, it will make a checkpoint of the current model
	early_stopping(valid_loss, model)

	if early_stopping.early_stop:
		print("Early stopping")
		break

