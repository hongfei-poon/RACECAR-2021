# -*- coding:utf-8 _*-
"""
@author:YuTao
@time: 2019/07/01
"""
import numpy as np
import cv2
import cv2.aruco as aruco

# cv2.aruco 存放在 opencv -python contri
#  opencv-contrib-python

# dict = aruco.getPredefinedDictionary(aruco.DICT_7X7_1000)  # 编码点的类型,与生成的时候对应 7x7 内有1000个marker 创建字典对象
dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
K = np.array([[228.82, 0, 601.32],
              [0, 228.59, 363.39],
              [0, 0, 1]])  # 内参-相机图相焦距-标定测算
Dis = np.array([0.00490, 0.00105, 0.00012, 0.00047])  # 畸变-镜头 通过标定测算
EMap = np.loadtxt('/home/scut/catkin_ws/src/csi_cam_test/scripts/traffic_light/traffic/EMap.txt')
#EMap = np.loadtxt('EMap.txt')


# 解算位姿的函数
def CalculatePositon(Point3D, Point2D, K, Dis):
    '''
   axis=0，那么输出矩阵是1行，求每一列的平均（按照每一行去求平均）；axis=1，输出矩
   阵是1列，求每一行的平均（按照每一列去求平均）。还可以这么理解，axis是几，那就表
   明哪一维度被压缩成1。
	'''
    Center = np.mean(Point3D, axis=0)
    Point3D = Point3D - Center  # 去中心化
    ret, RvecW2C, tW2C = cv2.solvePnP(Point3D, Point2D, K, Dis)  # 解算位姿
    # print('cv2.soleve is ok')
    '''
	R:旋转矩阵，T:平移矩阵
	objPoints：N*3或者3*N的世界坐标系点坐标
   	imagePoints：N*2或者2*N的图像坐标系点的对应坐标
   	k-cameraMatrix,distCoeffs：相机的projection和distortion，通过多张图片标定得到
   	cameraMatrix: 内参矩阵
   	https://blog.csdn.net/aptx704610875/article/details/48914043
    distCoeffs: 畸变矩阵(默认获得5个即便参数k1,k2,p1,p2,k3,可修改) 
	'''
    RW2C = cv2.Rodrigues(RvecW2C)[0]
    RC2W = np.linalg.inv(RW2C)
    tC2W = -np.linalg.inv(RW2C).dot(tW2C)
    # print('before calculation ')
    CamPosition = tC2W.flatten() + Center  # 相机在世界坐标系下的坐标
    return CamPosition


def DealMarker(Img):
    CamPosition = None
    MarkerROI = None
    Corners, IDs, _ = aruco.detectMarkers(Img, dict)  # corner 每一个marker的四角；ids 每个maeker序列 _标记物检测参数
    if Img is None:
        print('error in img')
    if len(Corners) < 2:
        print('error in corner detect')
    if len(Corners) == 2:  # 如果检测点 四个角的标识点 存储在数组[不是很清楚为什么会设置成4，因为实际上检测交通灯是2个焦点]
        Point3D = np.empty((0, 3))
        Point2D = np.empty((0, 2))  # 0x2
        for i, Corner in enumerate(Corners):
            # enumerate() 函数用于将一个可遍历的数据对象(如列表、元组或字符串)组合为一个索引序列，同时列出数据和数据下标，
            # Point2D = np.vstack((Point2D, Corner.reshape(-1, 2)))#np.vstack:按垂直方向（行顺序）堆叠数组构成一个新的数组
            # .reshape(-1, 2)将此矩阵或者数组重组，以 c行2列的形式表示
            if IDs.flatten()[i] <= 1:
                Point2D = np.vstack((Point2D, Corner.reshape(-1, 2)))
                ID = IDs.flatten()[i]  # 返回一个折叠成一维的数组。
                Point3D = np.vstack((Point3D, np.hstack((EMap[ID, 3:].reshape(-1, 2), np.zeros((4, 1))))))  # 竖直拼接
            # print(Point3D)
        # CamPosition = CalculatePositon(Point3D, Point2D, K, Dis)  # in 02
        CamPosition = [0, 1, 0]
        aruco.drawDetectedMarkers(Img, Corners, IDs)
        '''
		#一个在输入图像中绘制检测到的标记的功能
		#image是输入/输出图像，其中将绘制标记（它通常与检测到标记的图像相同）。
		#markerCorners和markerIds是detectMarkers()函数提供的相同格式的检测到的标记的结构
		'''
        cv2.putText(Img, str(CamPosition), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 3)
        # cv2.各参数依次是：图片，添加的文字，左上角坐标，字体，字体大小，颜色，字体粗细
        MarkerROI = np.hstack((np.min(Point2D, axis=0), np.max(Point2D, axis=0))).astype(
            np.int)  # xmin,ymin,xmax,ymax 获得的整形...
    # roi指获得感兴趣区域

    return CamPosition, MarkerROI

# 不确定获得的距离参数是否正确
