# -*- coding:utf-8 _*-
"""
@Author  : Yu Cheng
@Time    : 2019/12/25 9:35
"""
import cv2
import numpy as np
import matplotlib.pyplot as plt
from Timer import *

np.set_printoptions(suppress=True, precision=4)

# ImgPaths = glob.glob("OriImage\\*.jpg")
# H = np.array([[  -0.61869793,   -2.24344654,  672.50410257],
#               [   0.00583877,    0.05218149, -226.93620917],
#               [  -0.00011433,   -0.00451613,    1.        ]])
#
# # 相机参数--update
# Dist = np.array([-0.34419 ,  0.13229  , 0.00300  , 0.00051  ,0.00000], dtype=np.float32)
# K = np.array([[404.25045, 0, 325.48406],
#               [0, 540.27288, 258.93040],
#               [0, 0, 1]], dtype=np.float32)
H = np.array([[-0.6823391, -1.78835825, 712.70187918],
              [-0.01334, 0.49514705, -299.68073631],
              [-0.00017208, -0.00341021, 1.]])
# 相机参数
Dist = np.array([-0.26538, 0.08153, -0.00109, -0.00233, 0.00000], dtype=np.float32)
K = np.array([[331.71415, 0, 321.54719],
              [0, 331.80738, 201.23948],
              [0, 0, 1]], dtype=np.float32)


def warpimg(img_forwarp):
    # img_forwarp= cv2.resize(img_forwarp, (640, 320))
    # print(img_forwarp.shape)
    # undist = timer(5)
    # img_forwarp = cv2.undistort(img_forwarp, K, Dist)
    # undist('time for undist')
    warper = timer(5)
    WarpedImg = cv2.warpPerspective(img_forwarp, H, (1000, 1000))
    #WarpedImg=img_forwarp
    WarpedImg2=cv2.resize(WarpedImg,(160,120))
    cv2.imshow('warp', WarpedImg2)
    cv2.waitKey(1)
    warper('time for warp')
    return WarpedImg


# for ImgPath in ImgPaths:
# 	print(ImgPath)
if __name__ == '__main__':
    Img = cv2.imread('forwarp.jpg')
    a = timer(5)
    # UndistImg = cv2.undistort(Img, K, Dist)  # 处理畸变
    # WarpedImg = cv2.warpPerspective(UndistImg, H, (1000, 1000))
    Wa = warpimg(Img)
    a('finish')
    plt.imshow(Wa)
    plt.show()
    # SavePath = ImgPath.replace('OriImage', 'WarpedImg')
    # os.makedirs(os.path.dirname(SavePath), exist_ok=True)
    cv2.imwrite('00.png', Wa)
# cv.imshow()--0.18
# 不展示--0.012
