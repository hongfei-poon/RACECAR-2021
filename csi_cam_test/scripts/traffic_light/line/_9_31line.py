#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 识别的是中线为白色

import cv2
import numpy as np
from Timer import * 
np.set_printoptions(suppress=True, precision=4)
# center定义

#直接读取图像中间这一行的线条情况
# beishu*y_szie就是具体的行数
def ori_find_diret(img,beishu):
    size_x =640
    size_y = 480
    xcenter = size_x / 2
    # 单看中间行的像素值v
    ycenter = size_y *beishu
    # 打开摄像头，图像尺寸640*480（长*高），opencv存储值为480*640（行*列）
    # cap = cv2.VideoCapture(0)
    white_line = int(ycenter)
    while (1):
        # ret, frame = cap.read()
        frame = img
        frame = cv2.resize(frame, (size_x, size_y))
        # cv2.imshow("img_resize", frame)
        # cv2.waitKey(0)
        # 转化为灰度图
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # cv2.imshow("gray", gray)
        # cv2.waitKey(0)
        # 大津法二值化
        retval, dst = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU)
        # cv2.imshow("dst", dst)
        # cv2.waitKey(0)
        # 腐蚀
        dst = cv2.erode(dst, None, iterations=6)
        # cv2.imshow("erode", dst)
        # cv2.waitKey(0)
        # 膨胀，白区域变大(考虑一个综合函数更快？)
        dst = cv2.dilate(dst, None, iterations=2)
        # cv2.imshow("dilate", dst)
        # cv2.waitKey(0)
        middle=dst.copy
        middle[:][310:320]=255
        # cv2.imshow('middle',middle)
        # cv2.waitKey(0)
        print('white_out',white_line)
        color = dst[white_line]
        try:
            # 找到白色的像素点个数，如寻黑色，0则改为0
            white_count = np.sum(color == 255)
            white_index = np.where(color == 255)
            print('white_count', white_count)
            # 防止white_count=0的报错
            if white_count == 0:
                white_count = 1
                white_line = white_line - int(size_y/10)
                # 跳跃检查巡线底部,变成负数的时候会去检查上半部分
                print(white_line)
                continue
                # 找到黑色像素的中心点位置
            # 计算方法应该是边缘检测，计算白色边缘的位置和/2，即是白色的中央位置。
            line_center = (white_index[0][white_count - 1] + white_index[0][0]) / 2
            # 计算出center与标准中心点的偏移量，因为图像大小是640，因此标准中心是320，因此320不能改。
            direction = line_center - xcenter
            control = int(direction * xcenter / white_line)
            print('direction', direction)
            print('conjtrol', control)
            # direction>0右转  <0左转
        except:
            continue
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        else:
            print('find the direction ')
            break


#读取图像的接近车子的几行
def find_direct(img,beishu):
    #img应该为cv格式
    size_x =640
    size_y = 480
    xcenter = size_x / 2
    # 单看中间行的像素值v
    ycenter = size_y *beishu
    #选取读取白线位置的行
    white_line = int(ycenter)
    #该行列白色的像素个数。用于计算直线中点
    white_count = 0
    #各行偏差初始化0
    sum_direction=0
    #计算direction有效个数
    count_direction=1
    # 打开摄像头，图像尺寸640*480（长*高），opencv存储值为480*640（行*列）
    # cap = cv2.VideoCapture(0)
    #while(1
    # ret, frame = cap.read()
    # 摄像头
    # frame = cv2.imread('222.png')
    # 测试时读取

    #图像处理
    frame=img
    frame = cv2.resize(frame, (size_x, size_y))
    # cv2.imshow("img_resize", frame)
    # cv2.waitKey(0)
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #由于网络的输出已经是灰度图，因此不必再rgb转化为gray
    gray=frame
    # 大津法二值化
    retval, dst = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU)
    # 腐蚀
    dst = cv2.erode(dst, None, iterations=6)
    # 膨胀，白区域变大(考虑一个综合函数更快？)
    dst = cv2.dilate(dst, None, iterations=2)
    # cv2.imshow("dilate", dst)
    # cv2.waitKey(0)
    # middle = dst.copy
    # # i=0
    # # while (i<=480==1):
    # #     i=i+1
    # #     middle[i][320] = 255
    # cv2.imshow('middle', middle)
    # cv2.waitKey(0)
    while(white_line>0):
        # print('white_out',white_line)
        #从240朝向0方向计算白色线条中心点坐标
        color = dst[white_line]
        white_count = np.sum(color == 255)
        if(white_count!=0):
            count_direction=count_direction+1
            white_index = np.where(color == 255)
            line_center = (white_index[0][white_count - 1] + white_index[0][0]) / 2
            # 计算出center与标准中00心点的偏移量，因为图像大小是640，因此标准中心是320，xcenter
            # direction（-320~~320）然后单位化
            direction = 100*(line_center - xcenter)/xcenter
            # print('direction',direction,count_direction)
            #为了避免direction过大，单位化。100意味着最大的位置，接近0偏转小
            sum_direction = (direction + sum_direction)
            sum_direction=float("{0:.3f}".format(sum_direction))
            #没有采用 white_count==0break避免240为0错误跳出
        white_line = white_line - 1
    print('final', count_direction)
    final_sum_direction = sum_direction / count_direction
    print('sum',final_sum_direction)
    return  sum_direction

    # while (white_count==0):
    #     print('white_out',white_line)
    #     color = dst[white_line]
    #     try:
    #         # 找到白色的像素点个数，如寻黑色，0则改为0
    #         white_count = np.sum(color == 255)
    #         white_index = np.where(color == 255)
    #         print('white_count', white_count)
    #         # 防止white_count=0的报错
    #         if white_count == 0:
    #             white_line = white_line - int(size_y/25)
    #             # 跳跃检查巡线底部,变成负数的时候会去检查上半部分
    #             print(white_line)
    #             continue
    #             # 找到黑色像素的中心点位置
    #         # 计算方法应该是边缘检测，计算白色边缘的位置和/2，即是白色的中央位置。
    #         line_center = (white_index[0][white_count - 1] + white_index[0][0]) / 2
    #         # 计算出center与标准中心点的偏移量，因为图像大小是640，因此标准中心是320，xcenter
    #         #direction（-320~~320）然后单位化
    #         direction =line_center - xcenter
    #         control = direction/ white_line
    #         #可以求反三角得出中州与偏移位点的角度
    #         print('direction', direction)
    #         print('conjtrol', control)
    #         # direction>0右转  <0左转
    #     except:
    #         continue
    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         break
    #     else:
    #         print('find the direction ')
    #         break


if __name__=='__main__':
    time_line0=timer(6)
    frame = cv2.imread('img.png')
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    print(type(frame))
    find_direct(frame, 0.5)
    time_line0('timei_line0')
#0087 9
#0029 -20
#0042 -0.085
#0076 -13
#0094  4
#0040  0.066
#0041  0.041
#0042  -23
#0036   17
#0037   16
#221    21