#!/usr/bin/env python3
# -*- coding:utf-8 _*-
# 识别的是中线为白色

import cv2
import numpy as np
from Timer import *
import sys

np.set_printoptions(suppress=True, precision=4)


# center定义

# 直接读取图像中间这一行的线条情况

# 读取图像的接近车子的几行
def find_direct(img, beishu):
    cal_line = timer(8)
    # img应该为cv格式
    size_x = 320
    size_y = 240
    xcenter = size_x / 2
    # 单看中间行的像素值v
    ycenter = size_y * beishu
    # 选取读取白线位置的行
    white_line = int(ycenter)
    # 该行列白色的像素个数。用于计算直线中点
    # 各行偏差初始化0
    sum_direction = 0
    # 计算direction有效个数
    count_direction = 1

    # 图像处理

    frame = cv2.resize(img, (size_x, size_y))
    # frame_mini = cv2.resize(frame, (int(frame.shape[1] / 3), int(frame.shape[0] / 3)))
    # cv2.imshow("img_line_resize", frame_mini)
    # cv2.waitKey(1)
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # 由于网络的输出已经是灰度图，因此不必再rgb转化为gray
    # 大津法二值化
    retval, dst = cv2.threshold(frame, 0, 255, cv2.THRESH_OTSU)
    # 腐蚀
    dst = cv2.erode(dst, None, iterations=2)
    cv2.imshow('erode',dst)
    # 膨胀，白区域变大(考虑一个综合函数更快？)
    # dst = cv2.dilate(dst, None, iterations=2)
    # cv2.imshow("dilate", dst)
    cv2.waitKey(1)
    # middle = dst.copy

    while white_line > 0:
        # print('white_out',white_line)
        # 从240行朝着0方向计算白色线条中心位置
        color = dst[white_line]
        white_count = np.sum(color == 255)
        # white——count：这一行白色像素点的数目
        if white_count != 0:
            count_direction = count_direction + 1
            white_index = np.where(color == 255)
            line_center = (white_index[0][white_count - 1] + white_index[0][0]) / 2
            # 计算出center与标准中00心点的偏移量，因为图像大小是640，因此标准中心是320，xcenter
            # 单行direction（-320~~320）然后单位化，
            #/xcenter is error    should /
            direction = 100 * (line_center - xcenter) / white_line
            # print('direction',direction,count_direction)..
            # 为了避免direction过大，单位化。100意味着最大的位置，接近0偏转小.大于0右转
            sum_direction = (direction + sum_direction)
            sum_direction = float("{0:.3f}".format(sum_direction))
            # 没有采用 white_count==0  break避免240为0错误跳出
        white_line = white_line - 1

    print('final_count_direction ', count_direction)
    average_direction = sum_direction / count_direction
    print('average direction', average_direction)
    cal_line('finish cal line')
    return average_direction,retval


if __name__ == '__main__':
    ti = timer(8)
    frame = cv2.imread('img.png')
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    print(type(frame))
    find_direct(frame, 0.5)
    ti('finish')
    # 倍数0-1，1为远距离；倍数最好大于0.4，因为图片接近车子部分被变化去除掉了
# 0087 9
# 0029 -20
# 0042 -0.085
# 0076 -13
# 0094  4
# 0040  0.066
# 0041  0.041
# 0042  -23
# 0036   17
# 0037   16
# 221    21
