#!/usr/bin/env python3
# -*- coding:utf-8 _*-
# 识别的是中线为白色

import cv2
import numpy as np
from Timer import *
import sys
import math

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
    white_line_top = int(ycenter)
    # 该行列白色的像素个数。用于计算直线中点
    # 各行偏差初始化0
    white_line = 1
    final_count = 1
    sum_direction = 0
    before_sum_direction = 0
    # 计算direction有效个数
    count_direction = 1

    frame = cv2.resize(img, (size_x, size_y))
    # frame_mini = cv2.resize(frame, (int(frame.shape[1] / 3), int(frame.shape[0] / 3)))
    # cv2.imshow("img_line_resize", frame_mini)
    # cv2.waitKey(1)

    # 大津法二值化
    retval, dst = cv2.threshold(frame, 0, 255, cv2.THRESH_OTSU)

    # 腐蚀
    dst = cv2.erode(dst, None, iterations=2)

    # 膨胀，白区域变大(考虑一个综合函数更快？)
    # dst = cv2.dilate(dst, None, iterations=2)
    # cv2.imshow("dilate", dst)

    err = 0
    count_err = 0
    while white_line_top >= white_line > 0:
        # print('white_out',white_line)
        # from y:0 to y:beishu*y_max
        color = dst[white_line]
        white_count = np.sum(color == 255)
        # white——count：这一行白色像素点的数目
        if white_count != 0:
            white_index = np.where(color == 255)
            if (white_index[0][white_count - 1] - white_index[0][0]) < int(size_x / 5):
                count_direction = count_direction + 1
                line_center = (white_index[0][white_count - 1] + white_index[0][0]) / 2
                # 计算出center与标准中00心点的偏移量，因为图像大小是640，因此标准中心是320，xcenter
                # 单行direction（-320~~320）然后单位化，
                line_cal = white_line

                direction = (line_center - xcenter) / (xcenter * 1.00)

                # print('======', direction, 'direction======', 'white_line', white_line, line_cal)

                # 为了避免direction过大，单位化。100意味着最大的位置，接近0偏转小.大于0右转
                sum_direction = (direction + sum_direction)

                sum_direction = float("{0:.4f}".format(sum_direction))
                # 没有采用 white_count==0  break避免240为0错误跳出d
                final_count = white_line

            if (white_index[0][white_count - 1] - white_index[0][0]) > int(size_x / 5):
                # print(white_index[0][white_count - 1], white_index[0][0], int(size_x / 5), retval,end='')
                # print('out of line ')
                count_err = count_err + 1
            if retval < 15:
                count_err = count_err + 1
            else:
                count_err = count_err

        white_line = white_line + 1
    print('retval===========', retval,'count_err=============',count_err)
    if retval < 15:
        err = 1
    if count_err > 15:
        err = 1
    if sum_direction == 0:
        err = 1
        print('no thing in img')
    print('err,count_err', err, count_err)
    if err == 1:
        print('error---------------------------------')

    # print('final_count_direction ', count_direction)
    average_direction = 100 * sum_direction / (1.00 * count_direction)

    if average_direction == 0:
        err = 1
    # if err == 1:
    #     average_direction=last_direction
    # if err==0:
    #     last_direction=average_direction

    # print('average direction', average_direction)

    cv2.putText(dst, str((retval, average_direction, err)), (10, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3,
                (255, 255, 250), 1)
    dst[int(white_line), :] = 128
    dst[:, int(xcenter)] = 128
    dst[int(final_count), :] = 128
    cal_line('finish cal line')
    show=timer(8)
    cv2.imshow('erode', dst)
    cv2.waitKey(1)
    show('show erode img took')
    return average_direction, retval, err, dst


def find_direct0(img, beishu):
    count_err = 0
    err = 0
    # cal_line = timer(8)
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
    dst = cv2.erode(dst, None, iterations=1)

    # 膨胀，白区域变大(考虑一个综合函数更快？)
    # dst = cv2.dilate(dst, None, iterations=2)
    # cv2.imshow("dilate", dst)

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
            # /xcenter is error    should /white line
            line_cal = max(white_line, 1)
            # direction =  10.00*(line_center - xcenter) /(xcenter*1.0)* (white_line+1)/(ycenter*1.0)
            direction = 10.00 * (line_center - xcenter) / (xcenter * 1.00)
            # direction = math.atan(direction)
            # print('======', 180 * direction / 3.141, 'direction======', 'white_line', white_line, line_cal)
            # print('direction',direction,count_direction)..
            # 为了避免direction过大，单位化。100意味着最大的位置，接近0偏转小.大于0右转
            sum_direction = (direction + sum_direction)
            sum_direction = float("{0:.6f}".format(sum_direction))
            # 没有采用 white_count==0  break避免240为0错误跳出
            if (white_index[0][white_count - 1] - white_index[0][0]) > int(size_x / 4):
                print(white_index[0][white_count - 1], white_index[0][0], int(size_x / 4), retval, 'out of line')
                count_err = count_err + 2
                if retval < 14:
                    count_err = count_err + 2
            else:
                count_err = count_err
        white_line = white_line - 1
    if retval < 10:
        print('low in ret')
        count_err = count_err + 20
    if retval < 5:
        count_err = count_err + 20
    if retval > 60:
        count_err = 0

    if retval < 8:
        err = 1
        # print('error in one img ,try to find line')
    if count_err > 30:
        err = 1
        # cv2.waitKey(0)
    if err == 1:
        print('error in one img ,try to find line-------')

    # print('final_count_direction ', count_direction)
    # average_direction = 180 * sum_direction / count_direction / 3.14159
    average_direction = 10 * sum_direction / count_direction
    print('average direction', average_direction)
    # cal_line('finish cal line')
    showdst = cv2.resize(dst, (120, 80))
    cv2.putText(showdst, str((retval, average_direction, err)), (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.3,
                (255, 255, 250), 1)
    cv2.imshow('erode', showdst)
    cv2.waitKey(1)

    return average_direction, retval, err


def find_direct_tan_arc_tan(img, beishu):
    count_err = 0
    err = 0
    # cal_line = timer(8)
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
    dst = cv2.erode(dst, None, iterations=1)

    # 膨胀，白区域变大(考虑一个综合函数更快？)
    # dst = cv2.dilate(dst, None, iterations=2)
    # cv2.imshow("dilate", dst)

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
            # /xcenter is error    should /white line
            line_cal = max(white_line, 1)
            # direction =  10.00*(line_center - xcenter) /(xcenter*1.0)* (white_line+1)/(ycenter*1.0)
            direction = 1.00 * (line_center - xcenter) / (line_cal * 1.00)
            direction = math.atan(direction)
            # print('======', 180 * direction / 3.141, 'direction======', 'white_line', white_line, line_cal)
            # print('direction',direction,count_direction)..
            # 为了避免direction过大，单位化。100意味着最大的位置，接近0偏转小.大于0右转
            sum_direction = (direction + sum_direction)
            sum_direction = float("{0:.6f}".format(sum_direction))
            # 没有采用 white_count==0  break避免240为0错误跳出
            if (white_index[0][white_count - 1] - white_index[0][0]) > int(size_x / 4):
                print(white_index[0][white_count - 1], white_index[0][0], int(size_x / 4), retval, 'out of line')
                count_err = count_err + 2
                if retval < 14:
                    count_err = count_err + 2
            else:
                count_err = count_err
        white_line = white_line - 1
    if retval < 10:
        print('low in ret')
        count_err = count_err + 20
    if retval < 5:
        count_err = count_err + 20
    if retval > 60:
        count_err = 0

    if retval < 8:
        err = 1
        # print('error in one img ,try to find line')
    if count_err > 30:
        err = 1
        # cv2.waitKey(0)
    if err == 1:
        print('error in one img ,try to find line-------')

    # print('final_count_direction ', count_direction)
    # average_direction = 180 * sum_direction / count_direction / 3.14159
    average_direction = math.tan(sum_direction / count_direction) * 50
    print('average direction', average_direction)
    # cal_line('finish cal line')
    showdst = cv2.resize(dst, (120, 80))
    cv2.putText(showdst, str((retval, average_direction, err)), (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.3,
                (255, 255, 250), 1)
    cv2.imshow('erode', showdst)
    cv2.waitKey(1)

    return average_direction, retval, err


if __name__ == '__main__':
    ti = timer(8)
    frame = cv2.imread('img.png')
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    print(type(frame))
    a, b, c,d = find_direct(frame, 0.5)
    print(a, b, c)
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
