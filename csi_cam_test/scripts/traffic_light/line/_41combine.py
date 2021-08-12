from _31runpt import *
from _31line import find_direct

import cv2


def follow_line(img_after_net, a):
    out_net2 = from_camera_to_net(img_after_net)
    control_di, retl = find_direct(out_net2, a)
    # retl -- retval--0-255
    return control_di, out_net2, retl


if __name__ == '__main__':
    # video=cv2.VideoCapture(0)
    # while(1):
    #     ret,img=video.read()
    #     cv2.imshow('img',img),
    #     cv2.waitKey(0)
    #     test_img=img
    #     follow_line(test_img,0.6)
    img = cv2.imread('1.jpg')
    cv2.imshow('img', img),
    cv2.waitKey(0)
    test_img = img
    control, out_net, ret = follow_line(test_img, 0.6)
    cv2.imshow('out', out_net)
    cv2.waitKey(0)
