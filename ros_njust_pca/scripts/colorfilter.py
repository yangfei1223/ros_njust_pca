# -*- coding: utf-8 -*-
import os
import cv2
import time
import numpy as np
import copy
import math


size_min = 20 * 20
size_max = 200 * 200

def pad_box(horbox, _ratio):
    '''
    horbox:[[x1,y1],[x2,y1],[x2,y2],[x1,y2]]
    pad box'size by _ratio.
    return: padded horbox, padded size.
    '''
    x_min, y_min = horbox[0]
    x_max, y_max = horbox[2]
    width = x_max - x_min
    height = y_max - y_min
    ctr_x = (x_max + x_min) * 0.5
    ctr_y = (y_max + y_min) * 0.5

    pad_width = float(width) * math.sqrt(_ratio)
    pad_height = float(height) * math.sqrt(_ratio)

    new_x_min = ctr_x - pad_width * 0.5
    new_x_max = ctr_x + pad_width * 0.5
    new_y_min = ctr_y - pad_height * 0.5
    new_y_max = ctr_y + pad_height * 0.5
    li_box=[]
    li_box.append([new_x_min,new_y_min,new_x_max,new_y_max])
    return li_box


def get_size(horbox):
    '''
    horbox:[[x1,y1],[x2,y1],[x2,y2],[x1,y2]]
    return: box's size.
    '''
    x_min, y_min = horbox[0]
    x_max, y_max = horbox[2]
    return (y_max - y_min) * (x_max - x_min)



def detect_box(ori_img,_ratio):
    img = copy.deepcopy(ori_img)
    height, width = img.shape[:2]
    ratio = 3
    img[:height // ratio, :, :] = [0, 0, 0]

    ######## 颜色过滤
    ## BGR转HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_red = np.array([0, 165, 0])
    upper_red = np.array([255, 255, 255])
    mask = cv2.inRange(hsv, lower_red, upper_red)

    blurred = cv2.blur(mask, (9, 9))
    (_, thresh) = cv2.threshold(blurred, 120, 255, cv2.THRESH_BINARY)

    ######## 腐蚀,膨胀，去除噪声
    kernel = np.ones((3, 3), np.uint8)
    erosion = cv2.erode(thresh, kernel, iterations=1)
    kernel = np.ones((1, 1), np.uint8)
    erosion = cv2.erode(erosion, kernel, iterations=4)
    kernel = np.ones((1, 1), np.uint8)
    dilation = cv2.dilate(erosion, kernel, iterations=3)

    ######## 先膨胀再腐蚀，填充前景物体中的小洞
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))  ## 25*25矩形
    dilation = cv2.morphologyEx(dilation, cv2.MORPH_CLOSE, kernel)

    try:
        ######## get box
        _, cnts, _ = cv2.findContours(dilation.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        c = sorted(cnts, key=cv2.contourArea, reverse=True)[0]
        rect = cv2.minAreaRect(c)
        box = np.int0(cv2.boxPoints(rect))

        ######## process box
        Xs = [i[0] for i in box]
        Ys = [i[1] for i in box]
        x1 = min(Xs)
        x2 = max(Xs)
        y1 = min(Ys)
        y2 = max(Ys)
        horbox = [[x1, y1], [x2, y1], [x2, y2], [x1, y2]]
        size = get_size(horbox)
        if size_min <= size <= size_max:
            box = pad_box(horbox, _ratio)
            return box
        else:
            return []
    except Exception as e:
        return []


if __name__ == '__main__':
    imgdir = '/home/njust1/Data/Testdata-001_0/Testdata-001-Camera'
    filelist=os.listdir(imgdir)
    filelist.sort()
    for file in filelist:
        filename=imgdir+'/'+file
        print filename
        org_img=cv2.imread(filename)
        li_box=detect_box(org_img, _ratio=1.25)
        for box in li_box:
            contours=np.array([[box[0],box[1]],[box[2],box[1]],[box[2],box[3]],[box[0],box[3]]]).astype(np.int32)  #x1,y1 x2,y1 x1,y2 x2,y2
            cv2.drawContours(org_img, [contours], -1, (0, 255, 0), 3)
        cv2.imshow("Image", org_img)
        cv2.waitKey(5)
