#!/usr/bin/env python
import numpy as np
import cv2
from cv_bridge import CvBridge
from ros_njust_pca.msg import NJUST_Answer_st_Object

def hist_equal_color(img):
    ycrcb=cv2.cvtColor(img,cv2.COLOR_BGR2YCR_CB)
    channels=cv2.split(ycrcb)
    cv2.equalizeHist(channels[0],channels[0])
    cv2.merge(channels,ycrcb)
    cv2.cvtColor(ycrcb,cv2.COLOR_YCR_CB2BGR,img)
    return img

# fill a object message, input box, object type
def fill_object_msg(stamp,im, box, type):
    msg = NJUST_Answer_st_Object()
    x1 = int(round(box[0]))
    y1 = int(round(box[1]))
    x2 = int(round(box[2]))
    y2 = int(round(box[3]))
    width = x2 - x1
    height = y2 - y1
    msg.timestamp = stamp
    msg.box_type = 0
    msg.box = [x1, y1, x2, y1, x2, y2, x1, y2]
    msg.attribute = type
    msg.longitude = 0.
    msg.latitude = 0.
    msg.image_rows=msg.image_cols=0
    box_im = im[y1:y2, x1:x2, :]
    width=50 if width<50 else width
    width=200 if width>200 else width
    height=50 if height<50 else height
    height=200 if height>200 else height
    msg.image_cols = width
    msg.image_rows = height
    box_im_resize = cv2.resize(box_im, (width, height), cv2.INTER_CUBIC)
    # box_im_resize = hist_equal_color(box_im_resize)
    # kernel=np.array([[-1,-1,-1],[-1,9,-1],[-1,-1,-1]])
    # box_im_resize=cv2.filter2D(box_im_resize,-1,kernel)
    # box_im_resize=cv2.bilateralFilter(box_im_resize,-1,3,3)
    msg.image_data = CvBridge().cv2_to_imgmsg(box_im_resize, encoding='bgr8')
    '''
    # don't restrict size here
    if type in [1,2]:
        if width>=70 and height>=70:
            if width>200:
                width=200
            if height>200:
                height=200
            box_im_resize = cv2.resize(box_im, (width, height), cv2.INTER_LINEAR)
            msg.image_cols = width
            msg.image_rows = height
            msg.image_data = CvBridge().cv2_to_imgmsg(box_im_resize, encoding='bgr8')
    if type==0:
        if width>=20 and height>=30:
            if width>200:
                width=200
            if height>200:
                height=200
            box_im_resize = cv2.resize(box_im, (width, height), cv2.INTER_LINEAR)
            msg.image_cols = width
            msg.image_rows = height
            msg.image_data = CvBridge().cv2_to_imgmsg(box_im_resize, encoding='bgr8')
    '''
    return msg

# publish object message, input object list, object type
def publish_result(pub,stamp,im, li_box, type):
    for box in li_box:
        if all(i>0 for i in box):
            msg=fill_object_msg(stamp,im, box,type)
            pub.publish(msg)
            # print 'publish object succeed, timestam is %d!'%(stamp)

# draw a box on image, input box color, object name
def draw_box(im_draw, li_box, color, str):
    for box in li_box:
        x1 = int(box[0])
        y1 = int(box[1])
        x2 = int(box[2])
        y2 = int(box[3])
        cv2.rectangle(im_draw, (x1, y1), (x2, y2), color, 2)
        cv2.putText(im_draw, str, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)


