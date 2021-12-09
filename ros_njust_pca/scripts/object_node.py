#!/usr/bin/env python
import os
import sys
import threading
import numpy as np
import cv2
from cv_bridge import CvBridge
import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from ros_njust_pca.msg import NJUST_Answer_st_Object
import tools
this_dir=os.path.dirname(__file__)
sys.path.append(this_dir+'/frcnn/tools')
import demo
# import colorfilter

global g_pub
g_flag = False
g_stamp = 0
g_im = np.zeros((1024, 768, 3), np.uint8)
g_mutex = threading.Lock()


def is_new():
    flag = False
    if g_mutex.acquire():
        flag = g_flag
    g_mutex.release()
    return flag


def ImageCallback(msg):
    timestamp = int(msg.header.stamp.to_sec() * 1000)
    image = CvBridge().imgmsg_to_cv2(msg)
    # print 'receive image message, timestamp is %d !' % (timestamp)
    global g_stamp, g_im, g_flag
    if g_mutex.acquire():
        g_flag = True
        g_stamp = timestamp
        g_im = image
        g_mutex.release()



def main():
    rospy.init_node('object_node')
    prototxt=rospy.get_param('~frcnnNetworkFile',default='/home/yangfei/NJUST_KYXZ2018_G/src/ros_njust_pca/scripts/frcnn/models/coco/VGG16/faster_rcnn_end2end/test.prototxt')
    caffemodel=rospy.get_param('~frcnnModelFile',default='/home/yangfei/NJUST_KYXZ2018_G/src/ros_njust_pca/scripts/frcnn/data/faster_rcnn_models/coco_vgg16_faster_rcnn_final.caffemodel')
    sub = rospy.Subscriber('NJUST_Sensor/Image', Image, ImageCallback,queue_size=1)
    pub = rospy.Publisher('NJUST_Result/Object', NJUST_Answer_st_Object, queue_size=10)
    im_pub = rospy.Publisher('NJUST_Debug/Object', Image, queue_size=1)
    demo.initialize(prototxt,caffemodel)
    rate = rospy.Rate(10)
    stamp = 0
    im = np.zeros((768, 1024, 3), np.uint8)
    # input_im=np.zeros((768, 1024, 3), np.uint8)
    global g_flag
    while not rospy.is_shutdown():
        if is_new():
            global g_flag
            if g_mutex.acquire():
                g_flag = False
                stamp = g_stamp
                im = g_im
                g_mutex.release()
            # input_im[300:, :, :]=im[300:, :, :]
            li_person, li_car = demo.donext(im)
            # li_box=colorfilter.detect_box(im,_ratio=1.25)
            tools.publish_result(pub, stamp, im, li_person, 0)
            tools.publish_result(pub, stamp, im, li_car, 1)
            # tools.publish_result(pub, stamp, im, li_box, 2)
            tools.draw_box(im, li_person, (255, 0, 0), 'Person')
            tools.draw_box(im, li_car, (0, 255, 0), 'Car')
            # tools.draw_box(im, li_car, (0, 0, 255), 'Box')
            msg = CvBridge().cv2_to_imgmsg(im, 'bgr8')
            im_pub.publish(msg)
    rate.sleep()


if __name__ == '__main__':
    try:
        print 'start python person and car detection node.'
        main()
    except rospy.ROSInterruptException:
        pass
    print 'exit'
