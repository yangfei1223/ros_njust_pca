#!/usr/bin/env python
import os
import sys
import threading
import numpy as np
import cv2
from cv_bridge import CvBridge
import rospy
from std_msgs.msg import Header
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
import tools
this_dir=os.path.dirname(__file__)
sys.path.append(this_dir+'/ENet')
import test_segmentation_demo as test

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
    rospy.init_node('segmentation_node')
    prototxt=rospy.get_param('~enetNetworkFile',default='/home/yangfei/NJUST_KYXZ2018_G/src/ros_njust_pca/scripts/ENet/final_model_weigths/bn_conv_merged_model.prototxt')
    caffemodel=rospy.get_param('~enetModelFile',default='/home/yangfei/NJUST_KYXZ2018_G/src/ros_njust_pca/scripts/ENet/final_model_weigths/bn_conv_merged_weights.caffemodel')
    sub = rospy.Subscriber('NJUST_Sensor/Image', Image, ImageCallback,queue_size=1)
    pub = rospy.Publisher('NJUST_Result/Segmentation',Image,queue_size=1)
    # pub_img=rospy.Publisher('NJUST_Debug/Segmentation',Image,queue_size=1)
    test.initialize(prototxt,caffemodel)
    rate = rospy.Rate(10)
    stamp = 0
    im = np.zeros((768, 1024, 3), np.uint8)
    kernel1 = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))
    kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (50, 50))
    global g_flag
    while not rospy.is_shutdown():
        if is_new():
            global g_flag
            if g_mutex.acquire():
                g_flag = False
                stamp = g_stamp
                im = g_im
                g_mutex.release()
            pred = test.donext(im)
            pred = cv2.morphologyEx(pred, cv2.MORPH_CLOSE, kernel1)
            pred = cv2.morphologyEx(pred, cv2.MORPH_OPEN, kernel1)
            pred=cv2.dilate(pred,kernel2)
            msg = CvBridge().cv2_to_imgmsg(pred*255, 'mono8')
            # img_msg =CvBridge().cv2_to_imgmsg(pred,'bgr8')
            header=Header()
            header.stamp=rospy.Time.from_sec(stamp/1000.0)
            msg.header=header
            # img_msg.header=header
            pub.publish(msg)
            # pub_img.publish(img_msg)
            # print 'publish segmentation result succeed !'
    rate.sleep()


if __name__ == '__main__':
    try:
        print 'start python segmentation node.'
        main()
    except rospy.ROSInterruptException:
        pass
    print 'exit'