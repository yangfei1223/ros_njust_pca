#!/usr/bin/env python
import os
import sys
import threading
import numpy as np
from cv_bridge import CvBridge
import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from ros_njust_pca.msg import NJUST_Answer_st_Object
import tools
this_dir=os.path.dirname(__file__)
sys.path.append(this_dir+'/svm_box')
# import svm_box
# import colorfilter
import colorfilter_svm_dir as color_svm

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
    model_path=rospy.get_param('~svmModelFile',default='/home/njust1/catkin_ws/src/ros_njust_pca/scripts/svm_box/models/20pixel/svm_20pixel_pca_200_pso.model')
    mean_path=rospy.get_param('~meanValFile',default='/home/njust1/catkin_ws/src/ros_njust_pca/scripts/svm_box/features/PCA/20pixel/meanVal_train_20pixel.mean')
    eigen_path=rospy.get_param('~eigenValFile',default='/home/njust1/catkin_ws/src/ros_njust_pca/scripts/svm_box/features/PCA/20pixel/n_eigVects_train_20pixel_200.eig')
    sub = rospy.Subscriber('NJUST_Sensor/Image', Image, ImageCallback,queue_size=1)
    pub = rospy.Publisher('NJUST_Result/Object', NJUST_Answer_st_Object, queue_size=10)
    im_pub = rospy.Publisher('NJUST_Debug/Box', Image, queue_size=1)
    # svm_box.initialize(model)
    color_svm.initialize(model_path,mean_path,eigen_path)
    rate = rospy.Rate(10)
    stamp = 0
    im = np.zeros((768, 1024, 3), np.uint8)
    global g_flag
    while not rospy.is_shutdown():
        if is_new():
            if g_mutex.acquire():
                g_flag = False
                stamp = g_stamp
                im = g_im
                g_mutex.release()
            # li_box = svm_box.donext(im)   # color filter
            # li_box=colorfilter.detect_box(im,_ratio=1.25)     # svm
            li_box=color_svm.donext(im)     # color filter + svm
            tools.publish_result(pub,stamp, im, li_box, 2)
            tools.draw_box(im, li_box, (0, 0, 255), 'Box')
            msg = CvBridge().cv2_to_imgmsg(im, 'bgr8')
            im_pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        print 'start python box detection node.'
        main()
    except rospy.ROSInterruptException:
        pass
    print 'exit'
