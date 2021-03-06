//
// Created by yangfei on 18-8-1.
//

#ifndef NJUST_PCALAB_GLOBAL_H
#define NJUST_PCALAB_GLOBAL_H
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <netdb.h>
#include <pthread.h>
#include <errno.h>
#include <pcap.h>


#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/cloud_viewer.h>

#include <std_msgs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/Marker.h>

#include <velodyne_msgs/VelodynePacket.h>
#include <velodyne_msgs/VelodyneScan.h>
#include "ros_njust_pca/NJUST_Sensor_st_Lidar.h"
#include "ros_njust_pca/NJUST_Sensor_st_Lidar32Point.h"
#include "ros_njust_pca/NJUST_Sensor_st_Lidar32.h"
#include "ros_njust_pca/NJUST_Sensor_st_IMU.h"
#include "ros_njust_pca/NJUST_Sensor_st_GPS.h"
#include "ros_njust_pca/NJUST_Sensor_st_Canbus_Gear.h"
#include "ros_njust_pca/NJUST_Sensor_st_Canbus_Odom.h"
#include "ros_njust_pca/NJUST_Sensor_st_Canbus_Steer.h"
#include "ros_njust_pca/NJUST_Answer_st_Object.h"
#include "ros_njust_pca/NJUST_Answer_st_Location.h"
#include "ros_njust_pca/NJUST_Answer_st_Map.h"

#include "ros_njust_pca/NJUST_Command.h"
#include "ros_njust_pca/NJUST_InitState.h"

#define MAX_IP_LENGTH           124800      // ????????????????????????
#define MAX_POINT_NUM           100000      // 32???????????????????????????
#define IMG_WIDTH               1024        // ?????????
#define IMG_HEIGHT              768         // ?????????
#define IMG_SIZE                1024*768*3
#define GRID_WIDTH              81
#define GRID_HEIGHT             81
#define GRID_SIZE               GRID_WIDTH*GRID_HEIGHT
#define GRID_RESOLUTION         0.5
#define LIDAR_IP_PACKET_SIZE    1206        // velodyne???????????????
#define PI                      3.14159265359 // ?????????
#define DEG_RAD                 0.01745329252    // ??????????????????
#define RE                      6378137     // ????????????
#define RN                      6356755     // ????????????????????????
#define MAP_RESOLUTION          0.05        // ???????????????

#define WHEEL_RADIUS            0
#define GPS_STATE_THRE          32

#define USE_NEGATIVE    // ???????????????????????????
#define SEND_AT_TASK_STAMP    //  ?????????????????????
#define SEND_BIG_ENDIAN   // ???????????????
#define DATA_DECODE


////////////////////////////////////////////////////////////////////////////////////////////////
///
///  ??????????????????
///
////////////////////////////////////////////////////////////////////////////////////////////////
#pragma pack(push) //??????????????????
#pragma pack(1)    //???1????????????,?????????????????????

enum GRID_TYPE
{
    GRID_UNKNOWN=0x00,
    GRID_POSITIVE,
    GRID_NEGATIVE,
    GRID_PASSABLE,
    GRID_WATER
};

////////////////////////////////////////////////////////////////////////////////////////////////
///
///  ??????????????????
///
////////////////////////////////////////////////////////////////////////////////////////////////
#pragma pack(pop)//??????????????????
#endif //NJUST_PCALAB_GLOBAL_H
