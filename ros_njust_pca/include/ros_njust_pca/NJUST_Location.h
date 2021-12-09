//
// Created by yangfei on 18-9-6.
//

#ifndef PROJECT_NJUST_LOCATION_H
#define PROJECT_NJUST_LOCATION_H

#include "NJUST_Global.h"
#include "NJUST_Transform.h"
#include "loam_velodyne/CircularBuffer.h"

/// 通过IMU和GPS定位和建图
class Location
{
private:
    ros::Subscriber m_subIMU;
    ros::Subscriber m_subGPS;

    ros::Publisher m_pubPose;

    /// 初始姿态
    double m_rollOrg,m_pitchOrg,m_yawOrg;

    /// 初始位置
    double m_longitudeOrg,m_latitudeOrg,m_altitudeOrg;

    int64_t m_lastStamp;
    geometry_msgs::Point m_lastPosition;
    geometry_msgs::Point m_lastVelocity;

    /// IMU和GPS的缓存队列
    loam::CircularBuffer<ros_njust_pca::NJUST_Sensor_st_IMU> m_ImuQue;
    loam::CircularBuffer<ros_njust_pca::NJUST_Sensor_st_GPS> m_GpsQue;


    void IMUCallback(const ros_njust_pca::NJUST_Sensor_st_IMU::ConstPtr &msg);
    void GPSCallback(const ros_njust_pca::NJUST_Sensor_st_GPS::ConstPtr &msg);

public:
    Location();
    ~Location();
    bool Initialize(ros::NodeHandle &node);
    void DoNext(ros::NodeHandle &node);
};

#endif //PROJECT_NJUST_LOCATION_H
