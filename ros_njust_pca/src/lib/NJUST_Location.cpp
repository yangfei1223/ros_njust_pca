//
// Created by yangfei on 18-9-6.
//
#include <ros/ros.h>
#include "ros_njust_pca/NJUST_Location.h"


Location::Location()
{
    m_lastStamp=0;
}

Location::~Location()=default;


void Location::IMUCallback(const ros_njust_pca::NJUST_Sensor_st_IMU::ConstPtr &msg)
{
//    if(m_ImuQue.empty())
//    {
//        m_rollOrg=msg->roll;
//        m_pitchOrg=msg->pitch;
//        m_yawOrg=msg->heading<=180?msg->heading:msg->heading-360;
//    }
    m_ImuQue.push(*msg);
}
void Location::GPSCallback(const ros_njust_pca::NJUST_Sensor_st_GPS::ConstPtr &msg)
{
//    if(m_GpsQue.empty())
//    {
//        m_longitudeOrg=msg->longitude;
//        m_latitudeOrg=msg->latitude;
//        m_altitudeOrg=msg->altitude;
//    }
    m_GpsQue.push(*msg);
}



bool Location::Initialize(ros::NodeHandle &node)
{
    ros::ServiceClient client = node.serviceClient<ros_njust_pca::NJUST_InitState>("NJUST_Service/InitState");
    ros_njust_pca::NJUST_InitState srv;
    ROS_INFO("location node try to get init IMU state ...");
    while (!client.call(srv))
    {
        usleep(100000);
    }
    ROS_INFO("location node get init IMU state succeed !");
    m_rollOrg=srv.response.roll;
    m_pitchOrg=srv.response.pitch;
    m_yawOrg=srv.response.heading<=180?srv.response.heading:srv.response.heading-360;
    m_longitudeOrg=srv.response.longitude;
    m_latitudeOrg=srv.response.latitude;
    m_altitudeOrg=srv.response.altitude;

    m_subIMU=node.subscribe<ros_njust_pca::NJUST_Sensor_st_IMU>("NJUST_Sensor/IMU",1,&Location::IMUCallback,this);
    m_subGPS=node.subscribe<ros_njust_pca::NJUST_Sensor_st_GPS>("NJUST_Sensor/GPS",1,&Location::GPSCallback,this);
    m_pubPose=node.advertise<geometry_msgs::PoseStamped>("NJUST_Result/Pose",1);
}
void Location::DoNext(ros::NodeHandle &node)
{
    geometry_msgs::PoseStamped pose;
    ros_njust_pca::NJUST_Sensor_st_IMU curIMUState;
    ros_njust_pca::NJUST_Sensor_st_GPS curGPSState;
    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        ros::spinOnce();
        if(m_ImuQue.empty()||m_GpsQue.empty())
            continue;
        curIMUState=m_ImuQue.last();
        curGPSState=m_GpsQue.last();    // 频率一致，时间戳也一致，应该不用人工对齐了
        if(curIMUState.timestamp==m_lastStamp)
            continue;
        pose.header.stamp.fromSec(curIMUState.timestamp/1000.0);
        // GPS不正常时IMU精度会降低一些
        pose.pose.orientation.x=curIMUState.roll;
        pose.pose.orientation.y=curIMUState.pitch;
        pose.pose.orientation.z=curIMUState.heading;
        pose.pose.orientation.w=1.0;
        /// GPS正常的时候(第一帧肯定正常)
//        if(curGPSState.longitude!=-1&&curGPSState.latitude!=-1&&curGPSState.altitude!=-1
//                &&curGPSState.NSV1+curGPSState.NSV2>=32)
        if(curGPSState.NSV1+curGPSState.NSV2>=GPS_STATE_THRE)
        {
            double x,y;
            TransformLongitudeLatitude2World(curGPSState.longitude,curGPSState.latitude,
                                             m_longitudeOrg,m_latitudeOrg,&x,&y);
            pose.pose.position.x=x;
            pose.pose.position.y=y;
            pose.pose.position.z=curGPSState.altitude-m_altitudeOrg;

        }
        /// GPS不正常的时候
        else
        {
//            printf("GPS error, use IMU!\n");
            // 根据微分计算上一帧加速度
            double dt=(curIMUState.timestamp-m_lastStamp)/1000.0;
//            printf("current stamp is %d\n",curIMUState.timestamp);
//            printf("time diff is %lf\n",dt);
            double dvx=curIMUState.Ve-m_lastVelocity.x;     // 东向x
            double dvy=curIMUState.Vn-m_lastVelocity.y;     // 北向y
            double dvz=curIMUState.Vu-m_lastVelocity.z;     // 天向z
            double ax=dvx/dt;
            double ay=dvy/dt;
            double az=dvz/dt;
            // 根据加速度计算偏移
            pose.pose.position.x=m_lastPosition.x+m_lastVelocity.x*dt+0.5*ax*dt*dt;
            pose.pose.position.y=m_lastPosition.y+m_lastVelocity.y*dt+0.5*ay*dt*dt;
            pose.pose.position.z=m_lastPosition.z+m_lastVelocity.z*dt+0.5*az*dt*dt;
        }
        m_lastStamp=curIMUState.timestamp;
        m_lastVelocity.x=curIMUState.Ve;
        m_lastVelocity.y=curIMUState.Vn;
        m_lastVelocity.z=curIMUState.Vu;
        m_lastPosition=pose.pose.position;
        m_pubPose.publish(pose);
        loop_rate.sleep();
    }

}
