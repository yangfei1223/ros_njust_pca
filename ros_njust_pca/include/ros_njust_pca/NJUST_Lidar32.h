//
// Created by yangfei on 18-8-14.
//

#ifndef PROJECT_NJUST_LIDAR32_H
#define PROJECT_NJUST_LIDAR32_H

#include "NJUST_Global.h"
#include "ros_njust_pca/NJUST_Transform.h"

class Lidar32Convert
{
private:
    int64_t m_timestamp;    // 当前帧的时间戳（转换后，与其他数据一致）
    // 雷达内参
    const float m_InterPara[32]={-30.67,-9.33,-29.33,-8.00,-28.00,-6.67,-26.67,-5.33,
                                  -25.33,-4.00,-24.00,-2.67,-22.67,-1.33,-21.33,0,
                                  -20.00,1.33,-18.67,2.67,-17.33,4.00,-16.00,5.33,
                                  -14.67,6.77,-13.33,8.00,-12.00,9.33,-10.67,10.67};
    // 雷达外参转换
    CoordinateTrans m_velo32CTrans;

    int m_lineID[32];
    int m_pointID;

    ros_njust_pca::NJUST_Sensor_st_Lidar32 m_points;
    pcl::PointCloud<pcl::PointXYZI> m_pointcloud;

    void SortLineIDbyInterPara();
public:
    Lidar32Convert();
    ~Lidar32Convert();
    bool Initialize(const char *filename);
    void DoNext(const velodyne_msgs::VelodyneScan::ConstPtr &msgs);
    int64_t GetFrame(pcl::PointCloud<pcl::PointXYZI> &PointCloud, ros_njust_pca::NJUST_Sensor_st_Lidar32 &PointsMsg);
    void ReSet();
    void Debug(int frameID);
};

#endif //PROJECT_NJUST_LIDAR32_H
