//
// Created by yangfei on 18-8-30.
//

#ifndef PROJECT_NJUST_FUSION_H
#define PROJECT_NJUST_FUSION_H

#include "NJUST_Global.h"
#include "NJUST_Transform.h"
#include "NJUST_Lidar32.h"
#include "loam_velodyne/CircularBuffer.h"


class Fusion
{
private:
    /// 订阅传感器数据
    ros::Subscriber m_subIMU;   // 订阅IMU
    ros::Subscriber m_subGPS;   // 订阅GPS
    ros::Subscriber m_subVelodyneScan;  // 订阅32线雷达

    ros::Subscriber m_subLocation;
    ros::Subscriber m_subPointCloudOrg;


    /// 订阅传感器处理结果
    ros::Subscriber m_subObject;    // 订阅目标检测
    ros::Subscriber m_subOdom;  // 订阅雷达里程计
    ros::Subscriber m_subMap;   // 订阅全局地图
    image_transport::Subscriber m_subSeg;   // 订阅分割
    image_transport::Subscriber m_subGrid;  // 订阅障碍物栅格

    /// 发布转换后的传感器数据
    ros::Publisher m_pubImuTrans;   // 发布转换后的IMU数据，包含线加速度
    ros::Publisher m_pubPointCloud2;    // 发布转换后的32线点云数据，PointCloud2格式
    ros::Publisher m_pubPointCloudOrg;     // 发布转换后的32线数据，含线号，点序号，回波等所有信息

    /// 发布融合后的答案数据
    ros::Publisher m_pubObjectFusion;   // 发布融合后的目标检测结果，含定位结果，与答案要求一致
    ros::Publisher m_pubGridFusion;     // 发布融合后的栅格，与分割结果融合，与答案要求一致
    ros::Publisher m_pubOdomFusion;     // 发布融合后的定位数据，与答案要求一致


    int64_t m_timestamp;    // 当前时间戳

    /// 初始姿态
    double m_rollOrg,m_pitchOrg,m_yawOrg;
    double m_headingOrg;


    /// 初始位置
    double m_longitudeOrg,m_latitudeOrg,m_altitudeOrg;


    std::string m_MapSavePath;
    pcl::PointCloud<pcl::PointXYZI> m_Objects;
    pcl::PointCloud<pcl::PointXYZ> m_realTraj;
    pcl::PointCloud<pcl::PointXYZ> m_Traj;
    pcl::PointCloud<pcl::PointXYZRGB> m_MapPointCloud;
    cv::Mat m_GlobalGridMap;


    /// 俯视图转换查找表
    int m_pPosXLUT[GRID_SIZE]={-1};
    int m_pPosYLUT[GRID_SIZE]={-1};

    /// 历史数据缓存队列
    // IMU数据缓存队列
    loam::CircularBuffer<ros_njust_pca::NJUST_Sensor_st_IMU> m_imuQue;
    // GPS数据队列
    loam::CircularBuffer<ros_njust_pca::NJUST_Sensor_st_GPS> m_gpsQue;
    // 分割图像缓存队列
    loam::CircularBuffer<sensor_msgs::Image> m_segQue;
    // 32线点云缓存队列
    loam::CircularBuffer<sensor_msgs::PointCloud2> m_pointsQue;

    // 位姿缓存队列
    loam::CircularBuffer<geometry_msgs::PoseStamped> m_poseQue;


    /// 坐标转换，参考系为车体坐标

    // gps(惯导)坐标转换
    CoordinateTrans m_gpsCTrans;
    // 雷达坐标转换
    CoordinateTrans m_veloCTrans;
    // 图像坐标转换
    MonoCameraTrans m_monoTrans;
    // 雷达数据转换
    Lidar32Convert m_veloConvert;

private:
    /// 各种回调函数
    // 接收GPS数据，存入缓存队列
    void GPSCallback(const ros_njust_pca::NJUST_Sensor_st_GPS::ConstPtr &msg);
    // 接收惯导数据，存入缓存队列(必要时转换并发布ros imu格式的数据)
    void IMUCallback(const ros_njust_pca::NJUST_Sensor_st_IMU::ConstPtr &msg);
    // 点云回调函数
    void Lidar32Callback(const velodyne_msgs::VelodyneScan::ConstPtr &msg);
    // 接收分割图像数据，转换到车体栅格并存入缓存队列
    void SegmentationImageCallback(const sensor_msgs::Image::ConstPtr &msg);
    // 接收栅格数据，与分割结果融合后发布障碍物栅格
    void GridImageCallback(const sensor_msgs::ImageConstPtr &msg);
    // 目标检测回调函数，计算经纬度
    void ObjectDetectionCallback(const ros_njust_pca::NJUST_Answer_st_Object::ConstPtr &msg);
    // 雷达里程计回调函数，转到经纬度
    void LaserOdometeryCallback(const nav_msgs::Odometry::ConstPtr &msg);
    // 雷达建图回调函数，保存为栅格地图
    void LaserMapPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
    // 根据Ins定位
    void INSOdometryCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void INSMapPointCloudCallback(const ros_njust_pca::NJUST_Sensor_st_Lidar32::ConstPtr &msg);


    bool FindNearestIMU(int64_t stamp,ros_njust_pca::NJUST_Sensor_st_IMU &out);
    bool FindNearestGPS(int64_t stamp,ros_njust_pca::NJUST_Sensor_st_GPS &out);
    bool FindNearestSeg(int64_t stamp,sensor_msgs::Image &out);
    bool FindNearestPointCloud(int64_t stamp,sensor_msgs::PointCloud2 &out);
    bool FindNearestPose(int64_t stamp,geometry_msgs::PoseStamped &out);

    void ConstructLUTforGrid();
    void GenerateGridfromImg(cv::Mat &src, cv::Mat &dst);
    // 将一帧点云映射到对应帧的图像的某个区域上，如果成功返回区域的中值
    bool ProjectLidar2ImageArea(int x1,int y1,int x2,int y2, sensor_msgs::PointCloud2 &cloud, Point3d &centerPt);


public:
    Fusion();
    ~Fusion();
    /// 初始化
    bool Initialize(ros::NodeHandle &node,ros::NodeHandle &privateNode);
    /// 执行
    void DoNext(ros::NodeHandle &node);
    /// 调试
    void Debug();

};
#endif //PROJECT_NJUST_FUSION_H
