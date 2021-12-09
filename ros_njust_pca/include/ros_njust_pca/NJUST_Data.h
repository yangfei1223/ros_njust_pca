//
// Created by yangfei on 18-7-31.
//

#ifndef NJUST_PCALAB_NJUST_DATA_H
#define NJUST_PCALAB_NJUST_DATA_H

#include "NJUST_Global.h"

/// 用于解析CSV文件的公共类
class Base
{
public:
    ros::NodeHandle m_node;
    ros::Publisher m_pub;
    int64_t m_timeStamp;    // 当前帧的时间戳
    int m_idx;      // 当前时间戳的id,在新来一帧时此值为上一帧id
    std::vector<int> m_timeStampVec;     // 时间戳列表
    std::vector<std::string> m_toDecodeVec;   // 待解密的string列表（包含逗号分隔符）
    std::string m_decodedString;

    Base();
    ~Base();

    // 读取CSV文件
    void LoadCSVFile(const char *filename);
    // 寻找时间戳
    bool FindIdxbyTimeStamp(int64_t timestamp);
    // 解密数据（字符串数据，雷达需要另外写）
    void DecodeCSVData(int token);      //只能一条一条解码
    // 子类根据数据类型去实现
    virtual void ParseStringData()=0;

};

/// 相机类 timetamp,filenaem
class Camera: private Base
{
private:

    image_transport::ImageTransport m_transport;
    image_transport::Publisher m_pubImg;
    char m_imgDir[256];     // 图像文件夹
    char m_imgName[256];    // 图像文件名
    cv::Mat m_Img;

//    ros_njust_pca::NJUST_Sensor_st_Camera m_msg;
    sensor_msgs::ImagePtr m_msgPtr;
    // 从String中解析出数据格式
    void ParseStringData();
    // 根据时间戳读取当前帧的图像数据
    bool ReadImage();

public:
//     构造函数
    Camera();
//     析构函数
    ~Camera();
    // 初始化函数
    bool Initialize(const char * filename,const char *imagedir);
    // 执行
    bool DoNext(int64_t timestamp, int token);

    void Pubish();
    // 调试函数
    int Debug();
};


/// Canbus类 timestamp,value
class Canbus: public Base
{
private:
    void ParseStringData(){};       // 简单数据类型不用解析函数
public:
    Canbus();
    ~Canbus();
    bool Initialize(const char *filename);
    bool DoNext(int64_t timestamp, int token);
};


/// 档位类 timestamp,value
class Gear: public Canbus
{
private:
    ros_njust_pca::NJUST_Sensor_st_Canbus_Gear m_msg;
public:
    Gear();
    ~Gear();
    void Publish();
};

/// 里程计类 timestamp,value
class Odom: public Canbus
{
private:
    ros_njust_pca::NJUST_Sensor_st_Canbus_Odom m_msg;
public:
    Odom();
    ~Odom();
    void Publish();
};

/// 方向盘类 timestamp,value
class Steer: public Canbus
{
private:
    ros_njust_pca::NJUST_Sensor_st_Canbus_Steer m_msg;
public:
    Steer();
    ~Steer();
    void Publish();
};


/// GPS类 timestamp,latitude,longitude,altitude,NSV1,NSV2
class GPS: private Base
{
private:
    bool m_isFirst;
    double m_longitudeOrg;
    double m_latitudeOrg;
    double m_altitudeOrg;
    ros_njust_pca::NJUST_Sensor_st_GPS m_msg;
    void ParseStringData();
public:
    GPS();
    ~GPS();
    bool Initialize(const char *filename);
    bool DoNext(int64_t timestamp, int token);
    void Publish();
    bool GetInitState(double &longitude, double &latitude, double &altitude);
};

// IMU数据转换，给出完整的姿态角，角速度和线加速度
class IMU: private Base
{
private:
    bool m_isFirst;
    double m_headingOrg;
    double m_pitchOrg;
    double m_rollOrg;
    ros_njust_pca::NJUST_Sensor_st_IMU m_msg;
    void ParseStringData();
public:
    IMU();
    ~IMU();
    bool Initialize(const char *filename);
    bool DoNext(int64_t timestamp, int token);
    void Publish();
    bool GetInitSate(double &heading,double &pitch, double &roll);
};

/// 单线激光雷达类，暂留空
class URG
{
private:
public:
    URG(){};
    ~URG(){};
    bool Initialize(const char *filename){};
    bool DoNext(int64_t timestamp, int token){ return false;};
    void Publish(){};
};


/// 32激光雷达类
class VelodyneHDL32
{
private:
    ros::NodeHandle m_node;
    ros::Publisher m_pub;

    char *m_pPcap;
    int m_nSize;

    int m_idx;
    bool m_isFirst;     // 是否是一帧的第一包
    bool m_isOK;
    int m_preAzi;   // 上一个角度
    int m_degreeAcc;  // 角度累计

    int m_numPacket;

    std::vector<velodyne_msgs::VelodynePacket> m_vecPackets;    // 保存所有包
    std::vector<velodyne_msgs::VelodynePacket> m_vecEncode;  // 当前时间戳待解码
    std::vector<velodyne_msgs::VelodynePacket> m_vecDecode;  // 当前时间戳已解码
    std::vector<velodyne_msgs::VelodynePacket> m_vecNext;    // 遗留到下一帧的
    velodyne_msgs::VelodyneScan m_msg;      // 保存一整圈点云,得到一圈数据后发布

    void LoadPcapFile(const char *filename);
    void PreProcessPcap();
    bool FindPacketbyTimestamp(int64_t timestamp);  // 从pcap文件里按照给定的时间戳读取一帧，返回找到的包数
    void DecodeIPData(int key);
    void CheckFrame();
public:
    VelodyneHDL32();
    ~VelodyneHDL32();
    bool Initialize(const char *filename);    // 初始化
    bool DoNext(int64_t timestamp,int key);     // 执行，算法处理
    void ReSet();   // 不是必须
    void Publish();     // get函数void GetFrame(int * res)
    void Debug();   // 调试
};



#endif //NJUST_PCALAB_NJUST_DATA_H
