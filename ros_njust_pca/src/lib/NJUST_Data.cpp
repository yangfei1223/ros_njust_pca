//
// Created by yangfei on 18-7-31.
//

#include <_G_config.h>
#include "ros_njust_pca/NJUST_Data.h"


///////////////////////////////////////////////////////////////////
///
///     Base
///
///////////////////////////////////////////////////////////////////

Base::Base()
{
    m_timeStamp=0;
    m_idx=0;
    m_timeStampVec.clear();
    m_toDecodeVec.clear();
}

Base::~Base()
{

}

void Base::LoadCSVFile(const char *filename)
{
    char str[256];
    char s1[9];
    char s2[256];
    FILE *fp=fopen(filename,"rt");
    if(!fp)
    {
        printf("can not open file: %s\n !",filename);
        exit(-1);
    }
    printf("open %s succeed!\n",filename);
    fgets(str,256,fp);
    while(!feof(fp))
    {
        fgets(str,256,fp);
        memcpy(s1,str,8);   // 前8个字符
        s1[8]='\0';     // 结束符
        m_timeStampVec.push_back(atoi(s1));     // 时间戳
        memcpy(s2,str+9,248);   // 除去时间戳8个字节加1个逗号
        for(int i=0;i<256;i++)      // 去掉换行符
            if(s2[i]=='\r' || s2[i]=='\n')
            {
                s2[i]='\0';
                break;
            }
        m_toDecodeVec.push_back(std::string(s2));    // 待解密的数据（字符串类型）
    }
//    for(int i=0;i<m_toDecodeVec.size();i++)
//        std::cout<<m_toDecodeVec[i]<<std::endl;
}

/// 字符串解码函数， 暂未实现
void Base::DecodeCSVData(int token)
{

#ifndef DATA_DECODE
    // 在未拿到解码数据时，直接返回原始值
    m_decodedString=m_toDecodeVec[m_idx];
#else
    std::string str_s = m_toDecodeVec[m_idx];
    std::string str_d;
    for(int i=0;i<str_s.length();i++)
    {
        char c=token<5?(char)(str_s[i]-token):(char)(str_s[i]+token);
        str_d.push_back(c);
    }
    m_decodedString=str_d;
#endif
    // 去掉字符串中所以空格
    int index=0;
    while((index=m_decodedString.find(' ',index))!=std::string::npos)
        m_decodedString.erase(index,1);
}

/// 因为时间戳递增的属性，此方法比二分查找优
bool Base::FindIdxbyTimeStamp(int64_t timestamp)
{
    for(int i=m_idx;i<m_timeStampVec.size();i++)
        if(timestamp==m_timeStampVec[i])
        {
            m_idx=i;
            return true;
        }
    return false;
}

///////////////////////////////////////////////////////////////////
///
///     Camera
///
///////////////////////////////////////////////////////////////////

Camera::Camera():m_transport(m_node)
{
//    m_pub = m_nh.advertise<ros_njust_pca::NJUST_Sensor_st_Camera>("NJUST_Sensor/Image",1);
    m_pubImg = m_transport.advertise("NJUST_Sensor/Image",1);
};

Camera::~Camera()
{
};

bool Camera::Initialize(const char *filename, const char *imagedir)
{
    //图像文件夹
    sprintf(m_imgDir,"%s",imagedir);
    // 读取CSV文件
    LoadCSVFile(filename);
}


void Camera::ParseStringData()
{
    // 去掉第一个逗号直接返回 // 不用去掉逗号了
    memset(m_imgName,0,sizeof(m_imgName));
    memcpy(m_imgName,m_decodedString.c_str(),m_decodedString.size());
}

bool Camera::ReadImage()
{
    char filename[256];
    sprintf(filename,"%s/%s",m_imgDir,m_imgName);
    m_Img = cv::imread(filename,CV_LOAD_IMAGE_UNCHANGED);
    if(m_Img.data==NULL)
    {
        printf("there is no image name %s, timestamp is %ld\n",filename,m_timeStamp);
        return false;
    }
    return true;
}

bool Camera::DoNext(int64_t timestamp, int token)
{
    if(!FindIdxbyTimeStamp(timestamp))
        return false;
    m_timeStamp=timestamp;
    DecodeCSVData(token);
    ParseStringData();
    ReadImage();
    return true;
}


void Camera::Pubish()
{
    if(m_Img.data==NULL)
        return;
//    m_msg.timestamp=m_timeStamp;
    std_msgs::Header head;
    head.frame_id="camera";
    head.stamp.fromSec(m_timeStamp/1000.0);
    m_msgPtr=cv_bridge::CvImage(head,"bgr8",m_Img).toImageMsg();
    m_pubImg.publish(m_msgPtr);
//    ROS_INFO("++++++++++++++++++publish Image succeed!+++++++++++++++++++++++++\n");
}

int Camera::Debug()
{
    cv::imshow("img",m_Img);
    cv::waitKey(0);
}


///////////////////////////////////////////////////////////////////
///
///     Canbus
///
///////////////////////////////////////////////////////////////////

Canbus::Canbus()
{

}
Canbus::~Canbus()
{

}

bool Canbus::Initialize(const char *filename)
{
    LoadCSVFile(filename);
}

bool Canbus::DoNext(int64_t timestamp, int token)
{
    if(!FindIdxbyTimeStamp(timestamp))
        return false;
    m_timeStamp=timestamp;
    DecodeCSVData(token);
    return true;
}


///////////////////////////////////////////////////////////////////
///
///     Canbus-Gear
///
///////////////////////////////////////////////////////////////////
Gear::Gear()
{
    m_pub=m_node.advertise<ros_njust_pca::NJUST_Sensor_st_Canbus_Gear>("NJUST_Sensor/Canbus_Gear",1);
}

Gear::~Gear()
{

}

void Gear::Publish()
{
    m_msg.timestamp=m_timeStamp;
    m_msg.gear=m_decodedString.c_str()[0];
    m_pub.publish(m_msg);
//    ROS_INFO("++++++++++++++++++publish Gear succeed!+++++++++++++++++++++++++\n");
}


///////////////////////////////////////////////////////////////////
///
///     Canbus-Odom
///
///////////////////////////////////////////////////////////////////

Odom::Odom()
{
    m_pub=m_node.advertise<ros_njust_pca::NJUST_Sensor_st_Canbus_Odom>("NJUST_Sensor/Canbus_Odom",1);

}
Odom::~Odom()
{

}
void Odom::Publish()
{
    m_msg.timestamp=m_timeStamp;
    m_msg.odom = atoi(m_decodedString.c_str());
    m_pub.publish(m_msg);
//    ROS_INFO("++++++++++++++++++publish Odometry succeed!+++++++++++++++++++++++++\n");
}

///////////////////////////////////////////////////////////////////
///
///     Canbus-Steer
///
///////////////////////////////////////////////////////////////////

Steer::Steer()
{
    m_pub=m_node.advertise<ros_njust_pca::NJUST_Sensor_st_Canbus_Steer>("NJUST_Sensor/Canbus_Steer",1);
}
Steer::~Steer()
{

}
void Steer::Publish()
{
    m_msg.timestamp=m_timeStamp;
    m_msg.steer = atoi(m_decodedString.c_str());
    m_pub.publish(m_msg);
//    ROS_INFO("++++++++++++++++++publish Steer succeed!+++++++++++++++++++++++++\n");
}


///////////////////////////////////////////////////////////////////
///
///     GPS
///
///////////////////////////////////////////////////////////////////

GPS::GPS()
{
    m_pub=m_node.advertise<ros_njust_pca::NJUST_Sensor_st_GPS>("NJUST_Sensor/GPS",1);
    m_isFirst=true;
    m_longitudeOrg=0;
    m_latitudeOrg=0;
    m_altitudeOrg=0;
}
GPS::~GPS()
{

}

void GPS::ParseStringData()
{
    char *str;
    if(str=strtok((char *)m_decodedString.c_str(), ","))
        m_msg.latitude=atof(str);
    if(str=strtok(NULL, ","))
        m_msg.longitude=atof(str);
    if(str=strtok(NULL, ","))
        m_msg.altitude=atof(str);
    if(str=strtok(NULL, ","))
        m_msg.NSV1=atoi(str);
    if(str=strtok(NULL, ","))
        m_msg.NSV2=atoi(str);
}

bool GPS::Initialize(const char *filename)
{
    LoadCSVFile(filename);
}

bool GPS::DoNext(int64_t timestamp, int token)
{
    if(!FindIdxbyTimeStamp(timestamp))
        return false;
    m_timeStamp=timestamp;
    DecodeCSVData(token);
    ParseStringData();
    if(m_isFirst)
    {
        m_longitudeOrg=m_msg.longitude;
        m_latitudeOrg=m_msg.latitude;
        m_altitudeOrg=m_msg.altitude;
        m_isFirst= false;
    }
    return true;
}

void GPS::Publish()
{
    m_msg.timestamp=m_timeStamp;
    m_pub.publish(m_msg);
//    ROS_INFO("++++++++++++++++++publish GPS succeed!+++++++++++++++++++++++++\n");
}

bool GPS::GetInitState(double &longitude, double &latitude, double &altitude)
{
    longitude=m_longitudeOrg;
    latitude=m_latitudeOrg;
    altitude=m_altitudeOrg;
    return !m_isFirst;
}

///////////////////////////////////////////////////////////////////
///
///     IMU
///
///////////////////////////////////////////////////////////////////

IMU::IMU()
{
    m_pub=m_node.advertise<ros_njust_pca::NJUST_Sensor_st_IMU>("NJUST_Sensor/IMU",1);
    m_isFirst=true;
    m_headingOrg=0;
    m_pitchOrg=0;
    m_rollOrg=0;
}
IMU::~IMU()
{

}
void IMU::ParseStringData()
{
    char *str;
    if(str=strtok((char *)m_decodedString.c_str(), ","))
        m_msg.heading=atof(str);
    if(str=strtok(NULL, ","))
        m_msg.pitch=atof(str);
    if(str=strtok(NULL, ","))
        m_msg.roll=atof(str);
    // 增加速度解析部分
    if(str=strtok(NULL, ","))
        m_msg.Ve=atof(str);
    if(str=strtok(NULL, ","))
        m_msg.Vn=atof(str);
    if(str=strtok(NULL, ","))
        m_msg.Vu=atof(str);
}

bool IMU::Initialize(const char *filename)
{
    LoadCSVFile(filename);
}

bool IMU::DoNext(int64_t timestamp, int token)
{
    if(!FindIdxbyTimeStamp(timestamp))
        return false;
    m_timeStamp=timestamp;
    DecodeCSVData(token);
    ParseStringData();
    if(m_isFirst)
    {
        m_headingOrg=m_msg.heading;
        m_pitchOrg=m_msg.pitch;
        m_rollOrg=m_msg.roll;
        m_isFirst= false;
    }
    return true;
}

void IMU::Publish()
{
    m_msg.timestamp=m_timeStamp;
    m_pub.publish(m_msg);
//    ROS_INFO("++++++++++++++++++publish IMU succeed!+++++++++++++++++++++++++\n");
}

bool IMU::GetInitSate(double &heading,double &pitch, double &roll)
{
    heading=m_headingOrg;
    pitch=m_pitchOrg;
    roll=m_rollOrg;
    return !m_isFirst;
}


///////////////////////////////////////////////////////////////////
///
///     Velodyne HDL32
///
///////////////////////////////////////////////////////////////////

VelodyneHDL32::VelodyneHDL32()
{
    m_pub = m_node.advertise<velodyne_msgs::VelodyneScan>("NJUST_Sensor/VelodyneScan",1);
    m_idx=0;
    m_isFirst=true;
    m_isOK=false;
    m_preAzi=0;   // 上一个角度
    m_degreeAcc=0;  // 角度累计
    m_numPacket=0;

};

VelodyneHDL32::~VelodyneHDL32()
{

};

/// 添加直接将pcap文件载入内存
bool VelodyneHDL32::Initialize(const char *filename)
{
    // 每次都要读，保存文件名
    LoadPcapFile(filename);
    m_vecPackets.reserve((m_nSize-24)/1264);
    PreProcessPcap();
    m_vecEncode.reserve(50);
    m_vecDecode.reserve(50);
    m_vecNext.reserve(50);
    m_msg.packets.reserve(200);
    delete[] m_pPcap;
}

/// 载入pcap文件
void VelodyneHDL32::LoadPcapFile(const char *filename)
{
    FILE *fp=fopen(filename,"rb");
    if(!fp)
    {
        printf("can not open pcap file %s !\n",filename);
        exit(-1);
    }
    printf("open %s succeed!\n",filename);
    fseek(fp,0,SEEK_END);
    m_nSize=ftell(fp);
    printf("pcap file size is %d.\n",m_nSize);
    m_pPcap=new char[m_nSize];
    fseek(fp,0,SEEK_SET);
    fread(m_pPcap,m_nSize,1,fp);
    fclose(fp);
}
/// 转换时间戳并自动去掉坏包
void VelodyneHDL32::PreProcessPcap()
{
    char *p=m_pPcap;
    int nBytes=0;
    int thigh,tlow,len,rlen;
    int64_t timestamp;
    velodyne_msgs::VelodynePacket packet;
    // 读取pcap文件，并去掉坏包
    nBytes+=24;
    while(nBytes<m_nSize)
    {
        memcpy(&thigh,p+nBytes,4);
        nBytes+=4;
        memcpy(&tlow,p+nBytes,4);
        nBytes+=4;
        memcpy(&len,p+nBytes,4);
        nBytes+=4;
        memcpy(&rlen,p+nBytes,4);
        nBytes+=4;
        timestamp=(int64_t(thigh)*1000+int64_t(tlow/1000)+8*60*60*1000)%(24*60*60*1000);
        if(len!=rlen || rlen!=1248)
        {
            nBytes+=rlen;
            continue;
        }
        packet.stamp.fromSec(timestamp/1000.0);
        memcpy(&packet.data[0],p+nBytes+42,LIDAR_IP_PACKET_SIZE);
        nBytes+=1248;
        m_vecPackets.push_back(packet);
    }
    printf("valid udp size is %ld\n",m_vecPackets.size());
}


bool VelodyneHDL32::FindPacketbyTimestamp(int64_t timestamp)
{
    m_vecEncode.clear();
    while(m_idx<m_vecPackets.size())
    {
        int64_t cur_timestamp=m_vecPackets[m_idx].stamp.toSec()*1000;
        // 比当前时间戳大，继续往后查找
        if(cur_timestamp<timestamp)
        {
            m_idx++;
            continue;
        }
        // 找到时间戳
        if(cur_timestamp==timestamp)
        {
            m_vecEncode.push_back(m_vecPackets[m_idx]);
            m_idx++;
        }
        // 比当前时间戳小，证明是旧包，中断查找
        if(cur_timestamp>timestamp)
            break;

    }
    return !m_vecEncode.empty();
}

// 解码
void VelodyneHDL32::DecodeIPData(int key)
{
    m_vecDecode.clear();
#ifndef DATA_DECODE
    /// 暂时不解密
    for(auto &pack : m_vecEncode)
        m_vecDecode.push_back(pack);
#else
    /// 二进制解密
    velodyne_msgs::VelodynePacket decoded_pack;
    for(auto &pack:m_vecEncode)
    {
        decoded_pack.stamp=pack.stamp;
        for(int i=0;i<1206;i++)
            decoded_pack.data[i]=(uint8_t)((pack.data[i]-key)%256);
        m_vecDecode.push_back(decoded_pack);
    }
#endif
}

void VelodyneHDL32::CheckFrame()
{
    for(auto &pack : m_vecDecode)  // 处理一个时间戳的数据
    {
        if(!m_isOK)     // 是否已经满一帧
        {
            int azimuth;
            m_msg.packets.push_back(pack);    // 放入消息
            if (m_isFirst)  // 是否是第一包
            {
                azimuth = *( (u_int16_t*) (&pack.data[100*0+2]));   // 取出第一个block的角度（或者最后一个block？）
                m_preAzi = azimuth;     // 设置上一个角度为当前角度
                m_isFirst = false;      // 改变标志位
            }
            else    // 不是第一包
            {
                azimuth= *( (u_int16_t*) (&pack.data[100*11+2]));      // 判断最后一包
                int diff = azimuth - m_preAzi;     // 计算角度差
                if (diff < 0)   // 跨圈求补
                    diff += 36000;
                m_degreeAcc += diff;     // 累计角度
                m_preAzi = azimuth;     // 更新上一个角度
            }
            if (m_degreeAcc >= 36000)       // 判断是否满一包
                m_isOK = true;
            m_numPacket++;      // 包数+1
        }
        else    // 已经满一包则放入下一包
        {
            m_vecNext.push_back(pack);
        }

    }
}



bool VelodyneHDL32::DoNext(int64_t timestamp, int key)
{
    if(FindPacketbyTimestamp(timestamp))        // 找到对应时间戳的所有包
    {
        DecodeIPData(key);      // 解密
        CheckFrame();       // 放入msg，并检查是否达到一包
    }
    return m_isOK;  // 返回标志位
}

/// 注意reset的时候会清空数组，一定要先拷贝数据再reset
void VelodyneHDL32::ReSet()
{
    m_numPacket=0;      // 包数置0
    m_isFirst=true;     // 是第一包
    m_isOK=false;   // 没准备好
    m_preAzi=0;     // 上一个角度清空
    m_degreeAcc=0;  // 累计角度清空
    m_msg.packets.clear();  // 消息清空
    // 处理上个时间戳的遗留数据
    for(auto &pack : m_vecNext)   // 如果有遗留数据
    {
        m_msg.packets.push_back(pack);
        if(m_isFirst)   // 如果是第一帧
        {
            m_preAzi = *( (u_int16_t*) (&pack.data[2]));  // 设置角度
            m_isFirst = false;  // 改变标志位
        }
        m_numPacket++;      // 包数+1
    }
    m_vecNext.clear();      // 清空历史数据
}


void VelodyneHDL32::Publish()
{
    m_msg.header.stamp=m_msg.packets[0].stamp;
    m_msg.header.frame_id="velodyne_scan_org";
    m_pub.publish(m_msg);   // 发布一圈数据
//    ROS_INFO("++++++++++++++++++publish PointCloud succeed, point num is %d!+++++++++++++++++++++++++\n",m_numPacket);
}



void VelodyneHDL32::Debug()
{
    /**
    pcap_t *pcap_;
    char errbuf_[PCAP_ERRBUF_SIZE];
    pcap_ = pcap_open_offline(m_filename, errbuf_);
    struct pcap_pkthdr *header;
    const u_char *pkt_data;
    ros_njust_pca::NJUST_Sensor_st_Lidar32 msg;

    while (pcap_next_ex(pcap_, &header, &pkt_data))
    {
            int t_sec=header->ts.tv_sec;
            int t_usec=header->ts.tv_usec;
            uint64_t timestamp=(int64_t(t_sec)*1000+int64_t(t_usec/1000)+8*60*60*1000)%(24*60*60*1000);
            printf("sec %d, usec %d, timestamp %ld\n",t_sec,t_usec,timestamp);
            if(header->len!=header->caplen || header->caplen !=1248)
                continue;
        msg.timestamp=timestamp;
        memcpy(&msg.data[0],pkt_data+42,LIDAR_IP_PACKET_SIZE);
        m_vecMsg.push_back(msg);
    } // loop back and try again
    pcap_close(pcap_);
    **/
}

