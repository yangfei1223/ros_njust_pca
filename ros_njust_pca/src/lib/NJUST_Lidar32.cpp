//
// Created by yangfei on 18-8-14.
//

#include "ros_njust_pca/NJUST_Lidar32.h"

Lidar32Convert::Lidar32Convert()
{
    m_timestamp=0;
    m_pointID=0;
};

Lidar32Convert::~Lidar32Convert()=default;

bool Lidar32Convert::Initialize(const char *filename)
{
    m_points.points.reserve(70000);
    m_pointcloud.reserve(70000);
    SortLineIDbyInterPara();
    if(!m_velo32CTrans.LoadCalib(filename))
        return false;
    return true;
}


void Lidar32Convert::SortLineIDbyInterPara()
{
    float tmp[32];
    memcpy(tmp,m_InterPara,sizeof(m_InterPara));
    std::sort(tmp,tmp+32);
    for(int i=0;i<32;i++)
    {
        for(int j=0;j<32;j++)
        {
            if(m_InterPara[i]==tmp[j])
            {
                m_lineID[i]=j;
                break;
            }
        }
    }
}



void Lidar32Convert::DoNext(const velodyne_msgs::VelodyneScan::ConstPtr &msg)
{
    char buffer[LIDAR_IP_PACKET_SIZE];
    uint16_t azi;   // read the angle
    uint16_t dist;    // read the distance
    uint8_t ref;   // read the reflect value
    float range,alpha,omega,x,y,z;
    m_timestamp=int64_t(msg->packets[0].stamp.toSec()*1000);    /// 第一包的时间戳
    ros_njust_pca::NJUST_Sensor_st_Lidar32Point veloPoint;
    pcl::PointXYZI point;
    for(auto &packet:msg->packets)
    {
        memcpy(buffer,&packet.data[0],LIDAR_IP_PACKET_SIZE);
        char *p=buffer;
        for (int block = 0; block < 12; block++,m_pointID++)
        {
            p+=2;
            memcpy(&azi,p,2);
            p+=2;
            for (int id = 0; id < 32; id++)
            {
                memcpy(&dist,p,2);
                p+=2;
                memcpy(&ref,p,1);
                p++;
                range=dist*2/1000.0;
                alpha=azi/100.0/180.0*PI;
                omega=m_InterPara[id]/180.0*PI;
                x=range*cos(omega)*sin(alpha);
                y=range*cos(omega)*cos(alpha);
                z=range*sin(omega);

                // convert to KITTI(input of loam)
                point.x=-x;
                point.y=-y;
                point.z=z;
                point.intensity=ref;

                // transfoem from lidar coord to vehicle coord
                Point3d veloPt(x,y,z);
                Point3d vehiclePt;
                m_velo32CTrans.LocalP2VehicleP(veloPt,vehiclePt);

                veloPoint.range=range; // 测距值
                veloPoint.omega=m_InterPara[id];    // 垂直夹角
                veloPoint.azimuth=alpha;    // 水平夹角
                veloPoint.x=vehiclePt.x;
                veloPoint.y=vehiclePt.y;
                veloPoint.z=vehiclePt.z;
                veloPoint.intensity=ref;
                veloPoint.lineID=m_lineID[id];
                veloPoint.pointID=m_pointID;
                if(range>3.0)
                {
                    // 原始数据格式->lidar obs
                    m_points.points.push_back(veloPoint);
                    // pointcloud2格式->lidar odometry
                    m_pointcloud.push_back(point);
                }
            }
        }
    }
    m_points.timestamp=m_timestamp;
}


int64_t Lidar32Convert::GetFrame(pcl::PointCloud<pcl::PointXYZI> &pointcloud, ros_njust_pca::NJUST_Sensor_st_Lidar32 &msg)
{
    pointcloud=m_pointcloud;
    msg=m_points;
    return m_timestamp;
}


void Lidar32Convert::ReSet()
{
    m_timestamp=0;
    m_pointID=0;
    m_points.timestamp=0;
    m_points.points.clear();
    m_pointcloud.clear();
}



void Lidar32Convert::Debug(int frameID)
{
}
