#include "ros_njust_pca/NJUST_Global.h"
#include"ros_njust_pca/Lidar32OD.h"

bool flag=false;
int64_t g_timestamp;
Lidar32Data gData32;
pthread_mutex_t g_mutex_lidar = PTHREAD_MUTEX_INITIALIZER;
bool isNew()
{
    bool ret;
    pthread_mutex_lock(&g_mutex_lidar);
    ret=flag;
    pthread_mutex_unlock(&g_mutex_lidar);
    return ret;
}

// 点云回调函数
void LidarCallback(const ros_njust_pca::NJUST_Sensor_st_Lidar32::ConstPtr &msg)
{
    int lineID, pointID;
    pthread_mutex_lock(&g_mutex_lidar);
    g_timestamp=msg->timestamp;
    memset(&gData32,0,sizeof(Lidar32Data));
    for(auto &point:msg->points)
    {

        lineID=point.lineID;
        pointID=point.pointID;
        gData32.OneFrameData[lineID][pointID].x = point.x*100;            //单位，厘米
        gData32.OneFrameData[lineID][pointID].y = point.y*100;
        gData32.OneFrameData[lineID][pointID].z = point.z*100;
        gData32.OneFrameData[lineID][pointID].azi = point.azimuth*100;            //单位，0.01度
        gData32.OneFrameData[lineID][pointID].ver = point.omega*100;
        gData32.OneFrameData[lineID][pointID].realDistance = point.range*100;        //单位，厘米
        gData32.OneFrameData[lineID][pointID].Intensity = point.intensity;
        gData32.RealEachLinePointNum[lineID] = 2200;
    }
    flag= true;
    pthread_mutex_unlock(&g_mutex_lidar);
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"obs_node");
    ros::NodeHandle node;
    image_transport::ImageTransport transport(node);
    ros::Subscriber subPointCloud=node.subscribe("NJUST_Sensor/PointCloudOrg",1,LidarCallback);
    image_transport::Publisher pubGrid = transport.advertise("NJUST_Result/Grid",1);
    ROS_INFO("start obstacle node succeed !\n");
    CLidar32OD *pLidar32OD=new CLidar32OD();
    sensor_msgs::Image grid_msg;
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        if(isNew())
        {
            pthread_mutex_lock(&g_mutex_lidar);
            pLidar32OD->InputData(g_timestamp,&gData32);       // 输入
            flag=false;
            pthread_mutex_unlock(&g_mutex_lidar);
            pLidar32OD->PODFromLidar32();        // 执行
            // 返回结果
            cv::Mat grid;
            int64_t timestamp=pLidar32OD->GetGrid(grid);
            std_msgs::Header header;
            header.stamp.fromSec(timestamp/1000.0);
            cv_bridge::CvImage(header,"mono8",grid).toImageMsg(grid_msg);
            pubGrid.publish(grid_msg);
//            ROS_INFO("publish local map succeed !\n");
        }
        loop_rate.sleep();
    }
	delete pLidar32OD;
}