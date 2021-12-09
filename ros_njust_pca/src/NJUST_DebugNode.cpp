//
// Created by yangfei on 18-8-14.
//

#include "ros_njust_pca/NJUST_Global.h"
#include "ros_njust_pca/NJUST_Transform.h"

double YAW_ORG=0;
double PITCH_ORG=0;
double ROLL_ORG=0;
double LONGITUDE_ORG=0;
double LATITUDE_ORG=0;
double ALTITUDE_ORG=0;

/**
ros::Publisher img_pub;
void ImageCallback(const ros_njust_pca::NJUST_Sensor_st_Camera::ConstPtr &msg)
{
    sensor_msgs::Image im_msg=msg->image;
    img_pub.publish(im_msg);
}
**/

ros::Publisher pubPoinCloud2;
void Lidar32Callback(const ros_njust_pca::NJUST_Sensor_st_Lidar32::ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZI> pointcloud;
    pcl::PointXYZI point;
    for(auto &point_org:msg->points)
    {
        point.x=point_org.x;
        point.y=point_org.y;
        point.z=point_org.z;
        point.intensity=point_org.intensity;
        pointcloud.push_back(point);
    }

    sensor_msgs::PointCloud2 pointCloudMsg;
    pcl::toROSMsg(pointcloud,pointCloudMsg);
    pointCloudMsg.header.frame_id="vehicle";
    pointCloudMsg.header.stamp.fromSec(msg->timestamp/1000.0);
    pubPoinCloud2.publish(pointCloudMsg);

}



void IMUCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    sensor_msgs::Imu imu_msg;
    ROS_INFO("********receive IMU message********************\n");
}

ros::Publisher pubTruePath;
nav_msgs::Path gTruePathMsg;
void GPSCallback(const ros_njust_pca::NJUST_Sensor_st_GPS::ConstPtr &msg)
{
    if(msg->longitude!=-1&&msg->latitude!=-1)
    {
        double x,y;
        TransformLongitudeLatitude2World(msg->longitude,msg->latitude,LONGITUDE_ORG,LATITUDE_ORG,&x,&y);
        geometry_msgs::PoseStamped pose;
        pose.header.stamp.fromSec(msg->timestamp/1000.0);
        geometry_msgs::Point point;
        geometry_msgs::Quaternion orientation;
        point.x=x;
        point.y=y;
        point.z=0;
        orientation=tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
        pose.pose.position=point;
        pose.pose.orientation=orientation;
        gTruePathMsg.poses.push_back(pose);
        pubTruePath.publish(gTruePathMsg);
    }
}

////////////////////////////////////////////////////////////////////////////////////////
///
///        Canbus Sensor, It's actually useless!
///
////////////////////////////////////////////////////////////////////////////////////////

void GearCallback(const ros_njust_pca::NJUST_Sensor_st_Canbus_Gear::ConstPtr &msg)
{
    ROS_INFO("*********receive Gear message, timestamp is %ld (gear=%c)***********\n",
             msg->timestamp,msg->gear);
}
void OdomCallback(const ros_njust_pca::NJUST_Sensor_st_Canbus_Odom::ConstPtr &msg)
{
    ROS_INFO("*********receive Odometry message, timestamp is %ld (odom=%d)***********\n",
             msg->timestamp,msg->odom);
}
void SteerCallback(const ros_njust_pca::NJUST_Sensor_st_Canbus_Steer::ConstPtr &msg)
{
    ROS_INFO("*********receive Steer message, timestamp is %ld (steer=%d)***********\n",
             msg->timestamp,msg->steer);
}


ros::Publisher pubObjectMarker;
void ObjectCallback(const ros_njust_pca::NJUST_Answer_st_Object::ConstPtr &msg)
{
}


ros::Publisher pubPosGrid;
ros::Publisher pubNegGrid;
ros::Publisher pubPassGrid;
void LocalMapCallback(const ros_njust_pca::NJUST_Answer_st_Map::ConstPtr &msg)
{
    sensor_msgs::Image grid_im;
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg->map,"mono8");

    geometry_msgs::Point pt;
    nav_msgs::GridCells posGrid;
    nav_msgs::GridCells negGrid;
    nav_msgs::GridCells passGrid;

    posGrid.header.frame_id="vehicle";
    posGrid.header.stamp.fromSec(msg->timestamp/1000.0);
    posGrid.cell_height=GRID_RESOLUTION;
    posGrid.cell_width=GRID_RESOLUTION;

    negGrid.header.frame_id="vehicle";
    negGrid.header.stamp.fromSec(msg->timestamp/1000.0);
    negGrid.cell_height=GRID_RESOLUTION;
    negGrid.cell_width=GRID_RESOLUTION;

    passGrid.header.frame_id="vehicle";
    passGrid.header.stamp.fromSec(msg->timestamp/1000.0);
    passGrid.cell_height=GRID_RESOLUTION;
    passGrid.cell_width=GRID_RESOLUTION;

    for(int i=0;i<cv_ptr->image.rows;i++)
    {
        for(int j=0;j<cv_ptr->image.cols;j++)
        {
            GRID_TYPE type =GRID_TYPE(cv_ptr->image.at<uchar>(i,j));
            pt.x=(j-40)*GRID_RESOLUTION;
            pt.y=(80-i)*GRID_RESOLUTION;
            pt.z=0;
            switch (type)
            {
                case GRID_POSITIVE:
                    posGrid.cells.push_back(pt);
                    break;
                case GRID_NEGATIVE:
                    negGrid.cells.push_back(pt);
                    break;
                case GRID_PASSABLE:
                    passGrid.cells.push_back(pt);
                default:
                    break;
            }

        }
    }
    pubPosGrid.publish(posGrid);
    pubNegGrid.publish(negGrid);
    pubPassGrid.publish(passGrid);

}


ros::Publisher pubPath;
ros::Publisher pubArrow;
nav_msgs::Path gPathMsg;
void LocationCallback(const ros_njust_pca::NJUST_Answer_st_Location::ConstPtr &msg)
{
    // coordinate
    double x,y;
    TransformLongitudeLatitude2World(msg->longitude,msg->latitude,LONGITUDE_ORG,LATITUDE_ORG,&x,&y);
    geometry_msgs::PoseStamped pose;
    pose.header.stamp.fromSec(msg->timestamp/1000.0);
    geometry_msgs::Point point;
    geometry_msgs::Quaternion orientation;
    point.x=x;
    point.y=y;
    point.z=0;
    orientation=tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
    pose.pose.position=point;
    pose.pose.orientation=orientation;
    gPathMsg.poses.push_back(pose);
    // heading
    visualization_msgs::Marker marker;
    marker.header.frame_id="world";
    marker.header.stamp.fromSec(msg->timestamp/1000.0);
    marker.ns="arrow";
    marker.id=0;
    marker.type=visualization_msgs::Marker::ARROW;
    marker.action=visualization_msgs::Marker::ADD;

    double heading=msg->heading;
    heading=heading<=270?90-heading:450-heading;
    heading=heading*DEG_RAD;

    orientation=tf::createQuaternionMsgFromYaw(heading);
    marker.pose.orientation=orientation;
    marker.pose.position=point;
    marker.scale.x=1.0;
    marker.scale.y=1.0;
    marker.scale.z=1.0;
    marker.color.b=0.0f;
    marker.color.g=1.0f;
    marker.color.r=0.0f;
    marker.color.a=1.0f;
    marker.lifetime=ros::Duration();

    pubPath.publish(gPathMsg);
    pubArrow.publish(marker);
}




int main(int argc, char**argv)
{
    ros::init(argc,argv,"debug_node");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<ros_njust_pca::NJUST_InitState>("NJUST_Service/InitState");
    ros_njust_pca::NJUST_InitState srv;
    ROS_INFO("debug node try to get init IMU state ...");
    while (!client.call(srv))
    {
        usleep(100000);
    }
    ROS_INFO("debug node get init IMU state succeed !");
    ROLL_ORG=srv.response.roll;
    PITCH_ORG=srv.response.pitch;
    YAW_ORG=srv.response.heading<=180?srv.response.heading:srv.response.heading-360;
    LONGITUDE_ORG=srv.response.longitude;
    LATITUDE_ORG=srv.response.latitude;
    ALTITUDE_ORG=srv.response.altitude;

    image_transport::ImageTransport it(nh);
    gTruePathMsg.header.frame_id="world";
    gPathMsg.header.frame_id="world";

    /// Subscribe
    ros::Subscriber subGPS=nh.subscribe<ros_njust_pca::NJUST_Sensor_st_GPS>("NJUST_Sensor/GPS",1,GPSCallback);
    ros::Subscriber subPointCloudOrg=nh.subscribe<ros_njust_pca::NJUST_Sensor_st_Lidar32>("NJUST_Sensor/PointCloudOrg",1,Lidar32Callback);
    ros::Subscriber subGridFusion=nh.subscribe<ros_njust_pca::NJUST_Answer_st_Map>("NJUST_Answer/Map",1,LocalMapCallback);
    ros::Subscriber subLocation=nh.subscribe<ros_njust_pca::NJUST_Answer_st_Location>("NJUST_Answer/Location",1,LocationCallback);


    /// Advertise
    pubPoinCloud2 = nh.advertise<sensor_msgs::PointCloud2>("NJUST_Debug/PointCloud2",1);
    pubPosGrid = nh.advertise<nav_msgs::GridCells>("NJUST_Debug/PosGrid",1);
    pubNegGrid = nh.advertise<nav_msgs::GridCells>("NJUST_Debug/NegGrid",1);
    pubPassGrid = nh.advertise<nav_msgs::GridCells>("NJUST_Debug/PassGrid",1);
    pubTruePath = nh.advertise<nav_msgs::Path>("NJUST_Debug/TruePath",1);
    pubPath = nh.advertise<nav_msgs::Path>("NJUST_Debug/Path",1);
    pubArrow = nh.advertise<visualization_msgs::Marker>("NJUST_Debug/Heading",1);

    ros::spin();
    return 0;
}
