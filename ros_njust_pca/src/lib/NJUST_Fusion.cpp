//
// Created by yangfei on 18-8-30.
//

#include "ros_njust_pca/NJUST_Fusion.h"


void Fusion::GPSCallback(const ros_njust_pca::NJUST_Sensor_st_GPS::ConstPtr &msg)
{
//    if (m_gpsQue.empty())       // 第一帧
//    {
//        m_longitudeOrg = msg->longitude;
//        m_latitudeOrg = msg->latitude;
//    }
    m_gpsQue.push(*msg);
    // draw groundtruth on global map
//    if (msg->longitude != -1 && msg->latitude != -1)
    if (msg->NSV1+msg->NSV2>=GPS_STATE_THRE)
    {
//        int grid_x, grid_y;
        pcl::PointXYZ point;
        double x, y;
        TransformLongitudeLatitude2World(msg->longitude, msg->latitude, m_longitudeOrg, m_latitudeOrg, &x, &y);
        point.x=x;
        point.y=y;
        point.z=0;
        m_realTraj.push_back(point);

        /**
        grid_x = x / MAP_RESOLUTION + MAP_WIDTH / 2;  // 东向
        grid_y = MAP_HEIGHT / 2 - y / MAP_RESOLUTION; // 北向
        if (grid_x >= 0 && grid_x < MAP_WIDTH
            && grid_y > 0 && grid_y < MAP_HEIGHT)
            cv::drawMarker(m_GlobalGridMap, cv::Point(grid_x, grid_y), cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 10, 2);
        **/

    }

}
// 接收惯导数据，存入缓存队列(必要时转换并发布ros imu格式的数据)
void Fusion::IMUCallback(const ros_njust_pca::NJUST_Sensor_st_IMU::ConstPtr &msg)
{
    // 存入缓存队列
//    if(m_imuQue.empty())
//    {
//        m_rollOrg=msg->roll;
//        m_pitchOrg=msg->pitch;
//        m_yawOrg=msg->heading<=180?msg->heading:msg->heading-360;
//    }
    m_imuQue.push(*msg);
    // 融合IMU数据
#if 0
    if(m_imuQue.size()>0)
    {
        sensor_msgs::Imu imuMsg;
        ros_njust_pca::NJUST_Sensor_st_IMU curState=*msg;
        ros_njust_pca::NJUST_Sensor_st_IMU preState=m_imuQue.last();
        double dt=(curState.timestamp-preState.timestamp)/1000.0;   // 毫秒转换为秒
        /// 线速度差分，用于计算线加速度,得到的是上一时刻的加速度
        double dvx=curState.Ve-preState.Ve;     // 东向
        double dvy=curState.Vn-preState.Vn;     // 北向
        double dvz=curState.Vu-preState.Vu;     // 天向

        imuMsg.header.frame_id="imu";
        imuMsg.header.stamp.fromSec(preState.timestamp/1000.0);

        /// 注意正负
        double roll=preState.roll*DEG_RAD;      // 右翻为正
        double pitch=-preState.pitch*DEG_RAD;      // 俯角为正
        double heading=preState.heading;    // 逆时针为正
        heading=heading<=270?90-heading:450-heading;
        heading=heading*DEG_RAD;
        geometry_msgs::Quaternion orientation;
        // 这样得到的旋转角分别是x,y,z,顺时针为正?
        orientation=tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,heading);    // 转为弧度?
        imuMsg.orientation=orientation;
        // 上一时刻的线性加速度,算出来就是合力
        imuMsg.linear_acceleration.x=dvy/dt;     // 世界坐标（东向）x
        imuMsg.linear_acceleration.y=-dvx/dt;     // 世界坐标（北向）y
        imuMsg.linear_acceleration.z=dvz/dt;     // 世界坐标（天向）z
        m_pubImuTrans.publish(imuMsg);
    }
#endif

}

void Fusion::Lidar32Callback(const velodyne_msgs::VelodyneScan::ConstPtr &msg)
{
    int64_t stamp;
    pcl::PointCloud<pcl::PointXYZI> pointcloud;
    sensor_msgs::PointCloud2 msg2loam;
    ros_njust_pca::NJUST_Sensor_st_Lidar32 msg2obs;
    m_veloConvert.DoNext(msg);
    stamp=m_veloConvert.GetFrame(pointcloud,msg2obs);
    m_veloConvert.ReSet();

    pcl::toROSMsg(pointcloud,msg2loam);
    msg2loam.header.stamp.fromSec(stamp/1000.0);
    msg2loam.header.frame_id="loam_input";
    m_pubPointCloud2.publish(msg2loam);
    m_pubPointCloudOrg.publish(msg2obs);

    // 保存点云到缓存队列
    pcl::PointCloud<pcl::PointXYZI> tmp_points;
    tmp_points.reserve(20000);
    sensor_msgs::PointCloud2 tmp_msg;
    for(auto &point:pointcloud)
    {
        if(point.x>=-20&&point.x<=20&&point.y>=0&&point.y<=25)
        {
            tmp_points.push_back(point);
        }
    }
    pcl::toROSMsg(tmp_points,tmp_msg);
    tmp_msg.header.stamp.fromSec(stamp/1000.0);
    m_pointsQue.push(tmp_msg);
}

// 接收分割图像数据，转换到车体栅格并存入缓存队列
void Fusion::SegmentationImageCallback(const sensor_msgs::Image::ConstPtr &msg)
{
    sensor_msgs::Image seg;
    seg=*msg;
    m_segQue.push(seg);
}


// 接收栅格数据，与分割结果融合后发布障碍物栅格
void Fusion::GridImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    // 转换为毫秒
    int64_t stamp = int64_t(msg->header.stamp.toSec()*1000);
    cv::Mat grid=cv::Mat::zeros(GRID_HEIGHT,GRID_WIDTH,CV_8U);
    ros_njust_pca::NJUST_Answer_st_Map fusionMsg;
    fusionMsg.timestamp=stamp;
    cv_bridge::CvImagePtr cv_ptr_grid=cv_bridge::toCvCopy(msg,"mono8");
    sensor_msgs::Image imgMsg;
    int x,y;
    // 找到最近的图像分割结果,融合
    if(m_segQue.size()>0&&FindNearestSeg(stamp,imgMsg))
    {
        cv_bridge::CvImagePtr cv_ptr_img = cv_bridge::toCvCopy(imgMsg, "mono8");
        cv::Mat img_grid(GRID_HEIGHT,GRID_WIDTH,CV_8U);
        GenerateGridfromImg(cv_ptr_img->image,img_grid);
        for (y = 0; y < GRID_HEIGHT; y++)
        {
            for(x = 0;x < GRID_WIDTH; x++)
            {
                if(img_grid.at<uchar>(y,x)>128)
                    grid.at<uchar>(y,x)=GRID_PASSABLE;
                uint8_t val=cv_ptr_grid->image.at<uchar>(y,x);
                grid.at<uchar>(y,x)=val>0?val:grid.at<uchar>(y,x);

            }
        }
        cv_bridge::CvImage(std_msgs::Header(), "mono8", grid).toImageMsg(fusionMsg.map);
    }
    else
    {
        // 没找到，不融合
        for (y = 0; y < GRID_HEIGHT; y++)
        {
            for(x = 0;x < GRID_WIDTH; x++)
            {
                uint8_t val=cv_ptr_grid->image.at<uchar>(y,x);
                grid.at<uchar>(y,x)=val>0?val:GRID_PASSABLE;
            }
        }
        cv_bridge::CvImage(std_msgs::Header(), "mono8", grid).toImageMsg(fusionMsg.map);
    }
    m_pubGridFusion.publish(fusionMsg);
}


// 将一帧点云映射到对应帧的图像的某个区域上，如果成功返回区域的中值
bool Fusion::ProjectLidar2ImageArea(int x1,int y1,int x2,int y2, sensor_msgs::PointCloud2 &cloud, Point3d &centerPt)
{
    pcl::PointCloud<pcl::PointXYZI> pointCloud;
    pcl::fromROSMsg(cloud,pointCloud);
    double x,y;
    Point3d vehiclePt;
    pcl::PointCloud<pcl::PointXYZI> pointsInBox;
    for(auto &point:pointCloud)
    {
        vehiclePt.x=point.x;
        vehiclePt.y=point.y;
        vehiclePt.z=point.z;
        m_monoTrans.VehicleP2ImageP(vehiclePt,x,y);
        if(x>=x1&&x<=x2&&y>=y1&&y<=y2)
        {
            pointsInBox.push_back(point);
        }
    }
    if(pointsInBox.empty())
        return false;
    else
    {
        // 去除离群点
//        printf("point number before filter is %d\n",pointsInBox.size());
        pcl::PointCloud<pcl::PointXYZI> selectedPoints;
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> filter;
        filter.setMeanK(10);
        filter.setStddevMulThresh(1.0);
        filter.setInputCloud(pointsInBox.makeShared());
        filter.filter(selectedPoints);
//        printf("point number after filter is %d\n",selectedPoints.size());
        if(selectedPoints.empty())
            return false;
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(selectedPoints,centroid);
        centerPt.x=centroid[0];
        centerPt.y=centroid[1];
        centerPt.z=centroid[2];
        return true;
    }
}


// 目标定位
void Fusion::ObjectDetectionCallback(const ros_njust_pca::NJUST_Answer_st_Object::ConstPtr &msg)
{
    bool isLocation=false;
    bool isTrust=false;   // 是否可信的标志，我们认为激光定位是可信的，而图像定位不可信
    int x1,y1,x2,y2;
    int64_t stamp = msg->timestamp;
    x1=msg->box[0];
    y1=msg->box[1];
    x2=msg->box[2];
    y2=msg->box[7];
    int box_width = x2 - x1;
    int box_height = y2 - y1;
    int box_size = box_width * box_height;
    ros_njust_pca::NJUST_Answer_st_Object fusionMsg;
    fusionMsg=*msg;
    ros_njust_pca::NJUST_Sensor_st_IMU curImuState;
    ros_njust_pca::NJUST_Sensor_st_GPS curGpsState;
    sensor_msgs::PointCloud2 curPoints;
    Point3d vehiclePt;
    double dist;
    // if location, before vanish point
    if(m_imuQue.size()>0&&m_gpsQue.size()>0
       &&FindNearestIMU(stamp,curImuState)
       &&FindNearestGPS(stamp,curGpsState)
       &&y2>400)    // 80m
    {
        /// 首选激光雷达定位
        if (m_pointsQue.size() > 0
            && FindNearestPointCloud(stamp, curPoints)
            && ProjectLidar2ImageArea(x1, y1, x2, y2, curPoints, vehiclePt))  // 找到邻近激光雷达帧
        {
            isTrust = true;
        }
        /// 定位失败选择图像定位
        if (!isTrust)
        {
            // 包围框底边中点转到车体坐标
            vehiclePt.z = WHEEL_RADIUS;
            m_monoTrans.ImageP2VehicleP(vehiclePt, (x1 + x2) / 2, y2);
        }
        // there must be a dist value
        dist = sqrt(vehiclePt.x * vehiclePt.x + vehiclePt.y * vehiclePt.y);
        if((isTrust&&dist<60)||(!isTrust&&dist<40))     // 50m
            isLocation=true;
    }
    // Location succeed
    if(isLocation)
    {
        // 转到gps坐标
        Point3d gpsPt;
        m_gpsCTrans.VehicleP2LocalP(vehiclePt, gpsPt);
        double roll=curImuState.roll;
        double pitch=curImuState.pitch;
        double yaw=curImuState.heading<=180?curImuState.heading:curImuState.heading-360;
        roll=roll*DEG_RAD;
        pitch=pitch*DEG_RAD;
        yaw=yaw*DEG_RAD;
        // 旋转转到世界坐标
        TransformLocal2WorldbyRPY(gpsPt, roll, pitch, yaw);
        // 根据GPS的当前经纬度获取目标经纬度
        double longitude, latitude;
        TransformWorld2LongitudeLatitude(gpsPt.x, gpsPt.y, curGpsState.longitude, curGpsState.latitude, &longitude, &latitude);
        fusionMsg.longitude = longitude;
        fusionMsg.latitude = latitude;

        // 再由经纬度转到世界坐标，并在栅格地图上标记
        double x, y;
        TransformLongitudeLatitude2World(longitude, latitude, m_longitudeOrg, m_latitudeOrg, &x, &y);
        pcl::PointXYZI point;
        point.x = x;
        point.y = y;
        point.z = msg->attribute;
        point.intensity = isTrust ? 1 : 0;

        /// judge dist and box size, if send object
        // person and car, almost do nothing
        if (msg->attribute == 0 || msg->attribute == 1)
        {
            m_pubObjectFusion.publish(fusionMsg);
            m_Objects.push_back(point);
        }
        // box, restrict
        if (msg->attribute == 2)
        {
            // send box image
            if ((dist <= 40 && box_size >= 900)||(dist > 40 && box_size < 2500))
            {
                m_pubObjectFusion.publish(fusionMsg);
                m_Objects.push_back(point);
            }
        }
    }
    // Location failed
    else
    {
        // person and car, almost do nothing
        if (msg->attribute == 0 || msg->attribute == 1)
        {
            m_pubObjectFusion.publish(fusionMsg);
        }
        // box, restrict
        if (msg->attribute == 2)
        {
            // send box image
            if (box_size <= 1000)
            {
                m_pubObjectFusion.publish(fusionMsg);
            }
        }
    }

#if 0
        // 再由经纬度转到世界坐标，并在栅格地图上标记
        double x, y;
        TransformLongitudeLatitude2World(longitude, latitude, m_longitudeOrg, m_latitudeOrg, &x, &y);
        int grid_x = x / MAP_RESOLUTION + MAP_WIDTH / 2;  // 东向
        int grid_y = MAP_HEIGHT / 2 - y / MAP_RESOLUTION; // 北向
        if (grid_x >= 0 && grid_x < MAP_WIDTH
            && grid_y > 0 && grid_y < MAP_HEIGHT) {
            if (msg->attribute == 0) {
                cv::putText(m_GlobalGridMap, "P", cv::Point(grid_x, grid_y), cv::FONT_HERSHEY_SIMPLEX, 1,
                            cv::Scalar(0, 255, 0), 1);
                if (isTrust)
                    cv::circle(m_GlobalGridMap, cv::Point(grid_x, grid_y), 10, cv::Scalar(0, 255, 0), -1);
                else
                    cv::drawMarker(m_GlobalGridMap, cv::Point(grid_x, grid_y), cv::Scalar(0, 139, 0),
                                   cv::MARKER_STAR, 20, 2);
            }
            if (msg->attribute == 1) {
                cv::putText(m_GlobalGridMap, "C", cv::Point(grid_x, grid_y), cv::FONT_HERSHEY_SIMPLEX, 1,
                            cv::Scalar(0, 255, 255), 1);
                if (isTrust)
                    cv::circle(m_GlobalGridMap, cv::Point(grid_x, grid_y), 10, cv::Scalar(0, 255, 255), -1);
                else
                    cv::drawMarker(m_GlobalGridMap, cv::Point(grid_x, grid_y), cv::Scalar(0, 205, 205),
                                   cv::MARKER_STAR, 20, 2);

            }
            if (msg->attribute == 2) {
                cv::putText(m_GlobalGridMap, "B", cv::Point(grid_x, grid_y), cv::FONT_HERSHEY_SIMPLEX, 1,
                            cv::Scalar(147, 20, 255), 1);
                if (isTrust)
                    cv::circle(m_GlobalGridMap, cv::Point(grid_x, grid_y), 10, cv::Scalar(147, 20, 255), -1);
                else
                    cv::drawMarker(m_GlobalGridMap, cv::Point(grid_x, grid_y), cv::Scalar(180, 105, 255),
                                   cv::MARKER_STAR, 20, 2);

            }

        }
#endif
}

// 根据雷达里程计计算经纬度和朝向角
void Fusion::LaserOdometeryCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    ros_njust_pca::NJUST_Answer_st_Location locationMsg;
    int64_t stamp=int64_t(msg->header.stamp.toSec()*1000);
    double heading,longitude,latitude;
    heading=m_headingOrg;
    //  LOAM雷达坐标转赛方雷达坐标
    double roll,pitch,yaw;
    geometry_msgs::Quaternion geoQuat=msg->pose.pose.orientation;
    // RPY逆时针为正
    tf::Matrix3x3(tf::Quaternion(geoQuat.z,-geoQuat.x,-geoQuat.y,geoQuat.w)).getRPY(roll,pitch,yaw);
    // 计算朝向角
    yaw=yaw/DEG_RAD;   // 顺时针?
    yaw=yaw>=0?yaw:yaw+360; // 转到0-360区间,顺时针转过的角度
    heading=heading+yaw;    // 加上初始姿态
    heading=heading<360?heading:heading-360;    // 对360取余
    // 计算坐标
    Point3d veloPt,vehiclePt,gpsPt;
    // 还原到雷达坐标
    veloPt.x=-msg->pose.pose.position.z;
    veloPt.y=-msg->pose.pose.position.x;
    veloPt.z=msg->pose.pose.position.y;
    // 转到车体坐标
    m_veloCTrans.LocalP2VehicleP(veloPt,vehiclePt);
    // 转到GPS坐标
    m_gpsCTrans.VehicleP2LocalP(vehiclePt,gpsPt);
    // 旋转到世界坐标
    roll=m_rollOrg*DEG_RAD;
    pitch=m_pitchOrg*DEG_RAD;
    yaw=m_yawOrg*DEG_RAD;
    TransformLocal2WorldbyRPY(gpsPt,roll,pitch,yaw);
    // 转到经纬度
    TransformWorld2LongitudeLatitude(gpsPt.x,gpsPt.y,m_longitudeOrg,m_latitudeOrg,&longitude,&latitude);

    locationMsg.timestamp=stamp;
    locationMsg.heading=heading;
    locationMsg.longitude=longitude;
    locationMsg.latitude=latitude;
    m_pubOdomFusion.publish(locationMsg);

    pcl::PointXYZ point;
    point.x=gpsPt.x;
    point.y=gpsPt.y;
    point.z=0;
    m_Traj.push_back(point);
}

void Fusion::LaserMapPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZI> pointCloud;
    pcl::fromROSMsg(*msg,pointCloud);
    pcl::PointCloud<pcl::PointXYZRGB> currentCloud;
    currentCloud.reserve(70000);
    Point3d veloPt,vehiclePt,gpsPt;
    pcl::PointXYZRGB pointRGB;
    for(auto &point:pointCloud)
    {
        // loam -> velo
        veloPt.x=-point.z;
        veloPt.y=-point.x;
        veloPt.z=point.y;
        // velo->vehicle
        m_veloCTrans.LocalP2VehicleP(veloPt,vehiclePt);
        // vehicle->gps
        m_gpsCTrans.VehicleP2LocalP(vehiclePt,gpsPt);
        double roll=m_rollOrg*DEG_RAD;
        double pitch=m_pitchOrg*DEG_RAD;
        double yaw=m_yawOrg*DEG_RAD;
        // gps->world
        TransformLocal2WorldbyRPY(gpsPt,roll,pitch,yaw);
        pointRGB.x=gpsPt.x;
        pointRGB.y=gpsPt.y;
        pointRGB.z=gpsPt.z;
        // colorized intensity
        int tmp=int(point.intensity)*8;
        tmp=tmp<255?tmp:255;
        pointRGB.r=abs(0-tmp);
        pointRGB.g=abs(127-tmp);
        pointRGB.b=abs(255-tmp);
        currentCloud.push_back(pointRGB);
    }
    // 点云深度滤波
    pcl::PointCloud<pcl::PointXYZRGB> filterCloud;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statistical_filter;
    statistical_filter.setMeanK(50);
    statistical_filter.setStddevMulThresh(1.0);
    statistical_filter.setInputCloud(currentCloud.makeShared());
    statistical_filter.filter(filterCloud);
    m_MapPointCloud += filterCloud;
}

// 根据Ins定位
void Fusion::INSOdometryCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    ros_njust_pca::NJUST_Answer_st_Location locationMsg;
    int64_t stamp=int64_t(msg->header.stamp.toSec()*1000);
    double heading,longitude,latitude;
    heading=msg->pose.orientation.z;
    TransformWorld2LongitudeLatitude(msg->pose.position.x,msg->pose.position.y,m_longitudeOrg,m_latitudeOrg,&longitude,&latitude);
    locationMsg.timestamp=stamp;
    locationMsg.heading=heading;
    locationMsg.longitude=longitude;
    locationMsg.latitude=latitude;
    m_pubOdomFusion.publish(locationMsg);
    m_poseQue.push(*msg);

    pcl::PointXYZ point;
    point.x=msg->pose.position.x;
    point.y=msg->pose.position.y;
    point.z=0;
    m_Traj.push_back(point);
    /**
    // draw trajectory on the global map
    int grid_x, grid_y;
    grid_x = msg->pose.position.x / MAP_RESOLUTION + MAP_WIDTH / 2;  // 东向
    grid_y = MAP_HEIGHT / 2 - msg->pose.position.y / MAP_RESOLUTION; // 北向
    if (grid_x >= 0 && grid_x < MAP_WIDTH
        && grid_y > 0 && grid_y < MAP_HEIGHT)
        cv::drawMarker(m_GlobalGridMap, cv::Point(grid_x, grid_y), cv::Scalar(255, 255, 240), cv::MARKER_STAR, 10, 2);
    **/
}

void Fusion::INSMapPointCloudCallback(const ros_njust_pca::NJUST_Sensor_st_Lidar32::ConstPtr &msg)
{
    static int count=0;
    int64_t stamp=msg->timestamp;
    geometry_msgs::PoseStamped pose;

    if(count++%60==0&&m_poseQue.size()>0&&FindNearestPose(stamp,pose))
    {
        pcl::PointCloud<pcl::PointXYZRGB> currentCloud;
        currentCloud.reserve(70000);
        pcl::PointXYZRGB pointRGB;
        double roll,yaw,pitch;      // 当前姿态角
        Point3d vehiclePt;
        roll=pose.pose.orientation.x;
        pitch=pose.pose.orientation.y;
        yaw=pose.pose.orientation.z;
        yaw=yaw<=180?yaw:yaw-360;
        roll=roll*DEG_RAD;
        pitch=pitch*DEG_RAD;
        yaw=yaw*DEG_RAD;
        for(auto &point:msg->points)
        {
            vehiclePt.x=point.x;
            vehiclePt.y=point.y;
            vehiclePt.z=point.z;
            TransformLocal2WorldbyRPY(vehiclePt,roll,pitch,yaw);
            pointRGB.x=vehiclePt.x+pose.pose.position.x;
            pointRGB.y=vehiclePt.y+pose.pose.position.y;
            pointRGB.z=vehiclePt.z+pose.pose.position.z;
//            printf("intensity is %d\n",point.intensity);
//            int tmp=int(point.intensity)*8;
//            tmp=tmp<255?tmp:255;
            float h=vehiclePt.z;
            h=h<=2?h:2;
            h=h>=-0.5?h:-0.5;
            int tmp=((h+0.5)/2.5)*255;
            tmp=tmp<255?tmp:255;
            pointRGB.r=abs(0-tmp);
            pointRGB.g=abs(127-tmp);
            pointRGB.b=abs(255-tmp);
            currentCloud.push_back(pointRGB);
        }
        // 点云深度滤波
//        pcl::PointCloud<pcl::PointXYZRGB> filterCloud;
//        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statistical_filter;
//        statistical_filter.setMeanK(50);
//        statistical_filter.setStddevMulThresh(1.0);
//        statistical_filter.setInputCloud(currentCloud.makeShared());
//        statistical_filter.filter(filterCloud);
        m_MapPointCloud += currentCloud;
    }
}


bool Fusion::FindNearestIMU(int64_t stamp,ros_njust_pca::NJUST_Sensor_st_IMU &out)
{
    size_t idx=0;
    int64_t timeDiff=stamp-m_imuQue[idx].timestamp;
    while (idx < m_imuQue.size() - 1 && timeDiff > 0)
    {
        idx++;
        timeDiff = stamp - m_imuQue[idx].timestamp;
    }
    if(idx==0||timeDiff>0)
    {
        if(abs(timeDiff)<10)
        {
            out=m_imuQue[idx];
            return true;
        }
        else
            return false;
    }
    else
    {
        size_t nearestIdx=abs(stamp-m_imuQue[idx-1].timestamp)<abs(timeDiff)?idx-1:idx;
        out=m_imuQue[nearestIdx];
        return true;
    }
}
bool Fusion::FindNearestGPS(int64_t stamp,ros_njust_pca::NJUST_Sensor_st_GPS &out)
{
    size_t idx=0;
    int64_t timeDiff=stamp-m_gpsQue[idx].timestamp;
    while (idx < m_gpsQue.size() - 1 && timeDiff > 0)
    {
        idx++;
        timeDiff = stamp - m_gpsQue[idx].timestamp;
    }
    if(idx==0||timeDiff>0)
    {
        if(abs(timeDiff)<10)
        {
            out=m_gpsQue[idx];
            return true;
        }
        else
            return false;
    }
    else
    {
        size_t nearestIdx=abs(stamp-m_gpsQue[idx-1].timestamp)<abs(timeDiff)?idx-1:idx;
        out=m_gpsQue[nearestIdx];
        return true;
    }
}
bool Fusion::FindNearestSeg(int64_t stamp,sensor_msgs::Image &out)
{
    size_t idx=0;
    // 时间的单位是毫秒
    int64_t timeDiff = int64_t(stamp-m_segQue[idx].header.stamp.toSec()*1000);
    while (idx < m_segQue.size() - 1 && timeDiff > 0)
    {
        idx++;
        timeDiff = int64_t(stamp - m_segQue[idx].header.stamp.toSec()*1000);
    }
    if(idx==0||timeDiff>0)
    {
        if(abs(timeDiff)<50)
        {
            out=m_segQue[idx];
            return true;
        }
        else
            return false;
    }
    else
    {
        size_t nearestIdx=abs(stamp-m_segQue[idx-1].header.stamp.toSec()*1000)<abs(timeDiff)?idx-1:idx;
        out=m_segQue[nearestIdx];
        return true;
    }

}
bool Fusion::FindNearestPointCloud(int64_t stamp,sensor_msgs::PointCloud2 &out)
{
    size_t idx=0;
    // 时间的单位是毫秒
    int64_t timeDiff = int64_t(stamp-m_pointsQue[idx].header.stamp.toSec()*1000);
    while (idx < m_pointsQue.size() - 1 && timeDiff > 0)
    {
        idx++;
        timeDiff = int64_t(stamp - m_pointsQue[idx].header.stamp.toSec()*1000);
    }
    if(idx==0||timeDiff>0)
    {
        if(abs(timeDiff)<50)
        {
            out=m_pointsQue[idx];
            return true;
        }
        else
            return false;
    }
    else
    {
        size_t nearestIdx=abs(stamp-m_pointsQue[idx-1].header.stamp.toSec()*1000)<abs(timeDiff)?idx-1:idx;
        out=m_pointsQue[nearestIdx];
        return true;
    }

}
bool Fusion::FindNearestPose(int64_t stamp, geometry_msgs::PoseStamped &out)
{
    size_t idx=0;
    // 时间的单位是毫秒
    int64_t timeDiff = int64_t(stamp-m_poseQue[idx].header.stamp.toSec()*1000);
    while (idx < m_poseQue.size() - 1 && timeDiff > 0)
    {
        idx++;
        timeDiff = int64_t(stamp - m_poseQue[idx].header.stamp.toSec()*1000);
    }
    if(idx==0||timeDiff>0)
    {
        if(abs(timeDiff)<50)
        {
            out=m_poseQue[idx];
            return true;
        }
        else
            return false;
    }
    else
    {
        size_t nearestIdx=abs(stamp-m_poseQue[idx-1].header.stamp.toSec()*1000)<abs(timeDiff)?idx-1:idx;
        out=m_poseQue[nearestIdx];
        return true;
    }

}

/*
void Fusion::ConstructLUTforGrid()
{
    for(int y=0;y<IMG_HEIGHT;y++)
    {
        for(int x=0;x<IMG_WIDTH;x++)
        {
            // 车体坐标
            Point3d vehiclePt;
            vehiclePt.z=WHEEL_RADIUS;
            m_monoTrans.ImageP2VehicleP(vehiclePt,x,y);
            // 栅格坐标
            int grid_x=vehiclePt.x/GRID_RESOLUTION+GRID_WIDTH/2;  // 车体栅格到栅格图的坐标
            int grid_y=GRID_HEIGHT-vehiclePt.y/GRID_RESOLUTION;
            if(grid_x>=0&&grid_x<GRID_WIDTH&&grid_y>=0&&grid_y<GRID_HEIGHT)
            {
                m_pPosXLUT[grid_y*GRID_WIDTH+grid_x]=x;
                m_pPosYLUT[grid_y*GRID_WIDTH+grid_x]=y;
            }
        }
    }
}
*/

void Fusion::ConstructLUTforGrid()
{
    for(int y=0;y<GRID_HEIGHT;y++)
    {
        for(int x=0;x<GRID_WIDTH;x++)
        {
            Point3d vehiclePt;
            double tmpX, tmpY;
            int imageX, imageY;
            vehiclePt.x = (x-GRID_WIDTH/2)*GRID_RESOLUTION;
            vehiclePt.y = (GRID_HEIGHT-y)*GRID_RESOLUTION;
            vehiclePt.z = 0;
            m_monoTrans.VehicleP2ImageP(vehiclePt, tmpX, tmpY);
            imageX = int(tmpX+0.5);
            imageY = int(tmpY+0.5);
            if(imageX>=0&&imageX<IMG_WIDTH&&imageY>=0&&imageY<IMG_HEIGHT)
            {
                m_pPosXLUT[y*GRID_HEIGHT+x] = imageX;
                m_pPosYLUT[y*GRID_HEIGHT+x] = imageY;
            }
        }
    }
}


void Fusion::GenerateGridfromImg(cv::Mat &src, cv::Mat &dst)
{
    int i,x,y;
    uint8_t grid[GRID_SIZE]={0};
    for (i = 0; i < GRID_SIZE; i++)
    {
        x=m_pPosXLUT[i];
        y=m_pPosYLUT[i];
        if(x>0&&y>0)    // 坐标是否有效
            grid[i]=src.at<uchar>(y,x);
    }
    memcpy(dst.data,grid,GRID_SIZE);
}

void Fusion::Debug()
{
    /**
    cv::Mat grid(GRID_HEIGHT,GRID_WIDTH,CV_8U);
    cv::Mat im=cv::imread("/media/yangfei/Repository/Datasets/KYXZ_2018/Testdata-001_0/Testdata-001-Camera/1530067507419464274.png",cv::IMREAD_GRAYSCALE);
    GenerateGridfromImg(im,grid);
    cv::namedWindow("src",CV_WINDOW_NORMAL);
    cv::namedWindow("dst",CV_WINDOW_NORMAL);
    cv::imshow("src",im);
    cv::imshow("dst",grid);
    cv::waitKey(0);
    **/
    if(!m_monoTrans.LoadCameraCalib("/home/njust1/Data/exam/Calib/Camera.camera"))
        return;
    Point3d vehiclePt;
    vehiclePt.z=0;
    /**
     * 400p->80m
     * 450p->21m
     * 500p->12m
     * **/
     FILE *fp=fopen("/home/njust1/dist.txt","wt");
     for(int pix=300;pix<600;pix++)
     {
         m_monoTrans.ImageP2VehicleP(vehiclePt,512,pix);
         fprintf(fp,"%d %f\n",pix,vehiclePt.y);
         printf("vehicle x=%lf\tvehicle y=%lf\n",vehiclePt.x,vehiclePt.y);
     }
     fclose(fp);
}


Fusion::Fusion()
{
    m_timestamp=0;
    m_longitudeOrg=0;
    m_latitudeOrg=0;
    m_altitudeOrg=0;
    m_rollOrg=0;
    m_pitchOrg=0;
    m_yawOrg=0;
    m_headingOrg=0;
    m_Traj.reserve(5000);
    m_realTraj.reserve(5000);
};
Fusion::~Fusion()=default;


/// 初始化
bool Fusion::Initialize(ros::NodeHandle &node,ros::NodeHandle &privateNode)
{
    ros::ServiceClient client = node.serviceClient<ros_njust_pca::NJUST_InitState>("NJUST_Service/InitState");
    ros_njust_pca::NJUST_InitState srv;
    ROS_INFO("fusion node try to get init IMU state ...");
    while (!client.call(srv))
    {
        usleep(100000);
    }
    ROS_INFO("fusion node get init IMU state succeed !");
    m_rollOrg=srv.response.roll;
    m_pitchOrg=srv.response.pitch;
    m_yawOrg=srv.response.heading<=180?srv.response.heading:srv.response.heading-360;
    m_headingOrg=srv.response.heading;
    m_longitudeOrg=srv.response.longitude;
    m_latitudeOrg=srv.response.latitude;
    m_altitudeOrg=srv.response.altitude;


    /// initialize calibration file
    std::string fileParam;
    if(privateNode.getParam("MapSaveFile",fileParam))
        m_MapSavePath=fileParam;
    else
        m_MapSavePath="/home/njust1/map.png";
    if(privateNode.getParam("GPSCalibFile",fileParam))
    {
        if (!m_gpsCTrans.LoadCalib(fileParam))
            return false;
    }
    else
    {
        if(!m_gpsCTrans.LoadCalib("/media/yangfei/Repository/Datasets/KYXZ_2018/Testdata-001_0/Testdata-001-Calib/Testdata-001-GPS.txt"))
            return false;
    }

    if(privateNode.getParam("CmeraCalibFile",fileParam))
    {
        if(!m_monoTrans.LoadCameraCalib(fileParam.c_str()))
            return false;
    }
    else
    {
        if(!m_monoTrans.LoadCameraCalib("/media/yangfei/Repository/Datasets/KYXZ_2018/Testdata-001_0/Testdata-001-Calib/Testdata-001-Camera.camera"))
            return false;
    }

    if(privateNode.getParam("VeloCalibFile",fileParam))
    {
        if(!m_veloCTrans.LoadCalib(fileParam)||!m_veloConvert.Initialize(fileParam.c_str()))
            return false;
    }
    else
    {
        if(!m_veloCTrans.LoadCalib("/media/yangfei/Repository/Datasets/KYXZ_2018/Testdata-001_0/Testdata-001-Calib/Testdata-001-HDL32-E.txt")
           || !m_veloConvert.Initialize("/media/yangfei/Repository/Datasets/KYXZ_2018/Testdata-001_0/Testdata-001-Calib/Testdata-001-HDL32-E.txt"))
            return false;
    }


    /// construct LUT of BEV image
    ConstructLUTforGrid();
    image_transport::ImageTransport transport(node);
    /// subscribe topics
    m_subIMU=node.subscribe<ros_njust_pca::NJUST_Sensor_st_IMU>("NJUST_Sensor/IMU",1,&Fusion::IMUCallback,this);
    m_subGPS=node.subscribe<ros_njust_pca::NJUST_Sensor_st_GPS>("NJUST_Sensor/GPS",1,&Fusion::GPSCallback,this);
    m_subVelodyneScan=node.subscribe<velodyne_msgs::VelodyneScan>("NJUST_Sensor/VelodyneScan",1,&Fusion::Lidar32Callback,this);
    m_subSeg=transport.subscribe("NJUST_Result/Segmentation",1,&Fusion::SegmentationImageCallback,this);
    m_subObject=node.subscribe<ros_njust_pca::NJUST_Answer_st_Object>("NJUST_Result/Object",10,&Fusion::ObjectDetectionCallback,this);
    m_subOdom=node.subscribe<nav_msgs::Odometry>("/integrated_to_init",1,&Fusion::LaserOdometeryCallback,this);
    m_subMap=node.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surround",1,&Fusion::LaserMapPointCloudCallback,this);

    m_subLocation=node.subscribe<geometry_msgs::PoseStamped>("NJUST_Result/Pose",1,&Fusion::INSOdometryCallback,this);
    m_subPointCloudOrg=node.subscribe<ros_njust_pca::NJUST_Sensor_st_Lidar32>("NJUST_Sensor/PointCloudOrg",1,&Fusion::INSMapPointCloudCallback,this);

    usleep(1000);   // 等待一定的缓存数据后开始融合
    m_subGrid=transport.subscribe("NJUST_Result/Grid",1,&Fusion::GridImageCallback,this);

    /// advertise topics
    m_pubImuTrans=node.advertise<sensor_msgs::Imu>("NJUST_Sensor/IMU2",1);
    m_pubPointCloud2=node.advertise<sensor_msgs::PointCloud2>("NJUST_Sensor/PointCloud2",1);
    m_pubPointCloudOrg=node.advertise<ros_njust_pca::NJUST_Sensor_st_Lidar32>("NJUST_Sensor/PointCloudOrg",1);

    m_pubObjectFusion=node.advertise<ros_njust_pca::NJUST_Answer_st_Object>("NJUST_Answer/Object",10);
    m_pubGridFusion=node.advertise<ros_njust_pca::NJUST_Answer_st_Map>("NJUST_Answer/Map",1);
    m_pubOdomFusion=node.advertise<ros_njust_pca::NJUST_Answer_st_Location>("NJUST_Answer/Location",1);
    return true;
}
// 执行
void Fusion::DoNext(ros::NodeHandle &node)
{
    bool isOK=false;
    ros::ServiceClient client = node.serviceClient<ros_njust_pca::NJUST_Command>("NJUST_Service/Command");
    ros_njust_pca::NJUST_Command srv;
    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        ros::spinOnce();
        client.call(srv);
        if(srv.response.cmd==0x33&&!isOK)
        {
            ROS_INFO("waiting for 5 secs util mapping finished!\n");
            if(!m_Traj.empty())     // environment model
            {
                usleep(5000000);    // 等待5s
                printf("point num before filter is %d\n",m_MapPointCloud.size());

                pcl::PointCloud<pcl::PointXYZRGB> filterCloud;
                // 深度滤波
                pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statistical_filter;
                statistical_filter.setMeanK(100);
                statistical_filter.setStddevMulThresh(1.0);
                statistical_filter.setInputCloud(m_MapPointCloud.makeShared());
                statistical_filter.filter(filterCloud);
                filterCloud.swap(m_MapPointCloud);

                filterCloud.clear();
                // 体素滤波
                pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
                voxel_filter.setLeafSize(0.1,0.1,0.1);
                voxel_filter.setInputCloud(m_MapPointCloud.makeShared());
                voxel_filter.filter(filterCloud);
                filterCloud.swap(m_MapPointCloud);

                printf("point num after filter is %d\n",m_MapPointCloud.size());


                // get map size by traj
                pcl::PointXYZ minPt,maxPt;
                pcl::getMinMax3D(m_Traj,minPt,maxPt);

                int width=2*(int((maxPt.x-minPt.x)/MAP_RESOLUTION)+1000);
                int height=2*(int((maxPt.y-minPt.y)/MAP_RESOLUTION)+1000);

                m_GlobalGridMap=cv::Mat::zeros(height,width,CV_8UC3);

                cv::line(m_GlobalGridMap,cv::Point(width/2-100,height/2),cv::Point(width/2+100,height/2),cv::Scalar(0,0,255),10);
                cv::line(m_GlobalGridMap,cv::Point(width/2,height/2-100),cv::Point(width/2,height/2+100),cv::Scalar(0,255,0),10);
                cv::putText(m_GlobalGridMap,"N",cv::Point(width/2,height/2-100),cv::FONT_HERSHEY_SIMPLEX,5,cv::Scalar(0,255,0),5);

                char strLon[256],strLat[256];
                sprintf(strLon,"Longitude:%lf",m_longitudeOrg);
                sprintf(strLat,"Latitude:%lf",m_latitudeOrg);
                cv::putText(m_GlobalGridMap,strLon,cv::Point(width/2+10,height/2-75),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0,0,255),2);
                cv::putText(m_GlobalGridMap,strLat,cv::Point(width/2+10,height/2+25),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0,0,255),2);

                int grid_x,grid_y;
                // map points
                for (auto &point:m_MapPointCloud)
                {
                    grid_x = point.x / MAP_RESOLUTION + width / 2;  // 东向
                    grid_y = height / 2 - point.y / MAP_RESOLUTION; // 北向
                    if (grid_x >= 0 && grid_x < width
                        && grid_y > 0 && grid_y < height)
                    {
//            cv::circle(m_GlobalGridMap,cv::Point(grid_x,grid_y),1,cv::Scalar(point.b,point.g,point.r),-1);
                        m_GlobalGridMap.at<cv::Vec3b>(grid_y, grid_x) = cv::Vec3b(point.b, point.g, point.r);
                    }
                }
                // traj
                for(auto &point:m_Traj)
                {
                    grid_x = point.x / MAP_RESOLUTION + width / 2;  // 东向
                    grid_y = height / 2 - point.y / MAP_RESOLUTION; // 北向
                    if (grid_x >= 0 && grid_x < width
                        && grid_y > 0 && grid_y < height)
                    {
                        cv::drawMarker(m_GlobalGridMap, cv::Point(grid_x, grid_y), cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 2);
                    }
                }
                // real traj
                for(auto &point:m_realTraj)
                {
                    grid_x = point.x / MAP_RESOLUTION + width / 2;  // 东向
                    grid_y = height / 2 - point.y / MAP_RESOLUTION; // 北向
                    if (grid_x >= 0 && grid_x < width
                        && grid_y > 0 && grid_y < height)
                    {
                        cv::drawMarker(m_GlobalGridMap, cv::Point(grid_x, grid_y), cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 10, 2);
                    }
                }
                /**
                FILE* fp=fopen("/home/njust1/input.txt","wt");
                fprintf(fp,"%lf %lf\n",m_latitudeOrg,m_longitudeOrg);
                fclose(fp);
                printf("save origin gps in input.txt succeed!\n");
                **/
                printf("origin latitude is %lf\torigin longitude is %lf\n",m_latitudeOrg,m_longitudeOrg);
            }
            else
            {
                // get map size by traj
                pcl::PointXYZ minPt, maxPt;
                pcl::getMinMax3D(m_realTraj, minPt, maxPt);

                int width = 2 * (int((maxPt.x - minPt.x) / MAP_RESOLUTION) + 1000);
                int height = 2 * (int((maxPt.y - minPt.y) / MAP_RESOLUTION) + 1000);

                m_GlobalGridMap = cv::Mat::zeros(height, width, CV_8UC3);

                cv::line(m_GlobalGridMap, cv::Point(width / 2 - 100, height / 2),
                         cv::Point(width / 2 + 100, height / 2), cv::Scalar(0, 0, 255), 10);
                cv::line(m_GlobalGridMap, cv::Point(width / 2, height / 2 - 100),
                         cv::Point(width / 2, height / 2 + 100), cv::Scalar(0, 255, 0), 10);
                cv::putText(m_GlobalGridMap, "N", cv::Point(width / 2, height / 2 - 100), cv::FONT_HERSHEY_SIMPLEX, 5,
                            cv::Scalar(0, 255, 0), 5);


                int grid_x, grid_y;

                // real traj
                for (auto &point:m_realTraj)
                {
                    grid_x = point.x / MAP_RESOLUTION + width / 2;  // 东向
                    grid_y = height / 2 - point.y / MAP_RESOLUTION; // 北向
                    if (grid_x >= 0 && grid_x < width
                        && grid_y > 0 && grid_y < height)
                    {
                        cv::drawMarker(m_GlobalGridMap, cv::Point(grid_x, grid_y), cv::Scalar(0, 255, 0),
                                       cv::MARKER_CROSS, 10, 2);
                    }
                }
                // objects
                for (auto &point:m_Objects)
                {
                    grid_x = point.x / MAP_RESOLUTION + width / 2;  // 东向
                    grid_y = height / 2 - point.y / MAP_RESOLUTION; // 北向
                    if (grid_x >= 0 && grid_x < width
                        && grid_y > 0 && grid_y < height)
                    {

                        if (point.z == 0)
                        {
                            cv::putText(m_GlobalGridMap, "P", cv::Point(grid_x, grid_y), cv::FONT_HERSHEY_SIMPLEX, 1,
                                        cv::Scalar(0, 255, 0), 1);
                            if (point.intensity == 1)
                                cv::circle(m_GlobalGridMap, cv::Point(grid_x, grid_y), 10, cv::Scalar(0, 255, 0), -1);
                            else
                                cv::drawMarker(m_GlobalGridMap, cv::Point(grid_x, grid_y), cv::Scalar(0, 139, 0),
                                               cv::MARKER_STAR, 20, 2);
                        }
                        if (point.z == 1)
                        {
                            cv::putText(m_GlobalGridMap, "C", cv::Point(grid_x, grid_y), cv::FONT_HERSHEY_SIMPLEX, 1,
                                        cv::Scalar(0, 255, 255), 1);
                            if (point.intensity == 1)
                                cv::circle(m_GlobalGridMap, cv::Point(grid_x, grid_y), 10, cv::Scalar(0, 255, 255), -1);
                            else
                                cv::drawMarker(m_GlobalGridMap, cv::Point(grid_x, grid_y), cv::Scalar(0, 205, 205),
                                               cv::MARKER_STAR, 20, 2);

                        }
                        if (point.z == 2)
                        {
                            cv::putText(m_GlobalGridMap, "B", cv::Point(grid_x, grid_y), cv::FONT_HERSHEY_SIMPLEX, 1,
                                        cv::Scalar(147, 20, 255), 1);
                            if (point.intensity == 1)
                                cv::circle(m_GlobalGridMap, cv::Point(grid_x, grid_y), 10, cv::Scalar(147, 20, 255),
                                           -1);
                            else
                                cv::drawMarker(m_GlobalGridMap, cv::Point(grid_x, grid_y), cv::Scalar(180, 105, 255),
                                               cv::MARKER_STAR, 20, 2);
                        }

                    }
                }
            }
            cv::imwrite(m_MapSavePath,m_GlobalGridMap);
            ROS_INFO("#########################################################\n");
            ROS_INFO("###################save map succeed!#####################\n");
            ROS_INFO("#########################################################\n");
            isOK=true;
        }
        loop_rate.sleep();
    }
}