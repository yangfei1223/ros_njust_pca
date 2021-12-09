// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#include "loam_velodyne/MultiScanRegistration.h"
#include "math_utils.h"

#include <pcl_conversions/pcl_conversions.h>


namespace loam {

MultiScanMapper::MultiScanMapper(const float& lowerBound,
                                 const float& upperBound,
                                 const uint16_t& nScanRings)
    : _lowerBound(lowerBound),
      _upperBound(upperBound),
      _nScanRings(nScanRings),
      _factor((nScanRings - 1) / (upperBound - lowerBound))
{

}

void MultiScanMapper::set(const float &lowerBound,
                          const float &upperBound,
                          const uint16_t &nScanRings)
{
  _lowerBound = lowerBound;
  _upperBound = upperBound;
  _nScanRings = nScanRings;
  _factor = (nScanRings - 1) / (upperBound - lowerBound);
}



int MultiScanMapper::getRingForAngle(const float& angle) {
  return int(((angle * 180 / M_PI) - _lowerBound) * _factor + 0.5);
}






MultiScanRegistration::MultiScanRegistration(const MultiScanMapper& scanMapper,
                                             const RegistrationParams& config)
    : ScanRegistration(config),
      _systemDelay(SYSTEM_DELAY),
      _scanMapper(scanMapper)
{

};



bool MultiScanRegistration::setup(ros::NodeHandle& node,
                                  ros::NodeHandle& privateNode)
{
    // 父类初始化
  if (!ScanRegistration::setup(node, privateNode)) {
    return false;
  }

  // fetch scan mapping params
  std::string lidarName;
  // 设置参数
  if (privateNode.getParam("lidar", lidarName)) {   // 如果有设置雷达
    if (lidarName == "VLP-16") {
      _scanMapper = MultiScanMapper::Velodyne_VLP_16();
    } else if (lidarName == "HDL-32") {
      _scanMapper = MultiScanMapper::Velodyne_HDL_32();     // 构造32线雷达的类
    } else if (lidarName == "HDL-64E") {
      _scanMapper = MultiScanMapper::Velodyne_HDL_64E();
    } else {
      ROS_ERROR("Invalid lidar parameter: %s (only \"VLP-16\", \"HDL-32\" and \"HDL-64E\" are supported)", lidarName.c_str());
      return false;
    }

    ROS_INFO("Set  %s  scan mapper.", lidarName.c_str());
    if (!privateNode.hasParam("scanPeriod")) {
      _config.scanPeriod = 0.1;     // 默认设为0.1s,即100ms(velodyne雷达的扫描周期)
      ROS_INFO("Set scanPeriod: %f", _config.scanPeriod);
    }
  } else {  /// 设置了雷达类型之后不会执行到此块语句
    float vAngleMin, vAngleMax;     // 最大最小锤子角度
    int nScanRings;     // 扫描线个数

    if (privateNode.getParam("minVerticalAngle", vAngleMin) &&
        privateNode.getParam("maxVerticalAngle", vAngleMax) &&
        privateNode.getParam("nScanRings", nScanRings)) {
      if (vAngleMin >= vAngleMax) {
        ROS_ERROR("Invalid vertical range (min >= max)");
        return false;
      } else if (nScanRings < 2) {
        ROS_ERROR("Invalid number of scan rings (n < 2)");
        return false;
      }

      _scanMapper.set(vAngleMin, vAngleMax, nScanRings);
      ROS_INFO("Set linear scan mapper from %g to %g degrees with %d scan rings.", vAngleMin, vAngleMax, nScanRings);
    }
  }


  // subscribe to input cloud topic
    /// 订阅点云数据(pointcloud2格式xyz点)，要注意此时的雷达坐标系是什么
  _subLaserCloud = node.subscribe<sensor_msgs::PointCloud2>
      ("/multi_scan_points", 2, &MultiScanRegistration::handleCloudMessage, this);

  return true;
}



void MultiScanRegistration::handleCloudMessage(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
  /// 延时操作，可能是在等待imu数据
  if (_systemDelay > 0) {
    _systemDelay--;
    return;
  }

  // fetch new input cloud
  pcl::PointCloud<pcl::PointXYZ> laserCloudIn;    // 输入点云
  pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);    // 消息转换

  process(laserCloudIn, laserCloudMsg->header.stamp);   // 处理
}



void MultiScanRegistration::process(const pcl::PointCloud<pcl::PointXYZ>& laserCloudIn,
                                    const ros::Time& scanTime)
{
  size_t cloudSize = laserCloudIn.size();   /// 点云大小

  // reset internal buffers and set IMU start state based on current scan time
  reset(scanTime);  /// scanTime是订阅的点云消息的时间戳，××理论上是获得第一个点的时间××

  // determine scan start and end orientations
    /// 计算的是横向扫描角y/x夹角    计算这个角的时候用的还是velodyne的坐标
  float startOri = -std::atan2(laserCloudIn[0].y, laserCloudIn[0].x);
  float endOri = -std::atan2(laserCloudIn[cloudSize - 1].y,
                             laserCloudIn[cloudSize - 1].x) + 2 * float(M_PI);
  if (endOri - startOri > 3 * M_PI) {
    endOri -= 2 * M_PI;
  } else if (endOri - startOri < M_PI) {
    endOri += 2 * M_PI;
  }

  bool halfPassed = false;      // 判断是否过了180度，用于计算横向角
  pcl::PointXYZI point;
    /// 每条扫描线的点云向量，下面这段代码使用垂直角度将点云分线存储
    /// 其实这段代码的话在我们的程序中可以直接根据原始数据获得，不必再进行转换
  std::vector<pcl::PointCloud<pcl::PointXYZI> > laserCloudScans(_scanMapper.getNumberOfScanRings());

  // extract valid points from input cloud
  for (int i = 0; i < cloudSize; i++) {
      ///     **这个赋值是从topic的雷达坐标转换到作者定义的雷达坐标**  这个转换很隐晦，不注意是看不懂的
    point.x = laserCloudIn[i].y;
    point.y = laserCloudIn[i].z;
    point.z = laserCloudIn[i].x;

    // skip NaN and INF valued points
    if (!pcl_isfinite(point.x) ||
        !pcl_isfinite(point.y) ||
        !pcl_isfinite(point.z)) {
      continue;
    }

    // skip zero valued points
    if (point.x * point.x + point.y * point.y + point.z * point.z < 0.0001) {
      continue;
    }

    // calculate vertical point angle and scan ID
      /// 计算纵向扫描角的时候用的确实作者定义的雷达坐标，很迷惑人
    float angle = std::atan(point.y / std::sqrt(point.x * point.x + point.z * point.z));
    int scanID = _scanMapper.getRingForAngle(angle);
    if (scanID >= _scanMapper.getNumberOfScanRings() || scanID < 0 ){
      continue;
    }

    // calculate horizontal point angle
      /// 计算每个点的横向扫描角，以此在计算每个点的时间戳，velodyne的话是32条线并行的
    float ori = -std::atan2(point.x, point.z);
    if (!halfPassed) {
      if (ori < startOri - M_PI / 2) {
        ori += 2 * M_PI;
      } else if (ori > startOri + M_PI * 3 / 2) {
        ori -= 2 * M_PI;
      }

      if (ori - startOri > M_PI) {
        halfPassed = true;      // 扫描已过180度
      }
    } else {
      ori += 2 * M_PI;

      if (ori < endOri - M_PI * 3 / 2) {
        ori += 2 * M_PI;
      } else if (ori > endOri + M_PI / 2) {
        ori -= 2 * M_PI;
      }
    }

    // calculate relative scan time based on point orientation
      /// 根据横向扫描角确定每个点的扫描时间
    float relTime = _config.scanPeriod * (ori - startOri) / (endOri - startOri);
    point.intensity = scanID + relTime;     /// intensity整数部分是线号，小数部分是扫描时间，作者pcl用得6

    // project point to the start of the sweep using corresponding IMU data
      ///  下面这段代码很关键
    if (hasIMUData()) {     /// 如果有imu数据
      setIMUTransformFor(relTime);      /// 找到最近的imu数据
      transformToStartIMU(point);   /// 转换坐标
    }

    laserCloudScans[scanID].push_back(point);   /// 存入点云
  }

  // construct sorted full resolution cloud
  cloudSize = 0;
  for (int i = 0; i < _scanMapper.getNumberOfScanRings(); i++) {
    _laserCloud += laserCloudScans[i];

    IndexRange range(cloudSize, 0);     /// pair的第一个是每条线点云在总点云中的起始下标
    cloudSize += laserCloudScans[i].size();
    range.second = cloudSize > 0 ? cloudSize - 1 : 0;   /// pair的第二个是每条点云在总点云中的终止下标
    _scanIndices.push_back(range);  /// 大小应该是32
  }

  // extract features
  extractFeatures();    /// 特征提取

  // publish result
  publishResult();      /// 数据发布
}

} // end namespace loam
