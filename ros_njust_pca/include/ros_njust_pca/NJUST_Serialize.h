//
// Created by yangfei on 18-8-4.
//

#ifndef NJUST_PCALAB_NJUST_SERIALIZE_H
#define NJUST_PCALAB_NJUST_SERIALIZE_H

#include "NJUST_Global.h"

/// 载入任务文件
void LoadTaskFile(const char *filename);

/// 寻找任务文件时间戳，以确定是否发送答案数据
int64_t FindTheNearestTaskStamp(int64_t stamp,std::vector<int64_t> &vec, int thre);

/// 目标检测结果回调处理函数
void ObjectResCallback(const ros_njust_pca::NJUST_Answer_st_Object::ConstPtr &msg);
/// 局部地图回调处理函数
void LocalMapResCallback(const ros_njust_pca::NJUST_Answer_st_Map::ConstPtr &msg);
/// 定位结果回调处理函数
void LocationResCallback(const ros_njust_pca::NJUST_Answer_st_Location::ConstPtr &msg);

#endif //NJUST_PCALAB_NJUST_SERIALIZE_H
