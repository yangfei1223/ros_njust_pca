//
// Created by yangfei on 18-8-28.
//

#ifndef PROJECT_NJUST_TRANSFORM_H
#define PROJECT_NJUST_TRANSFORM_H

#include "NJUST_Global.h"
#include "CoordinateTrans.h"
#include "MonoCameraTrans.h"

/// 翻滚角旋转
void rotYbyRoll(Point3d &p, double roll);
/// 俯仰角旋转
void rotXbyPitch(Point3d &p, double pitch);
/// 偏行角旋转
void rotZbyYaw(Point3d &p, double yaw);

/// 通过rpy角将当前坐标旋转到世界坐标(不考虑位移)
void TransformLocal2WorldbyRPY(Point3d &pt,double r, double p, double y);
/// 通过rpy角将世界坐标旋转到当前坐标
void TransformWorld2LocalbyRPY(Point3d &pt,double r, double p, double y);

/// 将世界坐标转换到经纬度
void TransformWorld2LongitudeLatitude(double x, double y, double longitudeOrg, double latitudeOrg, double *longitude, double *latitude);

void TransformLongitudeLatitude2World(double longitude,double latitude,double longitudeOrg, double latitudeOrg,double *x,double *y);

#endif //PROJECT_NJUST_TRANSFORM_H
