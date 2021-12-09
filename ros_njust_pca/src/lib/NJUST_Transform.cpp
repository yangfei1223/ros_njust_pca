//
// Created by yangfei on 18-8-28.
//

#include "ros_njust_pca/NJUST_Transform.h"

/// 按照官方定义的反方向旋转，有些是顺时针，有些是逆时针
/// 翻滚角旋转
void rotYbyRoll(Point3d &pt, double roll)
{
    double x=pt.x;
    double z=pt.z;
    pt.x=x*cos(roll)+z*sin(roll);
    pt.z=z*cos(roll)-x*sin(roll);
}
/// 俯仰角旋转
void rotXbyPitch(Point3d &pt, double pitch)
{
    double y=pt.y;
    double z=pt.z;
    pt.y=y*cos(pitch)-z*sin(pitch);
    pt.z=z*cos(pitch)+y*sin(pitch);
}

/// 偏行角旋转
void rotZbyYaw(Point3d &pt, double yaw)
{
    double x=pt.x;
    double y=pt.y;
    pt.x=x*cos(yaw)+y*sin(yaw);
    pt.y=y*cos(yaw)-x*sin(yaw);
}

/// 通过rpy角将当前坐标旋转到世界坐标(不考虑位移)
void TransformLocal2WorldbyRPY(Point3d &pt,double r, double p, double y)
{
    // Y->X->Z
    rotYbyRoll(pt,r);
    rotXbyPitch(pt,p);
    rotZbyYaw(pt,y);

}
/// 通过rpy角将世界坐标旋转到当前坐标
void TransformWorld2LocalbyRPY(Point3d &pt,double r, double p, double y)
{
    // Z->X->Y
    rotZbyYaw(pt,-y);
    rotXbyPitch(pt,-p);
    rotYbyRoll(pt,-r);
}

/// 将世界坐标转换到经纬度
void TransformWorld2LongitudeLatitude(double x, double y, double longitudeOrg, double latitudeOrg, double *longitude, double *latitude)
{
    // x->East,y->North
    *latitude=y/(RE*DEG_RAD)+latitudeOrg;
    *longitude=x/(RE*DEG_RAD*cos(latitudeOrg*DEG_RAD))+longitudeOrg;
}

void TransformLongitudeLatitude2World(double longitude,double latitude,double longitudeOrg, double latitudeOrg,double *x,double *y)
{
    *y=(latitude-latitudeOrg)*DEG_RAD*RE;
    *x=(longitude-longitudeOrg)*DEG_RAD*(RE*cos(latitudeOrg*DEG_RAD));
}
