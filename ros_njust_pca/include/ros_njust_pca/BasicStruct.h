#pragma once
#define BASIC_STRUCT_H
#include <cmath>
#include <iostream>
#include <stdio.h>
#include<vector>


#define MAX_EACH_LINE_LADAR_POINTS 2200

struct Lidar32Point
{
	int x;			//车体坐标系，厘米
	int y;
	int z;
	int azi;             //单位，0.01度
	int ver;
	int realDistance;     //单位厘米
	unsigned char Intensity;            //点的颜色强度，不同颜色代表不同的点类型
	unsigned char PointType;		//点的类型： 0、普通点;1、上升沿点;2、下降沿点; 
									//3、上升沿与下降沿之间的点;4、正障碍特征点(满足条件：在上升沿与下降沿之间且密度大于周围一定范围点);5、负障碍特征点
	Lidar32Point() {
		x = 0;
		y = 0;
		z = 0;
		azi = 0;
		ver = 0;
		realDistance = 0;
		Intensity = 0;
		PointType = 0;
	}
};


struct Lidar32Data
{
	Lidar32Point OneFrameData[32][2200];         //将每根线点数固定为2200
	unsigned int RealEachLinePointNum[32] = {0};                  //提取感兴趣区域后，每根线的实际点数,提取前，该数组无意义，全为0
	
};

struct Lidar32Grid
{
	std::vector<Lidar32Point>Points;
	unsigned int GridType;                     //栅格的类型，0-未知，1-正障碍，2-负障碍，3-可通行，4-水域
	unsigned int ClusterID;
	Lidar32Grid(){
		Points.clear();
		GridType = 0;
		ClusterID = 0;
	};

};

struct NegObsDetectedFeature
{
	//检测结果在当前坐标系下的坐标
	double StartX;
	double StartY;
	double EndX;
	double EndY;
	//检测结果在全局坐标系下的坐标
	double GlobalStartX;
	double GlobalStartY;
	double GlobalEndX;
	double GlobalEndY;
	NegObsDetectedFeature()
	{
		StartX = 0;
		StartY = 0;
		EndX = 0;
		EndY = 0;
		GlobalStartX = 0;
		GlobalStartY = 0;
		GlobalEndX = 0;
		GlobalEndY = 0;
	}
};