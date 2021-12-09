#pragma once
#define BASIC_STRUCT_H
#include <cmath>
#include <iostream>
#include <stdio.h>
#include<vector>


#define MAX_EACH_LINE_LADAR_POINTS 2200

struct Lidar32Point
{
	int x;			//��������ϵ������
	int y;
	int z;
	int azi;             //��λ��0.01��
	int ver;
	int realDistance;     //��λ����
	unsigned char Intensity;            //�����ɫǿ�ȣ���ͬ��ɫ����ͬ�ĵ�����
	unsigned char PointType;		//������ͣ� 0����ͨ��;1�������ص�;2���½��ص�; 
									//3�����������½���֮��ĵ�;4�����ϰ�������(���������������������½���֮�����ܶȴ�����Χһ����Χ��);5�����ϰ�������
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
	Lidar32Point OneFrameData[32][2200];         //��ÿ���ߵ����̶�Ϊ2200
	unsigned int RealEachLinePointNum[32] = {0};                  //��ȡ����Ȥ�����ÿ���ߵ�ʵ�ʵ���,��ȡǰ�������������壬ȫΪ0
	
};

struct Lidar32Grid
{
	std::vector<Lidar32Point>Points;
	unsigned int GridType;                     //դ������ͣ�0-δ֪��1-���ϰ���2-���ϰ���3-��ͨ�У�4-ˮ��
	unsigned int ClusterID;
	Lidar32Grid(){
		Points.clear();
		GridType = 0;
		ClusterID = 0;
	};

};

struct NegObsDetectedFeature
{
	//������ڵ�ǰ����ϵ�µ�����
	double StartX;
	double StartY;
	double EndX;
	double EndY;
	//�������ȫ������ϵ�µ�����
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