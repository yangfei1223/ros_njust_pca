#pragma once
#include "BasicStruct.h"
#include "ros_njust_pca/NJUST_Global.h"
typedef unsigned short int UINT16;
class CLidar32OD
{
public:
	CLidar32OD();
	~CLidar32OD();
	void ViewLidar32(Lidar32Data* p32,int frame);
	void SortLidar32(Lidar32Data* p32);
	void InitGridMap();
	void GridMap(Lidar32Data* p32);
	void GridProce(Lidar32Grid** pGrid32);
	void DetectNOLocalConvexity(Lidar32Data *p_l32data, int NeighborNum);
	void GetNOConvexPoints(Lidar32Data *p_l32data, int LineID, unsigned int* ConvexPair, int ValidCPNum, int MaxNeighDist=100, int MinNeighDist=10);
	void GetPointDensity(Lidar32Data *p_l32data, int* DensityArray, int LineID, int LeftInd, int RightInd);
	double GetScanRadiusDiff(int LineID);
	void FindMatchPoints(Lidar32Data *p_l32data);
	void PODFromLidar32();
	int InputData(int64_t stamp, Lidar32Data *p32);		// 重载
	int64_t timestamp;      //每帧的时间戳
	int64_t GetGrid(cv::Mat &grid)
	{
		grid=m_GridMat.clone();
		return timestamp;
	}

private:

	cv::Rect RangRect32;
	Lidar32Data* pData32;
	Lidar32Data* pSortData32;
	int m_PointNumEachLine;
	std::vector<int>	m_NOInds;
	std::vector<Lidar32Point> m_NegObsFeaturePairs;
	//LOCAL_POS_DATA *m_pLocalPoseData;
	int last_i ;

	int m_GridMapWidth;
	int m_GridMapHeight;
	cv::Mat m_GridMat;

	Lidar32Grid** pGridData32;

	int m_NegObsNum;
	int m_MaxSize;
	char* m_VistFlag;
	UINT16* m_NeighNum;
	UINT16* m_ClusterID;
	UINT16* m_KeyPointInds;
};
