#include"ros_njust_pca/Lidar32OD.h"
#include <math.h>
#include<vector>


#define MIN_SORTED_LIDAR32_ID		         1				//包括该ID
#define MAX_SORTED_LIDAR32_ID		         22				//包括该ID
#define LIDAR_HEIGHT				         212
#define OBS_PI                               3.141592654



//正障碍栅格阈值
#define OBS_GRID_WIDTH                       50             //栅格大小，单位cm
#define HEIGHT_THRESH                        35             //栅格内，高度差阈值，单位cm
#define VEHICLE_RANGE_HEIGHT_THRESH          50             //车前20m，左右2m的范围内，高度差阈值为VEHICLE_RANGE_HEIGHT_THRESH，单位cm
#define GRID_RANGE                           2              //用于构造n x n的网格，单障碍点滤除


//负障碍阈值
#define COMVEX_DISTOFF_THRESH                20             //上升沿高度阈值
#define NEG_COMVEX_DISTOFF_THRESH            -20            //下降沿高度阈值
#define MAX_CONVEX_PAIR				         60             //上升沿下降沿点对的最大对数
#define NO_COMVEX_THRESH                     35
#define NEG_OBS_MIN_LENGTH                   40
#define NEG_OBS_MAX_LENGTH                   240
#define NEG_OBS_MIN_WIDTH			         20				//负障碍的最小纵向宽度(非凸起点)
#define NEG_OBS_MIN_WIDTH_BULGE		         40				//负障碍的最小纵向宽度(凸起点)
#define NEG_OBS_MAX_WIDTH			         200		    //负障碍的最大纵向宽度
#define DESITY_THRESH				         0.75           //0.75
#define GAMMA_DENSITY_LIDAR32                0.55
#define GAMMA_ONE					         1.5            //1.5
#define GAMMA_TWO					         0.25


double DefaultRadius[32] = {336.8383897,355.9589639,376.1452931,398.1749812,422.5305219,449.2073548,478.8194495,512.1809147,            //理想情况下，每根线在XY平面的投影长度
							549.4954839,591.8948725,640.9438732,697.4828888,763.9841659,844.0851882,940.9260219,1061.516345,
							1217.330161,1423.073944,1710.246460,2143.730124,2860.133251,4288.711554,8614.359113,1200.000000,
							0,0,0,0,0,0,0,0};
CLidar32OD::CLidar32OD()
{
	RangRect32.x = -2025;                        //-2000
	RangRect32.width = 4050;                    //4000
	RangRect32.y = 0;                           //0
	RangRect32.height = 4050;                   //4000
	pData32 = new Lidar32Data;
	pSortData32 = new Lidar32Data;
	last_i = 0;

    m_NOInds.reserve(10000);
    m_NegObsFeaturePairs.reserve(1000);

	m_GridMapWidth = RangRect32.width / OBS_GRID_WIDTH;              //栅格初始化
	m_GridMapHeight = RangRect32.height / OBS_GRID_WIDTH;
	m_GridMat.create(m_GridMapHeight, m_GridMapWidth, CV_8UC1);


	pGridData32 = new Lidar32Grid*[m_GridMapHeight];
	for (int i = 0; i < m_GridMapHeight; i++)
	{
		pGridData32[i] = new Lidar32Grid[m_GridMapWidth];
	}
}

CLidar32OD::~CLidar32OD()
{
	if (pData32 != NULL) delete pData32;
	if (pSortData32 != NULL) delete pSortData32;

	for (int i = 0; i < m_GridMapWidth; i++)
	{
		delete[] pGridData32[i];
	}
	delete pGridData32;



}


double CLidar32OD::GetScanRadiusDiff(int LineID)
{
	return	DefaultRadius[LineID] - DefaultRadius[LineID-1];
}



void CLidar32OD::ViewLidar32(Lidar32Data* p32,int frame)            //显示雷达视图
{
	int MatRow = RangRect32.height, MatCol = RangRect32.width;
	cv::Mat mat_32 = cv::Mat::zeros(MatRow, MatCol, CV_8UC3);

	int Row, Col, RPt = 2;      // Rpt点半径
	int XBaseLine = RangRect32.x;
	int YBaseLine = RangRect32.height + RangRect32.y;
	int Num;
	for (int LineID = 0; LineID < 32; LineID++)
	{
		Num = p32->RealEachLinePointNum[LineID];
		for (int pID = 0; pID < Num; pID++)
		{

			if (p32->OneFrameData[LineID][pID].realDistance > 0)
			{
				Row = YBaseLine - p32->OneFrameData[LineID][pID].y;
				Col = p32->OneFrameData[LineID][pID].x - XBaseLine;
				Lidar32Point pP = p32->OneFrameData[LineID][pID];
				if (pP.PointType == 0|| pP.PointType == 1 || pP.PointType == 2 || pP.PointType == 3)
				{
					for (int i = Row - RPt; i < Row + RPt; i++)
					{
						for (int j = Col - RPt; j < Col + RPt; j++)
						{
							if (i >= 0 && i < MatRow && j >= 0 && j < MatCol)
							{
								mat_32.at<cv::Vec3b>(i, j)[0] = 255;          //0,1,2,3，  均为白色
								mat_32.at<cv::Vec3b>(i, j)[1] = 255;
								mat_32.at<cv::Vec3b>(i, j)[2] = 255;
							}
						}
					}
				}
				else
				{
					for (int i = Row - 2 * RPt; i < Row + 2 * RPt; i++)
					{
						for (int j = Col - 2 * RPt; j < Col + 2 * RPt; j++)
						{
							if (i >= 0 && i < MatRow && j >= 0 && j < MatCol)
							{
								//测试，4，正障碍点，红色   5，负障碍点，蓝色
								if (p32->OneFrameData[LineID][pID].PointType == 4)
								{
									mat_32.at<cv::Vec3b>(i, j)[0] = 0;
									mat_32.at<cv::Vec3b>(i, j)[1] = 0;
									mat_32.at<cv::Vec3b>(i, j)[2] = 255;
								}
								else if (p32->OneFrameData[LineID][pID].PointType == 5)
								{
									mat_32.at<cv::Vec3b>(i, j)[0] = 255;
									mat_32.at<cv::Vec3b>(i, j)[1] = 0;
									mat_32.at<cv::Vec3b>(i, j)[2] = 0;
								}
								else if (p32->OneFrameData[LineID][pID].PointType == 6)
								{
									mat_32.at<cv::Vec3b>(i, j)[0] = 0;
									mat_32.at<cv::Vec3b>(i, j)[1] = 255;
									mat_32.at<cv::Vec3b>(i, j)[2] = 0;
								}
							}
						}
					}
				}
			}
		}
	}
	for (std::vector<Lidar32Point>::iterator it = m_NegObsFeaturePairs.begin(); it != m_NegObsFeaturePairs.end();)
	{
		//if ((it + 1)->ClusterID != 0)
		{
			Lidar32Point PointC = *it++;
			Lidar32Point PointB = *it++;
			Lidar32Point PointA = *it++;
			cv::line(mat_32, cv::Point((PointA.x - XBaseLine), (YBaseLine - PointA.y)), \
			         cv::Point((PointB.x - XBaseLine), (YBaseLine - PointB.y)), cv::Scalar(255, 0, 0), 2, 8, 0);
			cv::line(mat_32, cv::Point((PointB.x - XBaseLine), (YBaseLine - PointB.y)), \
			         cv::Point((PointC.x - XBaseLine), (YBaseLine - PointC.y)), cv::Scalar(255, 255, 255), 2, 8, 0);
		}
	}
	m_NegObsFeaturePairs.clear();

	char FormatString[256];
	sprintf(FormatString, "Frame#%d", frame);
	cv::putText(mat_32, FormatString, cv::Point(20, 150), CV_FONT_HERSHEY_DUPLEX, 4, cv::Scalar(0, 0, 255), 3, 8, 0);

	cv::namedWindow("Lidar32", CV_WINDOW_NORMAL);
	cv::imshow("Lidar32", mat_32);
	cvWaitKey(20);
}



void CLidar32OD::InitGridMap()           //每帧处理前，清空某些东西
{
    memset(m_GridMat.data, 0, m_GridMapHeight*m_GridMapWidth);  //可视化矩阵清空
    for (int i = 0; i < m_GridMapHeight; i++)            //栅格地图中，每个栅格结构体清空
    {
        for (int j = 0; j < m_GridMapWidth; j++)
        {

            pGridData32[i][j].Points.clear() ;
            pGridData32[i][j].GridType = 0;
        }
    }
}



void CLidar32OD::GridMap(Lidar32Data* p32)    //提取感兴趣区域，并建立栅格图
{
    int Row, Col;
    for (int LineID=0;LineID<31 && p32->RealEachLinePointNum[LineID]>0 ;LineID++)
    {
        for (int pID = 0; pID < p32->RealEachLinePointNum[LineID]; pID++)
        {

            Row = (RangRect32.height - p32->OneFrameData[LineID][pID].y) / OBS_GRID_WIDTH;
            Col = (p32->OneFrameData[LineID][pID].x - RangRect32.x) / OBS_GRID_WIDTH;
            pGridData32[Row][Col].Points.push_back ( p32->OneFrameData[LineID][pID]);


        }
    }
}



/**
void CLidar32OD::GridProce(Lidar32Grid** pGrid32)
{
    unsigned short Max_Height, Min_Height;

    for (int i = 0; i < m_GridMapHeight; i++)
    {
        for (int j = 0; j < m_GridMapWidth; j++)
        {
            if (pGrid32[i][j].Points.size() > 1)
            {
                Min_Height = pGrid32[i][j].Points[0].z;
                Max_Height = pGrid32[i][j].Points[0].z;
                for (int n = 1; n < pGrid32[i][j].Points.size(); n++)
                {
                    Min_Height = pGrid32[i][j].Points[n].z < Min_Height ? pGrid32[i][j].Points[n].z : Min_Height;
                    Max_Height = pGrid32[i][j].Points[n].z > Min_Height ? pGrid32[i][j].Points[n].z : Max_Height;
                }
                if (i >= 35 && i <= 45 && j > 20 && j <= 80)                     //车前20m，左右2m的范围内，高度差阈值为VEHICLE_RANGE_HEIGHT_THRESH
                {
                    if (Max_Height - Min_Height > VEHICLE_RANGE_HEIGHT_THRESH && pGrid32[i][j].Points.size() > 2)   //悬空点滤除
                    {
                        pGrid32[i][j].GridType = 1;
                        m_GridMat.at<uchar>(i, j) = GRID_POSITIVE;                //移植过去，需改为正障碍标签
                    }
                }
                else                           //其他范围，高度差阈值为HEIGHT_THRESH
                {
                    if (Max_Height - Min_Height > HEIGHT_THRESH && pGrid32[i][j].Points.size() > 2)   //悬空点滤除
                    {
                        pGrid32[i][j].GridType = 1;
                        m_GridMat.at<uchar>(i, j) = GRID_POSITIVE;               //移植过去，需改为正障碍标签
                    }
                }
            }
        }
    }
////////////////////单点障碍滤除////////////////////////网格内障碍栅格数目小于阈值，则删除该障碍栅格
    for (int i = 0; i < m_GridMapHeight; i++)
    {
        for (int j = 0; j < m_GridMapWidth; j++)
        {
            int ObstacleGridNum = 0;
            if(pGrid32[i][j].GridType==1)
            {
                int NeiLeft = j - GRID_RANGE > 0 ? (j - GRID_RANGE) : 0;                            //（2*GRID_RANGE+1）x（2*GRID_RANGE+1）网格
                int NeiRight = j + GRID_RANGE < m_GridMapWidth ? (j + GRID_RANGE) : m_GridMapWidth;
                int NeiBottom = i - GRID_RANGE > 0 ? (i - GRID_RANGE) : 0;
                int NeiTop = i + GRID_RANGE < m_GridMapHeight ? (i + GRID_RANGE) : m_GridMapHeight;
                for (int Temp_i = NeiBottom; Temp_i < NeiTop; Temp_i++)
                {
                    for (int Temp_j = NeiLeft; Temp_j < NeiRight; Temp_j++)
                    {
                        if (pGrid32[Temp_i][Temp_j].GridType == 1)
                            ObstacleGridNum++;
                    }
                }
                if (ObstacleGridNum <= 2)         // || !GridLine(pGrid32[i][j])
                {
                    pGrid32[i][j].GridType == 0;
                    m_GridMat.at<uchar>(i, j) = 0;           //将对应的像素点标记为黑色（未知区域）
                }
            }
        }
    }
}
**/
void CLidar32OD::GridProce(Lidar32Grid** pGrid32)
{
    unsigned short Max_Height, Min_Height;
    std::vector<int> RoadPosGridInds;
    for (int i = 0; i < m_GridMapHeight; i++)
    {
        for (int j = 0; j < m_GridMapWidth; j++)
        {
            if (pGrid32[i][j].Points.size() > 1)
            {
                Min_Height = pGrid32[i][j].Points[0].z;
                Max_Height = pGrid32[i][j].Points[0].z;

                for (int n = 1; n < pGrid32[i][j].Points.size(); n++)
                {
                    Min_Height = pGrid32[i][j].Points[n].z < Min_Height ? pGrid32[i][j].Points[n].z : Min_Height;
                    Max_Height = pGrid32[i][j].Points[n].z > Min_Height ? pGrid32[i][j].Points[n].z : Max_Height;
                }
                if (i >=0 && i <= 80 && j > 35 && j < 45  )                     //车前20m，左右2m的范围内，高度差阈值为VEHICLE_RANGE_HEIGHT_THRESH
                {
                    if (Max_Height - Min_Height > HEIGHT_THRESH && pGrid32[i][j].Points.size() > 2)   //悬空点滤除
                    {
                        pGrid32[i][j].GridType = 1;
                        m_GridMat.at<uchar>(i, j) = GRID_POSITIVE;
                        RoadPosGridInds.push_back(i);
                        RoadPosGridInds.push_back(j);
                    }
                }
                else                           //其他范围，高度差阈值为HEIGHT_THRESH
                {
                    if (Max_Height - Min_Height > HEIGHT_THRESH && pGrid32[i][j].Points.size() > 2)   //悬空点滤除
                    {
                        pGrid32[i][j].GridType = 1;
                        m_GridMat.at<uchar>(i, j) = GRID_POSITIVE;
                    }
                }
            }
        }
    }
////////////////////单点障碍滤除////////////////////////网格内障碍栅格数目小于阈值，则删除该障碍栅格
    for (int i = 0; i < m_GridMapHeight; i++)
    {
        for (int j = 0; j < m_GridMapWidth; j++)
        {
            int ObstacleGridNum = 0;
            if(pGrid32[i][j].GridType==1)
            {
                int NeiLeft = j - GRID_RANGE > 0 ? (j - GRID_RANGE) : 0;                            //（2*GRID_RANGE+1）x（2*GRID_RANGE+1）网格
                int NeiRight = j + GRID_RANGE < m_GridMapWidth ? (j + GRID_RANGE) : m_GridMapWidth;
                int NeiBottom = i - GRID_RANGE > 0 ? (i - GRID_RANGE) : 0;
                int NeiTop = i + GRID_RANGE < m_GridMapHeight ? (i + GRID_RANGE) : m_GridMapHeight;
                for (int Temp_i = NeiBottom; Temp_i < NeiTop; Temp_i++)
                {
                    for (int Temp_j = NeiLeft; Temp_j < NeiRight; Temp_j++)
                    {
                        if (pGrid32[Temp_i][Temp_j].GridType == 1)
                            ObstacleGridNum++;
                    }
                }
                if (ObstacleGridNum <= 2)         // || !GridLine(pGrid32[i][j])
                {
                    pGrid32[i][j].GridType == 0;
                    m_GridMat.at<uchar>(i, j) = 0;           //将对应的像素点标记为黑色（未知区域）
                }
            }
        }
    }
////////////////////////过滤道路上只因为草的而被检测为正障碍栅格///////////////
    const int OBS_LOCAL_GRID_WIDTH = 5;
    const int MAX_LOCAL_GRID_SIZE = OBS_GRID_WIDTH / OBS_LOCAL_GRID_WIDTH;
    cv::Mat LocalMat = cv::Mat::zeros(MAX_LOCAL_GRID_SIZE, MAX_LOCAL_GRID_SIZE, 0);
    for (int i = 0; i < RoadPosGridInds.size()/2; i++)         //遍历每个路上的正障碍栅格
    {
        int LocalObstacleGridNum = 0;
        int Row = RoadPosGridInds[2 * i];
        int Col = RoadPosGridInds[2 * i+ 1] ;
        for (int j = 0; j < pGrid32[Row][Col].Points.size(); j++)   //遍历每个栅格中点云
        {
            int x = (pGrid32[Row][Col].Points[j].x-RangRect32.x)%OBS_GRID_WIDTH;
            int y = (RangRect32.height + RangRect32.y- pGrid32[Row][Col].Points[j].y)%OBS_GRID_WIDTH;
            int LocalCol = floor(x / OBS_LOCAL_GRID_WIDTH);
            int LocalRow = floor(y / OBS_LOCAL_GRID_WIDTH);
            if (pGrid32[Row][Col].Points[j].z > VEHICLE_RANGE_HEIGHT_THRESH)
            {
                if (LocalMat.at<uchar>(LocalRow, LocalCol) != GRID_POSITIVE)
                {
                    LocalMat.at<uchar>(LocalRow, LocalCol) = GRID_POSITIVE;
                    LocalObstacleGridNum++;
                }
            }
        }
        if (LocalObstacleGridNum < 2)
            m_GridMat.at<uchar>(Row, Col) = 0;
    }

    ///////////////////////膨胀腐蚀操作//////////////////////////////////
    cv::Mat kernel1 = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
    cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,4));
    cv::dilate(m_GridMat,m_GridMat,kernel2);
    cv::erode(m_GridMat, m_GridMat, kernel1);
}




void CLidar32OD::SortLidar32(Lidar32Data* p32)        //提取感兴趣区域的雷达点云
{
    int SortDataEachLineNum ;
    for (int LineID = 0; LineID < 32; LineID++)
    {
        SortDataEachLineNum = 0;
        for (int pID = 0; pID < p32->RealEachLinePointNum[LineID]; pID++)
        {
            if (p32->OneFrameData[LineID][pID].realDistance > 0)
            {
                Lidar32Point pt = p32->OneFrameData[LineID][pID];
                if(pt.x>RangRect32.x && pt.x<(RangRect32.x+ RangRect32.width)  &&pt.y>RangRect32.y && pt.y<(RangRect32.y+ RangRect32.height))
                {
                    pSortData32->OneFrameData[LineID][SortDataEachLineNum] = p32->OneFrameData[LineID][pID];
                    SortDataEachLineNum++;
                }
            }
        }
        pSortData32->RealEachLinePointNum[LineID] = SortDataEachLineNum;
    }
    //重新排序各点
    //获得角度偏移
    int PointTan;
    double k;            //斜率
    for (int LineID = 0; LineID < 32 && pSortData32->RealEachLinePointNum[LineID] !=0; LineID++)
    {
        for (int PointID = 0; PointID < pSortData32->RealEachLinePointNum[LineID]; PointID++)
        {
            k = double(pSortData32->OneFrameData[LineID][PointID].y) / double(pSortData32->OneFrameData[LineID][PointID].x);
            PointTan = atan(k) / OBS_PI * 18000;
            if (PointTan >= 0 && PointTan <= 9000)
            {
                pSortData32->OneFrameData[LineID][PointID].azi = 18000 - PointTan;
            }
            else
            {
                pSortData32->OneFrameData[LineID][PointID].azi = abs(PointTan);
            }
        }
        //按角度偏移量排序点云
        int t;
        Lidar32Point TempLidar32Point;
        for (int PointID = 0; PointID < pSortData32->RealEachLinePointNum[LineID]-1; PointID++)
        {
            int min = PointID;
            for (int j = PointID + 1; j < pSortData32->RealEachLinePointNum[LineID]; j++)
            {
                if (pSortData32->OneFrameData[LineID][min].azi > pSortData32->OneFrameData[LineID][j].azi)
                    min = j;
            }
            if (min != PointID)
            {
                TempLidar32Point = pSortData32->OneFrameData[LineID][PointID];
                pSortData32->OneFrameData[LineID][PointID] = pSortData32->OneFrameData[LineID][min];
                pSortData32->OneFrameData[LineID][min] = TempLidar32Point;

            }
        }
    }
}





void CLidar32OD::GetPointDensity(Lidar32Data *p_l32data, int* DensityArray, int LineID, int LeftInd, int RightInd)
{
    int ValidLeftInd = LeftInd < 1 ? 1 : LeftInd;
    int ValidRightInd = RightInd >(m_PointNumEachLine - 2) ? (m_PointNumEachLine - 2) : RightInd;
    Lidar32Point* ptrLidar32Point = &(p_l32data->OneFrameData[LineID][ValidLeftInd]), *TempLidar32Point = NULL;

    double ScanRadiusDiff = GetScanRadiusDiff(LineID);
    double TempScanRadiusDiff= GetScanRadiusDiff(LineID+1);
    double Lidar32DensityDistThresh = ScanRadiusDiff < TempScanRadiusDiff ? ScanRadiusDiff : TempScanRadiusDiff;
    Lidar32DensityDistThresh *= GAMMA_DENSITY_LIDAR32;
    Lidar32DensityDistThresh = Lidar32DensityDistThresh > 30 ? 30 : Lidar32DensityDistThresh;

    for (int pID = ValidLeftInd; pID <= ValidRightInd; ptrLidar32Point++, pID++)
    {
        int Density = 0, DistOut = 0;
        int CurHAngle = ptrLidar32Point->azi;
        if (CurHAngle == 0)	continue;
        int CurX = ptrLidar32Point->x;
        int CurY = ptrLidar32Point->y;
        int TempDist1 = 0,TempDist2=0;
        //向扫描半径更大的方向看
        int TempLineID = LineID + 1;
        while (TempLineID <= MAX_SORTED_LIDAR32_ID)
        {
            int TempPID = pID;
            int TempValidLeftInd = 0;
            int TempValidRightInd = p_l32data->RealEachLinePointNum[TempLineID];
            TempLidar32Point = &(p_l32data->OneFrameData[TempLineID][pID]);
            while (TempLidar32Point->azi == 0)
            {
                TempLidar32Point++;
                if (TempPID++ > TempValidRightInd) break;
            }

            if (CurHAngle < TempLidar32Point->azi)	//往左找
            {
                while (CurHAngle <(--TempLidar32Point)->azi )
                {
                    if (--TempPID < TempValidLeftInd) break;
                }

                TempDist1 = sqrt((CurX - TempLidar32Point->x)*(CurX - TempLidar32Point->x) + (CurY - TempLidar32Point->y)*(CurY - TempLidar32Point->y));
                TempDist2 = sqrt((CurX - (TempLidar32Point + 1)->x)*(CurX - (TempLidar32Point + 1)->x) + (CurY - (TempLidar32Point + 1)->y)*(CurY - (TempLidar32Point + 1)->y));
                if (TempDist1 < Lidar32DensityDistThresh && TempDist2 < Lidar32DensityDistThresh)   Density+=2;
                else if ((TempDist1 < Lidar32DensityDistThresh && TempDist2 > Lidar32DensityDistThresh) || (TempDist1 > Lidar32DensityDistThresh && TempDist2 < Lidar32DensityDistThresh)) Density++;
                else DistOut = 1;
            }
            else										//往右找
            {
                while (CurHAngle > (++TempLidar32Point)->azi )
                {
                    if (++TempPID > TempValidRightInd) break;
                }


                TempDist1 = sqrt((CurX - TempLidar32Point->x)*(CurX - TempLidar32Point->x) + (CurY - TempLidar32Point->y)*(CurY - TempLidar32Point->y));
                TempDist2 = sqrt((CurX - (TempLidar32Point - 1)->x)*(CurX - (TempLidar32Point - 1)->x) + (CurY - (TempLidar32Point - 1)->y)*(CurY - (TempLidar32Point - 1)->y));
                if (TempDist1 < Lidar32DensityDistThresh && TempDist2 < Lidar32DensityDistThresh)   Density += 2;
                else if ((TempDist1 < Lidar32DensityDistThresh && TempDist2 > Lidar32DensityDistThresh) || (TempDist1 > Lidar32DensityDistThresh && TempDist2 < Lidar32DensityDistThresh)) Density++;
                else DistOut = 1;
            }
            if (DistOut)	break;
            TempLineID++;
        }

        //向扫描半径更小的方向看
#if 0
        DistOut = 0;
		TempLineID = LineID - 1;
		while (TempLineID >= MIN_SORTED_LIDAR32_ID)
		{
			int TempPID = pID;
			int TempValidLeftInd = 0;
			int TempValidRightInd = p_l32data->RealEachLinePointNum[TempLineID];
			TempLidar32Point = &(p_l32data->OneFrameData[TempLineID][pID]);
			while (TempLidar32Point->azi == 0)
			{
				TempLidar32Point++;
				if (TempPID++ > TempValidRightInd) break;
			}

			if (CurHAngle <= TempLidar32Point->azi)	//往左找
			{
				while (CurHAngle <(--TempLidar32Point)->azi)
				{
					if (--TempPID < TempValidLeftInd) break;
		        }

				TempDist1 = sqrt((CurX - TempLidar32Point->x)*(CurX - TempLidar32Point->x) + (CurY - TempLidar32Point->y)*(CurY - TempLidar32Point->y));
				TempDist2 = sqrt((CurX - (TempLidar32Point + 1)->x)*(CurX - (TempLidar32Point + 1)->x) + (CurY - (TempLidar32Point + 1)->y)*(CurY - (TempLidar32Point + 1)->y));
				if (TempDist1 < Lidar32DensityDistThresh && TempDist2 < Lidar32DensityDistThresh)   Density += 2;
				else if ((TempDist1 < Lidar32DensityDistThresh && TempDist2 > Lidar32DensityDistThresh) || (TempDist1 > Lidar32DensityDistThresh && TempDist2 < Lidar32DensityDistThresh)) Density++;
				else DistOut = 1;
			}
			else										//往右找
			{
				while (CurHAngle > (++TempLidar32Point)->azi)
				{
					if (++TempPID > TempValidRightInd) break;
				}
				TempDist1 = sqrt((CurX - TempLidar32Point->x)*(CurX - TempLidar32Point->x) + (CurY - TempLidar32Point->y)*(CurY - TempLidar32Point->y));
				TempDist2 = sqrt((CurX - (TempLidar32Point - 1)->x)*(CurX - (TempLidar32Point - 1)->x) + (CurY - (TempLidar32Point - 1)->y)*(CurY - (TempLidar32Point - 1)->y));
				if (TempDist1 < Lidar32DensityDistThresh && TempDist2 < Lidar32DensityDistThresh)   Density += 2;
				else if ((TempDist1 < Lidar32DensityDistThresh && TempDist2 > Lidar32DensityDistThresh) || (TempDist1 > Lidar32DensityDistThresh && TempDist2 < Lidar32DensityDistThresh)) Density++;
				else DistOut = 1;
}
			if (DistOut)	break;
			TempLineID--;
		}
#endif
        DensityArray[pID] = Density;
    }
}





void CLidar32OD::GetNOConvexPoints(Lidar32Data *p_l32data, int LineID, unsigned int* ConvexPair, int ValidCPNum, int MaxNeighDist, int MinNeighDist)
{
    Lidar32Point *ptrLidar32Point = NULL;
    int ValidLeftInd = 0;
    int ValidRightInd = m_PointNumEachLine;
    int *DensityArray = new int[m_PointNumEachLine];
    memset(DensityArray, 0, sizeof(int)*m_PointNumEachLine);



    for (int i = 0; i < ValidCPNum; i++)
    {

        int AscendInd = ConvexPair[i * 2];    //上升点索引
        int DescendInd = ConvexPair[i * 2 + 1];   //下降点索引

        p_l32data->OneFrameData[LineID][AscendInd].PointType = 1;    // 1上升点,2下降点
        p_l32data->OneFrameData[LineID][DescendInd].PointType = 2;




        int AscendEdgeX = p_l32data->OneFrameData[LineID][AscendInd].x;
        int DescendEdgeX = p_l32data->OneFrameData[LineID][DescendInd].x;
        int NegObsLength =DescendEdgeX - AscendEdgeX;
        int NeighLength = int(NegObsLength*0.66 + 0.5);							//要比较的领域长度
        NeighLength = NeighLength < MinNeighDist ? MinNeighDist : NeighLength;
        NeighLength = NeighLength > MaxNeighDist ? MaxNeighDist : NeighLength;

        //往左找要比较的领域范围
        double AvgLeftNeighDist = 0;
        int LeftNeighNum = 0, LeftInd = AscendInd, TempDist = 0;

        ptrLidar32Point = &(p_l32data->OneFrameData[LineID][LeftInd - 1]);
        while (LeftInd > ValidLeftInd)
        {
            TempDist = AscendEdgeX - ptrLidar32Point->x;
            if (TempDist < NeighLength)
            {
                LeftInd--;
                LeftNeighNum++;
                AvgLeftNeighDist += ptrLidar32Point->y;
            }
            else
            {
                break;
            }

            ptrLidar32Point--;
        }
        if (LeftNeighNum > 0)	AvgLeftNeighDist /= LeftNeighNum;

        //往右找要比较的领域范围
        double AvgRightNeighDist = 0;
        int RightNeighNum = 0, RightInd = DescendInd;
        ptrLidar32Point = &(p_l32data->OneFrameData[LineID][RightInd + 1]);
        while (RightInd < ValidRightInd)
        {
            TempDist = ptrLidar32Point->x - DescendEdgeX;
            if (TempDist < NeighLength)
            {
                RightInd++;
                RightNeighNum++;
                AvgRightNeighDist += ptrLidar32Point->y;
            }
            else
            {
                break;
            }

            //找下一个有效扫描点
            ptrLidar32Point++;
        }
        if (RightNeighNum > 0) AvgRightNeighDist /= RightNeighNum;


        GetPointDensity(p_l32data, DensityArray, LineID, AscendInd,DescendInd );

        int ConvexPointNum = 0;
        double AvgDist = 0, AvgDensity = 0;
        ptrLidar32Point = &(p_l32data->OneFrameData[LineID][AscendInd]);
        for (int pID = AscendInd; pID <= DescendInd; ptrLidar32Point++, pID++)
        {
            AvgDist += ptrLidar32Point->y;
            AvgDensity += DensityArray[pID];
            ConvexPointNum++;
        }
        AvgDist /= ConvexPointNum;
        AvgDensity /= ConvexPointNum;

        if (AvgDensity > DESITY_THRESH  && (AvgDist -AvgLeftNeighDist  ) > NO_COMVEX_THRESH && (AvgDist - AvgRightNeighDist  ) > NO_COMVEX_THRESH  ) //
        {
            ptrLidar32Point = &(p_l32data->OneFrameData[LineID][AscendInd]);
            for (int pID = AscendInd; pID <= DescendInd; ptrLidar32Point++, pID++)
            {
                ptrLidar32Point->PointType = 5;
            }
            m_NOInds.push_back(LineID);
            m_NOInds.push_back(AscendInd);
            m_NOInds.push_back(DescendInd);
        }
        else {
            ptrLidar32Point = &(p_l32data->OneFrameData[LineID][AscendInd]);
            for (int pID = AscendInd; pID <= DescendInd; ptrLidar32Point++, pID++)
            {
                ptrLidar32Point->PointType = 3;
            }
        }

    }

    delete[] DensityArray;
}




unsigned int NOConvexPair[MAX_CONVEX_PAIR][2];
void CLidar32OD::DetectNOLocalConvexity(Lidar32Data *p_l32data, int NeighborNum)            //寻找上升沿、下降沿，将其索引存入NOConvexPair
{
    int ValidLeftInd;
    int	ValidRightInd;
    Lidar32Point* ptrLidar32Point = NULL, *TempLidar32Point = NULL;

    int ConvexInd = 0, UpOrDown = 0, UpperStartX = 0;		//UpOrDown: 0表示当前需要检测上升沿，1检测下降沿

    m_NOInds.clear();             //清空上一帧的负障碍索引
    for (int LineID = MIN_SORTED_LIDAR32_ID; LineID < MAX_SORTED_LIDAR32_ID; LineID++)
    {

        memset(NOConvexPair, 0, sizeof(NOConvexPair));
        m_PointNumEachLine = p_l32data->RealEachLinePointNum[LineID];
        int *DistOff = new int[m_PointNumEachLine];
        memset(DistOff, 0, sizeof(int)*m_PointNumEachLine);

        ValidLeftInd = 0;
        ValidRightInd = ValidLeftInd + m_PointNumEachLine;

        ptrLidar32Point = &(p_l32data->OneFrameData[LineID][ValidLeftInd]);
        TempLidar32Point = ptrLidar32Point + 1;


        int pID = ValidLeftInd;
        for (; pID < ValidRightInd; ptrLidar32Point++, TempLidar32Point++, pID++)
        {
            DistOff[pID] = TempLidar32Point->y - ptrLidar32Point->y;

        }

        UpOrDown = 0;
        ConvexInd = 0;
        UpperStartX = RangRect32.x;
        pID = ValidLeftInd;
        //检测上升沿与下降沿，并将索引存在NOConvexPair[MAX_CONVEX_PAIR][2]中
        while (pID < ValidRightInd)
        {
            /////////////////////////////////////检测上升沿/////////////////////////////////////////////////
            int WindowSize = 1;
            int DistOffInvertNum = 0;
            int TotalDistOff = DistOff[pID];
            while (pID < ValidRightInd)
            {
                if (TotalDistOff > COMVEX_DISTOFF_THRESH)					//上升沿
                {
                    UpOrDown = 1;
                    NOConvexPair[ConvexInd][0] = pID;
                    pID++;
                    UpperStartX = p_l32data->OneFrameData[LineID][pID].x;
                    break;
                }
                else if (TotalDistOff < NEG_COMVEX_DISTOFF_THRESH)			//下降沿
                {
                    if (UpOrDown == 1)										//和前面上升沿组成一对
                    {
                        int dist = p_l32data->OneFrameData[LineID][pID].x - UpperStartX;
                        if (dist > NEG_OBS_MIN_LENGTH && dist < NEG_OBS_MAX_LENGTH)
                        {
                            UpOrDown = 0;									//下次默认检测上升沿
                            NOConvexPair[ConvexInd][1] = pID;                //下降沿点号
                            ConvexInd++;
                        }
                    }
                    else													//当出现多个上升沿，更新当前组的上升沿
                    {
                        if ((p_l32data->OneFrameData[LineID][pID].x - UpperStartX)< NEG_OBS_MAX_LENGTH)	//只有之前检测到上升沿，才更新其下降沿
                            NOConvexPair[ConvexInd - 1][1] = pID;
                    }

                    break;
                }

                if (TotalDistOff > 0)			//上升方向
                {
                    if (DistOff[++pID] >= 0)
                    {
                        DistOffInvertNum = 0;
                        TotalDistOff += DistOff[pID];
                    }
                    else
                    {
                        if (DistOff[pID] > NEG_COMVEX_DISTOFF_THRESH)
                        {
                            pID--;
                            break;
                        }
                        else if ((++DistOffInvertNum) >= 2)		//检测上升沿时连续出现两次下降沿，从新开始
                        {
                            pID -= 2;
                            break;
                        }
                        else
                        {
                            TotalDistOff += DistOff[pID];
                        }
                    }

                }
                else							//下降方向
                {
                    if (DistOff[++pID] <= 0)
                    {
                        DistOffInvertNum = 0;
                        TotalDistOff += DistOff[pID];
                    }
                    else
                    {
                        if (DistOff[pID] > COMVEX_DISTOFF_THRESH)
                        {
                            pID--;
                            break;
                        }
                        else if ((++DistOffInvertNum) >= 2)		//检测下降沿时连续出现两次上升沿，从新开始
                        {
                            pID -= 2;
                            break;
                        }
                        else
                        {
                            TotalDistOff += DistOff[pID];
                        }
                    }
                }

                if (WindowSize >= NeighborNum)
                    TotalDistOff -= DistOff[pID - NeighborNum];
                else
                    WindowSize++;
            }
            pID++;
        }
        //对每一组上升沿和下降沿之间的点，进行进一步分析
        if (ConvexInd > 0)
            GetNOConvexPoints(p_l32data, LineID, &(NOConvexPair[0][0]), ConvexInd);
        delete[] DistOff;
    }
}



/**
void CLidar32OD::FindMatchPoints(Lidar32Data *p_l32data)
{
    m_NegObsFeaturePairs.clear();
    NegObsDetectedFeature TempNOP;
    Lidar32Point*  ptrLidar32PointA = NULL, *ptrLidar32PointB = NULL, *ptrLidar32PointC = NULL;
    for (std::vector<int>::iterator it = m_NOInds.begin(); it != m_NOInds.end(); )
    {
        int LineID = *it++;
        int LeftStartInd = *it++;
        int RightEndInd = *it++;

        ptrLidar32PointB = &(p_l32data->OneFrameData[LineID][LeftStartInd]);
        for (int pID = LeftStartInd; pID <= RightEndInd; ptrLidar32PointB++, pID++)
        {
            if (ptrLidar32PointB->PointType != 5) continue;
            ptrLidar32PointA = NULL;		//半径更小的对应点
            ptrLidar32PointC = NULL;		//半径更大的对应点


            int CurY = ptrLidar32PointB->y;
            int CurHAngle = ptrLidar32PointB->azi;
            //寻找匹配的点A
            int TempLineID = LineID - 1;
            while (TempLineID >= MIN_SORTED_LIDAR32_ID)
            {
                int TempPID = pID;
                int TempValidLeftInd = 0;
                int TempValidRightInd = p_l32data->RealEachLinePointNum[TempLineID];
                Lidar32Point* TempLidar32Point = &(p_l32data->OneFrameData[TempLineID][pID]);

                if (CurHAngle < TempLidar32Point->azi)	//往左找
                {
                    while (CurHAngle < (--TempLidar32Point)->azi)
                    {
                        if (--TempPID < TempValidLeftInd) break;
                    }
                    if (abs(CurHAngle - TempLidar32Point->azi) > abs(CurHAngle - (TempLidar32Point + 1)->azi))
                        TempLidar32Point++;

                }
                else										//往右找
                {
                    while (CurHAngle > (++TempLidar32Point)->azi)
                    {
                        if (++TempPID > TempValidRightInd) break;
                    }

                    if (abs(CurHAngle - TempLidar32Point->azi) > abs(CurHAngle - (TempLidar32Point - 1)->azi))
                        TempLidar32Point--;
                }

                if (abs(CurHAngle - TempLidar32Point->azi) <= 9)
                {
                    if (TempLidar32Point->PointType == 5) break;
                    else
                    {
                        int YDiff = CurY - TempLidar32Point->y;
                        if (((YDiff > NEG_OBS_MIN_WIDTH & TempLidar32Point->PointType == 0) \
							|| YDiff > NEG_OBS_MIN_WIDTH_BULGE) && YDiff < NEG_OBS_MAX_WIDTH)
                        {
                            ptrLidar32PointA = TempLidar32Point;
                            break;
                        }
                    }
                }
                TempLineID--;
            }
            if (NULL == ptrLidar32PointA)	continue;

            //寻找匹配的点C
            TempLineID = LineID + 1;
            while (TempLineID <= MAX_SORTED_LIDAR32_ID)
            {
                int TempPID = pID;
                int TempValidLeftInd = 0;
                int TempValidRightInd = p_l32data->RealEachLinePointNum[TempLineID];
                Lidar32Point* TempLidar32Point = &(p_l32data->OneFrameData[TempLineID][pID]);

                if (CurHAngle < TempLidar32Point->azi)	//往左找
                {
                    while (CurHAngle < (--TempLidar32Point)->azi)
                    {
                        if (--TempPID < TempValidLeftInd) break;
                    }

                    if (abs(CurHAngle - TempLidar32Point->azi) > abs(CurHAngle - (TempLidar32Point + 1)->azi))
                        TempLidar32Point++;

                }
                else										//往右找
                {
                    while (CurHAngle > (++TempLidar32Point)->azi)
                    {
                        if (++TempPID > TempValidRightInd) break;
                    }

                    if (abs(CurHAngle - TempLidar32Point->azi) > abs(CurHAngle - (TempLidar32Point - 1)->azi))
                        TempLidar32Point--;
                }

                if (abs(CurHAngle - TempLidar32Point->azi) <= 9)
                {
                    ptrLidar32PointC = TempLidar32Point;
                    break;

                }
                TempLineID++;
            }

            //根据几何该点对是否满足凹障碍条件

            if (NULL != ptrLidar32PointB && NULL != ptrLidar32PointC && ptrLidar32PointA->y!=0)
            {
                int DistAB = sqrt((ptrLidar32PointA->x - ptrLidar32PointB->x)*(ptrLidar32PointA->x - ptrLidar32PointB->x) \
					+ (ptrLidar32PointA->y - ptrLidar32PointB->y)*(ptrLidar32PointA->y - ptrLidar32PointB->y));
                int DistBC = sqrt((ptrLidar32PointB->x - ptrLidar32PointC->x)*(ptrLidar32PointB->x - ptrLidar32PointC->x) \
					+ (ptrLidar32PointB->y - ptrLidar32PointC->y)*(ptrLidar32PointB->y - ptrLidar32PointC->y));
                int DeltaAB = ptrLidar32PointA->z - ptrLidar32PointB->z;
                int HeightDiffThresh = GAMMA_TWO*(DistAB*LIDAR_HEIGHT / ptrLidar32PointA->y);
                if (DistAB > 30 && DistAB > GAMMA_ONE*DistBC && DeltaAB > HeightDiffThresh)
                {
                    m_NegObsFeaturePairs.push_back(*ptrLidar32PointC);
                    m_NegObsFeaturePairs.push_back(*ptrLidar32PointB);
                    m_NegObsFeaturePairs.push_back(*ptrLidar32PointA);
                }
                else
                {
                    ptrLidar32PointA->PointType = 0;
                    ptrLidar32PointB->PointType = 0;
                    ptrLidar32PointC->PointType = 0;
                }
            }
        }
    }
    int StartRow, StartCol, EndRow, EndCol,Temp;
    int k, MinRow,MinCol,MaxCol,TempRow,TempCol;
    for (int i = 0; i < (m_NegObsFeaturePairs.size()/3); i++)
    {
        if (m_NegObsFeaturePairs.size() / 3 > 15)
        {
            StartRow = (RangRect32.height - m_NegObsFeaturePairs[3 * i + 1].y) / OBS_GRID_WIDTH;        //提取出B、A点的mat坐标
            StartCol = (m_NegObsFeaturePairs[3 * i + 1].x - RangRect32.x) / OBS_GRID_WIDTH;
            EndRow = (RangRect32.height - m_NegObsFeaturePairs[3 * i + 2].y) / OBS_GRID_WIDTH;
            EndCol = (m_NegObsFeaturePairs[3 * i + 2].x - RangRect32.x) / OBS_GRID_WIDTH;

            MinRow = StartRow < EndRow ? StartRow:EndRow;                         //将(StartRow,StartCol),(EndRow,EndCol)两点连起来的直线，所在的栅格赋负障碍
            MinCol = StartCol < EndCol ? StartCol : EndCol;
            MaxCol = StartCol > EndCol ? StartCol : EndCol;
            if ((StartRow - EndRow) != 0)
            {
                k = (StartCol - EndCol) / (StartRow - EndRow);            //直线斜率
                for (TempRow = StartRow; TempRow <= EndRow; TempRow++)
                {
                    TempCol = k*(TempRow - StartRow) + StartCol;
                    m_GridMat.at<uchar>(TempRow, TempCol) = GRID_NEGATIVE;
                }
            }
            else                               //斜率不存在
            {
                for (TempCol = MinCol; TempCol <= MaxCol; TempCol++)
                {
                    m_GridMat.at<uchar>(StartRow, TempCol) = GRID_NEGATIVE;
                }
            }
        }
    }
}
**/

void CLidar32OD::FindMatchPoints(Lidar32Data *p_l32data)
{
    m_NegObsFeaturePairs.clear();
    NegObsDetectedFeature TempNOP;
    Lidar32Point*  ptrLidar32PointA = NULL, *ptrLidar32PointB = NULL, *ptrLidar32PointC = NULL;
    for (std::vector<int>::iterator it = m_NOInds.begin(); it != m_NOInds.end(); )
    {
        int LineID = *it++;
        int LeftStartInd = *it++;
        int RightEndInd = *it++;

        ptrLidar32PointB = &(p_l32data->OneFrameData[LineID][LeftStartInd]);
        for (int pID = LeftStartInd; pID <= RightEndInd; ptrLidar32PointB++, pID++)
        {
            if (ptrLidar32PointB->PointType != 5) continue;
            ptrLidar32PointA = NULL;		//半径更小的对应点
            ptrLidar32PointC = NULL;		//半径更大的对应点


            int CurY = ptrLidar32PointB->y;
            int CurHAngle = ptrLidar32PointB->azi;
            //寻找匹配的点A
            int TempLineID = LineID - 1;
            while (TempLineID >= MIN_SORTED_LIDAR32_ID)
            {
                int TempPID = pID;
                int TempValidLeftInd = 0;
                int TempValidRightInd = p_l32data->RealEachLinePointNum[TempLineID];
                Lidar32Point* TempLidar32Point = &(p_l32data->OneFrameData[TempLineID][pID]);

                if (CurHAngle < TempLidar32Point->azi)	//往左找
                {
                    while (CurHAngle < (--TempLidar32Point)->azi)
                    {
                        if (--TempPID < TempValidLeftInd) break;
                    }
                    if (abs(CurHAngle - TempLidar32Point->azi) > abs(CurHAngle - (TempLidar32Point + 1)->azi))
                        TempLidar32Point++;

                }
                else										//往右找
                {
                    while (CurHAngle > (++TempLidar32Point)->azi)
                    {
                        if (++TempPID > TempValidRightInd) break;
                    }

                    if (abs(CurHAngle - TempLidar32Point->azi) > abs(CurHAngle - (TempLidar32Point - 1)->azi))
                        TempLidar32Point--;
                }

                if (abs(CurHAngle - TempLidar32Point->azi) <= 9)
                {
                    if (TempLidar32Point->PointType == 5) break;
                    else
                    {
                        int YDiff = CurY - TempLidar32Point->y;
                        if (((YDiff > NEG_OBS_MIN_WIDTH & TempLidar32Point->PointType == 0) \
							|| YDiff > NEG_OBS_MIN_WIDTH_BULGE) && YDiff < NEG_OBS_MAX_WIDTH)
                        {
                            ptrLidar32PointA = TempLidar32Point;
                            break;
                        }
                    }
                }
                TempLineID--;
            }
            if (NULL == ptrLidar32PointA)	continue;

            //寻找匹配的点C
            TempLineID = LineID + 1;
            while (TempLineID <= MAX_SORTED_LIDAR32_ID)
            {
                int TempPID = pID;
                int TempValidLeftInd = 0;
                int TempValidRightInd = p_l32data->RealEachLinePointNum[TempLineID];
                Lidar32Point* TempLidar32Point = &(p_l32data->OneFrameData[TempLineID][pID]);

                if (CurHAngle < TempLidar32Point->azi)	//往左找
                {
                    while (CurHAngle < (--TempLidar32Point)->azi)
                    {
                        if (--TempPID < TempValidLeftInd) break;
                    }

                    if (abs(CurHAngle - TempLidar32Point->azi) > abs(CurHAngle - (TempLidar32Point + 1)->azi))
                        TempLidar32Point++;

                }
                else										//往右找
                {
                    while (CurHAngle > (++TempLidar32Point)->azi)
                    {
                        if (++TempPID > TempValidRightInd) break;
                    }

                    if (abs(CurHAngle - TempLidar32Point->azi) > abs(CurHAngle - (TempLidar32Point - 1)->azi))
                        TempLidar32Point--;
                }

                if (abs(CurHAngle - TempLidar32Point->azi) <= 9)
                {
                    ptrLidar32PointC = TempLidar32Point;
                    break;

                }
                TempLineID++;
            }

            //根据几何该点对是否满足凹障碍条件

            if (NULL != ptrLidar32PointB && NULL != ptrLidar32PointC && ptrLidar32PointA->y!=0)
            {
                int DistAB = sqrt((ptrLidar32PointA->x - ptrLidar32PointB->x)*(ptrLidar32PointA->x - ptrLidar32PointB->x) \
					+ (ptrLidar32PointA->y - ptrLidar32PointB->y)*(ptrLidar32PointA->y - ptrLidar32PointB->y));
                int DistBC = sqrt((ptrLidar32PointB->x - ptrLidar32PointC->x)*(ptrLidar32PointB->x - ptrLidar32PointC->x) \
					+ (ptrLidar32PointB->y - ptrLidar32PointC->y)*(ptrLidar32PointB->y - ptrLidar32PointC->y));
                int DeltaAB = ptrLidar32PointA->z - ptrLidar32PointB->z;
                int HeightDiffThresh = GAMMA_TWO*(DistAB*LIDAR_HEIGHT / ptrLidar32PointA->y);
                if (DistAB > 30 && DistAB > GAMMA_ONE*DistBC && DeltaAB > HeightDiffThresh)
                {
                    m_NegObsFeaturePairs.push_back(*ptrLidar32PointC);
                    m_NegObsFeaturePairs.push_back(*ptrLidar32PointB);
                    m_NegObsFeaturePairs.push_back(*ptrLidar32PointA);

                }
                else
                {
                    ptrLidar32PointA->PointType = 0;
                    ptrLidar32PointB->PointType = 0;
                    ptrLidar32PointC->PointType = 0;
                }
            }
        }
    }
    int StartRow, StartCol, EndRow, EndCol,Temp;
    int k, MinRow,MinCol,MaxCol,TempRow,TempCol;
    for (int i = 0; i < (m_NegObsFeaturePairs.size()/3); i++)
    {
        if (m_NegObsFeaturePairs.size() / 3 > 15 )
        {
            StartRow = (RangRect32.height - m_NegObsFeaturePairs[3 * i + 1].y) / OBS_GRID_WIDTH;        //提取出B、A点的mat坐标
            StartCol = (m_NegObsFeaturePairs[3 * i + 1].x - RangRect32.x) / OBS_GRID_WIDTH;
            EndRow = (RangRect32.height - m_NegObsFeaturePairs[3 * i + 2].y) / OBS_GRID_WIDTH;
            EndCol = (m_NegObsFeaturePairs[3 * i + 2].x - RangRect32.x) / OBS_GRID_WIDTH;
            if (abs(m_NegObsFeaturePairs[3 * i + 1].y - m_NegObsFeaturePairs[3 * i + 2].y) > NEG_OBS_MIN_WIDTH_BULGE && abs(m_NegObsFeaturePairs[3 * i + 1].y - m_NegObsFeaturePairs[3 * i + 2].y) < 120)
            {
                MinRow = StartRow < EndRow ? StartRow : EndRow;                         //将(StartRow,StartCol),(EndRow,EndCol)两点连起来的直线，所在的栅格赋负障碍
                MinCol = StartCol < EndCol ? StartCol : EndCol;
                MaxCol = StartCol > EndCol ? StartCol : EndCol;
                if ((StartRow - EndRow) != 0)
                {
                    k = (StartCol - EndCol) / (StartRow - EndRow);            //直线斜率
                    for (TempRow = StartRow; TempRow <= EndRow; TempRow++)
                    {
                        TempCol = k*(TempRow - StartRow) + StartCol;
                        if (m_GridMat.at<uchar>(TempRow, TempCol) != GRID_POSITIVE)
                        {
                            m_GridMat.at<uchar>(TempRow, TempCol) = GRID_NEGATIVE;
                        }

                    }
                }
                else
                {
                    for (TempCol = MinCol; TempCol <= MaxCol; TempCol++)
                    {
                        if (m_GridMat.at<uchar>(StartRow, TempCol) != GRID_POSITIVE)
                        {
                            m_GridMat.at<uchar>(StartRow, TempCol) = GRID_NEGATIVE;
                        }
                    }
                }
            }
        }
    }

}



void CLidar32OD::PODFromLidar32()
{


    InitGridMap();
    SortLidar32(pData32);

/////////////正障碍，栅格地图的方法///////////////
    GridMap(pSortData32);
    GridProce(pGridData32);

/////////////负障碍，上升沿下降沿的方法//////////


#ifdef USE_NEGATIVE
    DetectNOLocalConvexity(pSortData32, 6);
    FindMatchPoints(pSortData32);
#endif
}

int CLidar32OD::InputData(int64_t stamp, Lidar32Data *p32)
{
	timestamp=stamp;
	memcpy(pData32,p32,sizeof(Lidar32Data));
	return 0;
}