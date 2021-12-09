//
// Created by yangfei on 18-8-4.
//

#include "ros_njust_pca/NJUST_Serialize.h"
/// 发送数据
bool gFlag=false;
int gBytes;
char gBuffer[MAX_IP_LENGTH];
pthread_mutex_t g_mutex_buffer = PTHREAD_MUTEX_INITIALIZER;


std::vector<int64_t > gVecLocationTask;
std::vector<int64_t > gVecLocalMapTask;


extern void SwapEndian16(char* p);
extern void SwapEndian32(char* p);
extern void SwapEndian64(char* p);

/// 载入任务文件
void LoadTaskFile(const char *filename)
{
    FILE *fp=fopen(filename,"rt");
    if(!fp)
    {
        printf("can not open file %s !\n",filename);
        exit(-1);
    }
    int n;
    fscanf(fp,"%d",&n);
    gVecLocationTask.reserve(n);
    for(int i=0;i<n;i++)
    {
        int64_t stamp;
        fscanf(fp,"%ld",&stamp);
        gVecLocationTask.push_back(stamp);
    }
    fscanf(fp,"%d",&n);
    gVecLocalMapTask.reserve(n);
    for(int i=0;i<n;i++)
    {
        int64_t stamp;
        fscanf(fp,"%ld",&stamp);
        gVecLocalMapTask.push_back(stamp);
    }
    fclose(fp);
    printf("load task file succeed!\n");
}


/// 寻找任务文件时间戳，以确定是否发送答案数据
int64_t FindTheNearestTaskStamp(int64_t stamp,std::vector<int64_t> &vec, int thre)
{
    int idx=0;
    // 时间的单位是毫秒
    int64_t timeDiff = stamp-vec[idx];
    while (idx < vec.size() - 1 && timeDiff > 0)
    {
        idx++;
        timeDiff = int64_t(stamp - vec[idx]);
    }
    if(idx==0||timeDiff>0)
    {
        if(abs(timeDiff)<thre)
            return vec[idx];
        else
            return -1;
    }
    else
    {
        int nearestIdx=(stamp-vec[idx-1])<abs(timeDiff)?idx-1:idx;
        if(abs(stamp-vec[nearestIdx])<thre)
            return  vec[nearestIdx];
        else
            return -2;
    }
}



/// 寻找任务文件时间戳，以确定是否发送答案数据
int64_t FindTheNearestTaskStamp2(int64_t stamp,std::vector<int64_t> &vec)
{
    if(vec.empty())
        return -1;
    int64_t timeDiff=stamp-*(vec.begin());
    if(timeDiff<0)
        return -1;
    else
    {
        int64_t ret=*(vec.begin());
        vec.erase(vec.begin());
        return ret;
    }
}



/// 目标检测结果回调处理函数
void ObjectResCallback(const ros_njust_pca::NJUST_Answer_st_Object::ConstPtr &msg)
{
    printf("send object answer, stamp is %ld\n",msg->timestamp);
    /// 需要进行大小端转换的元素
    int64_t timestamp;
    double box[8],longitude,latitude;
    int16_t image_cols,image_rows;

    timestamp=msg->timestamp;
    memcpy(box,&msg->box[0],8* sizeof(double));
    longitude=msg->longitude;
    latitude=msg->latitude;
    image_cols=msg->image_cols;
    image_rows=msg->image_rows;

#ifdef SEND_BIG_ENDIAN
//    timestamp=htobe64(timestamp);
    SwapEndian64((char*)&timestamp);
    for(int i=0;i<8;i++)
//        box[i]=htobe64(box[i])
        SwapEndian64((char*)(box+i));
//    longitude=htobe64(longitude);
    SwapEndian64((char*)&longitude);
//    latitude=htobe64(latitude);
    SwapEndian64((char*)&latitude);

//    image_cols=htobe16(image_cols);
    SwapEndian16((char*)&image_cols);

//    image_rows=htobe16(image_rows);
    SwapEndian16((char*)&image_rows);

#endif
    int nBytes=0;
    char buffer[MAX_IP_LENGTH]={0};

    *(buffer+nBytes)=0x02;      // 开始标志
    nBytes+=5;      // 先跳过最后计算数据长度,4字节

    *(buffer+nBytes)=0x4F;       // 数据类型
    nBytes++;

    memcpy(buffer+nBytes,&timestamp,8);    // 时间戳
    nBytes+=8;

    memcpy(buffer+nBytes,&msg->box_type,1);     // 包围框类型
    nBytes++;

    memcpy(buffer+nBytes,box,8*sizeof(double));   // 包围框坐标
    nBytes+=8*sizeof(double);

    memcpy(buffer+nBytes,&msg->attribute,1);   // 目标属性
    nBytes++;

    memcpy(buffer+nBytes,&longitude,sizeof(double));   // 目标经度
    nBytes+=sizeof(double);

    memcpy(buffer+nBytes,&latitude,sizeof(double));    // 目标维度
    nBytes+=sizeof(double);

    memcpy(buffer+nBytes,&image_cols,2);   // 图像宽
    nBytes+=2;

    memcpy(buffer+nBytes,&image_rows,2);   // 图像高
    nBytes+=2;

    if(msg->image_rows!=0&&msg->image_cols!=0)
    {
        sensor_msgs::Image img=msg->image_data;
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr=cv_bridge::toCvCopy(msg->image_data,sensor_msgs::image_encodings::BGR8);
        memcpy(buffer+nBytes,cv_ptr->image.data,msg->image_rows*msg->image_cols*3);     // 图像数据
        nBytes+=msg->image_rows*msg->image_cols*3;
    }

    *(buffer+nBytes)=0x03;      // 结束位
    nBytes++;

    // 计算数据长度
    int length=nBytes-6;
#ifdef SEND_BIG_ENDIAN
//    length=htobe32(length);
    SwapEndian32((char*)&length);
#endif
    *((int *)(buffer+1))=length;   // 计算长度,nBytes-2×1-4

    // 全局变量填充，并设置标志位
    pthread_mutex_lock(&g_mutex_buffer);
    gBytes=nBytes;
    memcpy(gBuffer,buffer,MAX_IP_LENGTH);
    gFlag=true;
    pthread_mutex_unlock(&g_mutex_buffer);
}

/// 定位结果回调处理函数
void LocationResCallback(const ros_njust_pca::NJUST_Answer_st_Location::ConstPtr &msg)
{
    int64_t timestamp;
    double longitude,latitude,heading;
#ifdef SEND_AT_TASK_STAMP
//    timestamp=FindTheNearestTaskStamp(msg->timestamp,gVecLocationTask,50);
    timestamp=FindTheNearestTaskStamp2(msg->timestamp,gVecLocationTask);
    if(timestamp<0)
        return;
    printf("send location answer, stamp is %ld, real stamp is %ld\n",timestamp,msg->timestamp);
#else
    timestamp=msg->timestamp;
#endif
    longitude=msg->longitude;
    latitude=msg->latitude;
    heading=msg->heading;       // don't to rad
#ifdef SEND_BIG_ENDIAN
//    timestamp=htobe64(timestamp);
    SwapEndian64((char*)&timestamp);
//    longitude=htobe64(longitude);
    SwapEndian64((char*)&longitude);
//    latitude=htobe64(latitude);
    SwapEndian64((char*)&latitude);
//    heading=htobe64(heading);
    SwapEndian64((char*)&heading);

#endif

    int nBytes=0;
    char buffer[MAX_IP_LENGTH]={0};

    *(buffer+nBytes)=0x02;   // 开始标志
    nBytes+=5;      // 先跳过最后计算数据长度

    *(buffer+nBytes)=0x4C;       // 数据类型
    nBytes++;

    memcpy(buffer+nBytes,&timestamp,8);    //  时间戳
    nBytes+=8;

    memcpy(buffer+nBytes,&longitude,8);    // 经度
    nBytes+=8;

    memcpy(buffer+nBytes,&latitude,8);     // 纬度
    nBytes+=8;

    memcpy(buffer+nBytes,&heading,8);      // 朝向角
    nBytes+=8;

    *(buffer+nBytes)=0x03;   // 结束标志
    nBytes++;

    // 计算数据长度
    int length=nBytes-6;
#ifdef SEND_BIG_ENDIAN
//    length=htobe64(length);
    SwapEndian32((char*)&length);
#endif
    *((int *)(buffer+1))=length;
    // 全局变量填充，并设置标志位
    pthread_mutex_lock(&g_mutex_buffer);
    gBytes=nBytes;
    memcpy(gBuffer,buffer,MAX_IP_LENGTH);
    gFlag=true;
    pthread_mutex_unlock(&g_mutex_buffer);
}


/// 局部地图回调处理函数
void LocalMapResCallback(const ros_njust_pca::NJUST_Answer_st_Map::ConstPtr &msg)
{
    int64_t timestamp;
#ifdef SEND_AT_TASK_STAMP
//    timestamp=FindTheNearestTaskStamp(msg->timestamp,gVecLocalMapTask,50);
    timestamp=FindTheNearestTaskStamp2(msg->timestamp,gVecLocalMapTask);
    if(timestamp<0)
        return;
    printf("send local map answer, stamp is %ld, real stamp is %ld\n",timestamp,msg->timestamp);
#else
    timestamp=msg->timestamp;
#endif

#ifdef SEND_BIG_ENDIAN
//    timestamp=htobe64(timestamp);
    SwapEndian64((char*)&timestamp);
#endif

    int nBytes=0;
    char buffer[MAX_IP_LENGTH]={0};
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr=cv_bridge::toCvCopy(msg->map,sensor_msgs::image_encodings::MONO8);

    *(buffer+nBytes)=0x02;   // 开始标志
    nBytes+=5;      // 先跳过最后计算数据长度

    *(buffer+nBytes)=0x4D;       // 数据类型
    nBytes++;

    memcpy(buffer+nBytes,&timestamp,8);    // 时间戳
    nBytes+=8;

    memcpy(buffer+nBytes,cv_ptr->image.data,81*81);     // 地图数据
    nBytes+=81*81;

    *(buffer+nBytes)=0x03;   // 结束标志
    nBytes++;

    // 计算数据长度
    int length=nBytes-6;
#ifdef SEND_BIG_ENDIAN
//    length=htobe64(length);
    SwapEndian32((char*)&length);
#endif
    *((int *)(buffer+1))=length;

    // 全局变量填充，并设置标志位
    pthread_mutex_lock(&g_mutex_buffer);
    gBytes=nBytes;
    memcpy(gBuffer,buffer,MAX_IP_LENGTH);
    gFlag=true;
    pthread_mutex_unlock(&g_mutex_buffer);
}




//int EncodeObjectDetectionResult(OBJECT_RES *pRes, char *pBuff)
//{
//    int nBytes=0;
//    memset(pBuff,0,MAX_IP_LENGTH);     // 初始化
//
//    *(pBuff+nBytes)=0x02;   // 开始标志
//    nBytes+=3;      // 先跳过最后计算数据长度
//
//    *(pBuff+nBytes)=0x4F;       // 数据类型
//    nBytes++;
//
//    memcpy(pBuff+nBytes,pRes,sizeof(OBJECT_RES)-1);        //减去最后一个字节的指针，注意此做法需要结构体严格1字节对齐
//    nBytes+=sizeof(OBJECT_RES)-1;
//
//    memcpy(pBuff+nBytes,pRes->image_data,pRes->image_rows*pRes->image_cols*3);
//    nBytes+=pRes->image_rows*pRes->image_cols*3;
//
//    *(pBuff+nBytes)=0x03;   // 结束标志
//    nBytes++;
//
//    int16_t length=nBytes-4;
//    memcpy(pBuff+1,&length,2);
//
//    return nBytes;
//}
//
//
//int EncodeLocationResut(LOCATION_RES *pRes, char *pBuff)
//{
//    int nBytes=0;
//    memset(pBuff,0,MAX_IP_LENGTH);     // 初始化
//
//    *(pBuff+nBytes)=0x02;   // 开始标志
//    nBytes+=3;      // 先跳过最后计算数据长度
//
//    *(pBuff+nBytes)=0x4C;       // 数据类型
//    nBytes++;
//
//    memcpy(pBuff+nBytes,pRes,sizeof(LOCATION_RES));
//    nBytes+=sizeof(LOCATION_RES);
//
//    *(pBuff+nBytes)=0x03;   // 结束标志
//    nBytes++;
//
//    int16_t length=nBytes-4;
//    memcpy(pBuff+1,&length,2);
//
//    return nBytes;
//}


//int EncodeLocalMapResult(LOCAL_MAP_RES *pRes, char *pBuff)
//{
//    int nBytes=0;
//    memset(pBuff,0,MAX_IP_LENGTH);     // 初始化
//
//    *(pBuff+nBytes)=0x02;   // 开始标志
//    nBytes+=3;      // 先跳过最后计算数据长度
//
//    *(pBuff+nBytes)=0x4D;       // 数据类型
//    nBytes++;
//
//    memcpy(pBuff+nBytes,pRes,sizeof(LOCAL_MAP_RES));
//    nBytes+=sizeof(LOCAL_MAP_RES);
//
//    *(pBuff+nBytes)=0x03;   // 结束标志
//    nBytes++;
//
//    int16_t length=nBytes-4;
//    memcpy(pBuff+1,&length,2);
//
//    return nBytes;
//}