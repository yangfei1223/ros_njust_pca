#include "ros_njust_pca/NJUST_Global.h"
#include "ros_njust_pca/NJUST_Data.h"
#include "ros_njust_pca/NJUST_Tcp.h"
#include "ros_njust_pca/NJUST_Serialize.h"

/// 定义数据预处理类
Camera *gCamera;
VelodyneHDL32 *gVelo;
GPS *gGPS;
IMU *gIMU;
#if 0
URG *gURG;
Gear *gGear;
Odom *gOdom;
Steer *gSteer;
#endif

/// 全局指令
char gCmd;
pthread_mutex_t g_mutex_cmd = PTHREAD_MUTEX_INITIALIZER;
char Command()
{
    char ret;
    pthread_mutex_lock(&g_mutex_cmd);
    ret=gCmd;
    pthread_mutex_unlock(&g_mutex_cmd);
    return ret;
}

void SetCommand(char cmd)
{
    pthread_mutex_lock(&g_mutex_cmd);
    gCmd=cmd;
    pthread_mutex_unlock(&g_mutex_cmd);
}


void SwapEndian16(char* p)
{
    char tmp[2];
    memcpy(tmp,p,2);
    p[0]=tmp[1];
    p[1]=tmp[0];
}
void SwapEndian32(char* p)
{
    char tmp[4];
    memcpy(tmp,p,4);
    p[0]=tmp[3];
    p[1]=tmp[2];
    p[2]=tmp[1];
    p[3]=tmp[0];
}
void SwapEndian64(char* p)
{
    char tmp[8];
    memcpy(tmp,p,8);
    p[0]=tmp[7];
    p[1]=tmp[6];
    p[2]=tmp[5];
    p[3]=tmp[4];
    p[4]=tmp[3];
    p[5]=tmp[2];
    p[6]=tmp[1];
    p[7]=tmp[0];
}


bool GetInitIMUState(ros_njust_pca::NJUST_InitState::Request &req,ros_njust_pca::NJUST_InitState::Response &res)
{
    bool ret=false;
    ret=gIMU->GetInitSate(res.heading,res.pitch,res.roll);
    ret=gGPS->GetInitState(res.longitude,res.latitude,res.altitude);
//    ROS_INFO("sending back response: heading=%lf,pitch=%lf,roll=%lf,longitude=%lf,latitude=%lf,altitude=%lf",
//            res.heading,res.pitch,res.roll,res.longitude,res.latitude,res.altitude);
    return ret;
}

bool GetCommand(ros_njust_pca::NJUST_Command::Request &req,ros_njust_pca::NJUST_Command::Response &res)
{
    res.cmd=Command();
    return true;
}



///  初始化函数
void Initialize(ros::NodeHandle &privateNode)
{
    bool isDetection=false;
    std::string fileParam,dirParam;

    gCamera = new Camera();
    if(privateNode.getParam("ImageStampFile",fileParam)&&privateNode.getParam("ImageDirectory",dirParam))
        gCamera->Initialize(fileParam.c_str(),dirParam.c_str());
    else
        gCamera->Initialize("/home/njust1/Data/exam/Camera-Timestamp.csv",
                                   "/home/njust1/Data/exam/Camera");
    gVelo = new VelodyneHDL32();
    if(privateNode.getParam("VelodyneFile",fileParam))
        gVelo->Initialize(fileParam.c_str());
    else
        gVelo->Initialize("/home/njust1/Data/exam/HDL32.pcap");

    gIMU = new IMU();
    if(privateNode.getParam("IMUFile",fileParam))
        gIMU->Initialize(fileParam.c_str());
    else
        gIMU->Initialize("/home/njust1/Data/exam/IMU.csv");

    gGPS = new GPS();
    if(privateNode.getParam("GPSFile",fileParam))
        gGPS->Initialize(fileParam.c_str());
    else
        gGPS->Initialize("/home/njust1/Data/exam/GPS.csv");
#if 0
    gURG = new URG();
    if(privateNode.getParam("LaserFile",fileParam))
        gURG->Initialize(fileParam.c_str());
    else
        gURG->Initialize("/home/njust1/Data/exam/URG.lms");

    gGear = new Gear();
    if(privateNode.getParam("CanbusGearFile",fileParam))
        gGear->Initialize(fileParam.c_str());
    else
        gGear->Initialize("/home/njust1/Data/exam/Canbus-Gear.csv");

    gOdom = new Odom();
    if(privateNode.getParam("CanbusOdomFile",fileParam))
        gOdom->Initialize(fileParam.c_str());
    else
        gOdom->Initialize("/home/njust1/Data/exam/Canbus-Odom.csv");

    gSteer = new Steer();
    if(privateNode.getParam("CanbusSteerFile",fileParam))
        gSteer->Initialize(fileParam.c_str());
    else
        gSteer->Initialize("/home/njust1/Data/exam/Canbus-Steer.csv");
#endif

    privateNode.getParam("detection",isDetection);
    if(!isDetection)
    {
        if(privateNode.getParam("TaskFile",fileParam))
            LoadTaskFile(fileParam.c_str());
        else
            LoadTaskFile("/home/njust1/Data/exam/Task.txt");
    }
}


int main(int argc, char**argv)
{
    ros::init(argc,argv,"main_node");
    ros::NodeHandle node;
    ros::NodeHandle privateNode("~");

    /// step.1-----------------------------启动指令接收服务-------------------------------//
    pthread_t pid1;
    pthread_create(&pid1,nullptr,RecvCommandfromReferee,0);
    printf("启动指令服务器成功！\n");
    /// step.2-------------------------等待接收准备靠考试指令------------------------------//
    while(Command()!=0x30)
    {
        usleep(10000);
    }
    printf("收到：准备考试\n");

    /// step.3-----------------------------Initialize----------------------------------//
    /// 初始化
    Initialize(privateNode);

    /// step.4------------------------------接收令牌数据---------------------------------//
    pthread_t pid2;
    pthread_create(&pid2,nullptr,RecvTokenfromReferee,0);   // 创建线程接收令牌并解析数据
    printf("启动令牌服务器成功！\n");
    /// step.5------------------------------连接发送答案---------------------------------//
    pthread_t pid3;
    pthread_create(&pid3,nullptr,SendAnswertoReferee,0);   // 创建线程接收令牌并解析数据
    printf("连接答案服务器成功！\n");

    /// step.6----------------------------发送准备就绪指令--------------------------------//
    while(SendCommandtoReferee(0x32)==-1)
    {
        usleep(10000);
    }
    printf("发送：准备就绪\n");

    /// step.7-------------------------等待接收开始测试试指令------------------------------//
    while(Command()!=0x31);
    {
        usleep(10000);
    }
    printf("收到：开始测试\n");

    /// step.8---------------------------设置答案回调处理函数-----------------------------//
    ros::Subscriber subObject=node.subscribe<ros_njust_pca::NJUST_Answer_st_Object>("NJUST_Answer/Object",10,ObjectResCallback);
    ros::Subscriber subLocalMap=node.subscribe<ros_njust_pca::NJUST_Answer_st_Map>("NJUST_Answer/Map",1,LocalMapResCallback);
    ros::Subscriber subLocation=node.subscribe<ros_njust_pca::NJUST_Answer_st_Location>("NJUST_Answer/Location",1,LocationResCallback);

    ros::ServiceServer stateSrv=node.advertiseService("NJUST_Service/InitState", GetInitIMUState);
    ros::ServiceServer cmdSrv=node.advertiseService("NJUST_Service/Command",GetCommand);
    /// step.9---------------------------主线程挂起等待命令-------------------------------//
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        if(Command()==0x33)
        {

            ///////////////////////////////////////////
            ///
            ///   接收到令牌发送完成命令的处理程序 TODO
            ///
            ///////////////////////////////////////////

            ROS_INFO("receive token finished !");
            ROS_INFO("wait to enter [YES] to send [process over] to referee:");
            std::string str;
            std::cin>>str;
            while(str!="YES")
            {
                ROS_INFO("command is not [YES], if you want to send [process over] to referee, please enter [YES]:\n");
                std::cin>>str;
            }
            while(SendCommandtoReferee(0x35)==-1)
            {
                usleep(10000);
            }
            ROS_INFO("send [process over] to referee succeed!\n");

        }
        if(Command()==0x35)
        {

            usleep(1000000);
            break;
        }
        loop_rate.sleep();
    }
    printf("处理完成，程序退出!\n");
}