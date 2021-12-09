//
// Created by yangfei on 18-8-3.
//

#include "ros_njust_pca/NJUST_Tcp.h"
#include "ros_njust_pca/NJUST_Data.h"

extern Camera *gCamera;
extern VelodyneHDL32 *gVelo;
extern Gear *gGear;
extern Odom *gOdom;
extern Steer *gSteer;
extern GPS *gGPS;
extern IMU *gIMU;
extern URG gURG;

extern char Command();
extern void SetCommand(char cmd);

extern bool gFlag;
extern int gBytes;
extern char gBuffer[MAX_IP_LENGTH];
extern pthread_mutex_t g_mutex_buffer;
bool isNewAns()
{
    bool ret;
    pthread_mutex_lock(&g_mutex_buffer);
    ret =gFlag;
    pthread_mutex_unlock(&g_mutex_buffer);
    return ret;
}

extern void SwapEndian16(char* p);
extern void SwapEndian32(char* p);
extern void SwapEndian64(char* p);
/// 从裁判接收指令数据，我方为server（短连接）
void *RecvCommandfromReferee(void *arg)
{
    int socket_fd, connect_fd;
    struct sockaddr_in servaddr;
    char buffer[3];
    int n;
    //初始化Socket
    if( (socket_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1 )
    {
        printf("create socket error: %s(errno: %d)\n",strerror(errno),errno);
        exit(0);
    }
    //初始化
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);//IP地址设置成INADDR_ANY,让系统自动获取本机的IP地址。
    servaddr.sin_port = htons(7001);//设置的端口为DEFAULT_PORT

    //将本地地址绑定到所创建的套接字上
    if( bind(socket_fd, (struct sockaddr*)&servaddr, sizeof(servaddr)) == -1)
    {
        printf("bind socket error: %s(errno: %d)\n",strerror(errno),errno);
        exit(0);
    }
    //开始监听是否有客户端连接
    if( listen(socket_fd, 10) == -1)
    {
        printf("listen socket error: %s(errno: %d)\n",strerror(errno),errno);
        exit(0);
    }
    printf("======waiting for client's command======\n");
    while(1)
    {
        //阻塞直到有客户端连接，不然多浪费CPU资源。
        if( (connect_fd = accept(socket_fd, (struct sockaddr*)NULL, NULL)) == -1)
        {
            printf("accept socket error: %s(errno: %d)",strerror(errno),errno);
            continue;
        }
        //接受客户端传过来的数据
        if(recv(connect_fd, buffer, 3, 0)<0)
            continue;
        SetCommand(buffer[1]);
        if(Command()==0x33)
            break;
        close(connect_fd);
    }
    close(socket_fd);
}

/// 发送指令数据给裁判，我方为client（短连接）
int SendCommandtoReferee(char cmd)
{
    int sockfd;
    char buffer[3];
    buffer[0]=0x02;
    buffer[1]=cmd;
    buffer[2]=0x03;
    struct sockaddr_in  servaddr;

    if( (sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("create socket error: %s(errno: %d)\n", strerror(errno),errno);
        return -1;
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(7000);
    if( inet_pton(AF_INET, "127.0.0.1", &servaddr.sin_addr) <= 0)
    {
        printf("inet_pton error for %s\n","127.0.0.1");
        return -1;
    }
    if( connect(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0)
    {
        printf("connect error: %s(errno: %d)\n",strerror(errno),errno);
        return -1;
    }
    if( send(sockfd, buffer, 3, 0) < 0)
    {
        printf("send msg error: %s(errno: %d)\an", strerror(errno), errno);
        return -1;
    }
    close(sockfd);
    SetCommand(cmd);
    return 0;
}

/// 从裁判接收令牌，我方为server（长连接）
void *RecvTokenfromReferee(void *arg)
{
    int socket_fd, connect_fd;
    struct sockaddr_in servaddr;
//    char buffer[10];
    int64_t timestamp;
    char flag;
    char type;
    int8_t token;
    //初始化Socket
    if( (socket_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1 )
    {
        printf("create socket error: %s(errno: %d)\n",strerror(errno),errno);
        exit(0);
    }
    //初始化
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);//IP地址设置成INADDR_ANY,让系统自动获取本机的IP地址。
    servaddr.sin_port = htons(7002);//设置的端口为DEFAULT_PORT

    //将本地地址绑定到所创建的套接字上
    if( bind(socket_fd, (struct sockaddr*)&servaddr, sizeof(servaddr)) == -1)
    {
        printf("bind socket error: %s(errno: %d)\n",strerror(errno),errno);
        exit(0);
    }
    //开始监听是否有客户端连接
    if( listen(socket_fd, 10) == -1)
    {
        printf("listen socket error: %s(errno: %d)\n",strerror(errno),errno);
        exit(0);
    }
    printf("======waiting for client's token======\n");
    //阻塞直到有客户端连接，不然多浪费CPU资源。
    while( (connect_fd = accept(socket_fd, (struct sockaddr*)NULL, NULL)) == -1)
    {
        printf("accept socket error: %s(errno: %d, retry!)",strerror(errno),errno);
        usleep(10000);
    }

    // 接收令牌数据
    while(1)
    {
        //接受客户端传过来的数据
        recv(connect_fd,&flag,1,0);
        if(flag==0x02)
        {
            recv(connect_fd, &timestamp, 8, 0);

//            memcpy(&timestamp, buffer, 8);
#ifdef SEND_BIG_ENDIAN
//            timestamp = be64toh(timestamp);     // 大端转小端
            SwapEndian64((char*)&timestamp);
#endif
//            type = buffer[8];
//            token = buffer[9];
            recv(connect_fd, &type, 1, 0);
            recv(connect_fd, &token, 1, 0);

//            printf("*****receive token from referee, timestamp is %d, type is %c, token is %d*****\n",timestamp,type,token);
            switch (type)
            {
                case 'C':
                    if (gCamera->DoNext(timestamp, token))
                        gCamera->Pubish();
                    break;
                case 'L':
                    if (gVelo->DoNext(timestamp, token))
                    {
                        gVelo->Publish();
                        gVelo->ReSet();
                    }
                    break;
                case 'U':
                    if (gIMU->DoNext(timestamp, token))
                        gIMU->Publish();
                    break;
                case 'P':
                    if (gGPS->DoNext(timestamp, token))
                        gGPS->Publish();
                    break;
#if 0
                case 'R':
                    if(gURG.DoNext(timestamp,token))
                        gURG.Publish();
                    break;
                case 'G':
                    if (gGear->DoNext(timestamp, token))
                        gGear->Publish();
                    break;
                case 'O':
                    if (gOdom->DoNext(timestamp, token))
                        gOdom->Publish();
                    break;
                case 'S':
                    if (gSteer->DoNext(timestamp, token))
                        gSteer->Publish();
                    break;
#endif
                default:
                    break;
            }
        }
        if(Command()==0x33)
            break;
    }
    close(connect_fd);
    close(socket_fd);
}

/// 发送答案数据给裁判，我方为client（长连接）
void *SendAnswertoReferee(void *arg)
{
    int sockfd;
    struct sockaddr_in servaddr;

    if( (sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("create socket error: %s(errno: %d)\n", strerror(errno),errno);
        exit(-1);
    }
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(7003);
    if( inet_pton(AF_INET, "127.0.0.1", &servaddr.sin_addr) <= 0)
    {
        printf("inet_pton error for %s\n","127.0.0.1");
        exit(-1);
    }

    if( connect(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0)
    {
        printf("connect error: %s(errno: %d)\n",strerror(errno),errno);
        exit(-1);
    }
    int nBytes;
    char buffer[MAX_IP_LENGTH];
    while(1)
    {
        if(isNewAns())
        {

            pthread_mutex_lock(&g_mutex_buffer);
            nBytes=gBytes;
            memcpy(buffer,gBuffer,MAX_IP_LENGTH);
            gFlag=false;
            pthread_mutex_unlock(&g_mutex_buffer);
            if(send(sockfd,buffer,nBytes,0) < 0)
            {
                printf("send msg error: %s(errno: %d)\an", strerror(errno), errno);
            }
        }
        else
        {
            usleep(50000);
        }
        if(Command()==0x35)
            break;
    }
    close(sockfd);
}