//
// Created by yangfei on 18-8-3.
//

#ifndef NJUST_PCALAB_NJUST_TCP_H
#define NJUST_PCALAB_NJUST_TCP_H

#include "NJUST_Global.h"

void *RecvCommandfromReferee(void *arg);   // 子线程
int SendCommandtoReferee(char cmd);     // 主线程
void *RecvTokenfromReferee(void *arg);      // 子线程
void *SendAnswertoReferee(void *arg);      // 子线程

#endif //NJUST_PCALAB_NJUST_TCP_H