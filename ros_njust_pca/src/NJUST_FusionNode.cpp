//
// Created by yangfei on 18-8-30.
//

#include "ros_njust_pca/NJUST_Fusion.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv,"fusion_node");
    ros::NodeHandle node;
    ros::NodeHandle privateNode("~");
    Fusion fusion;
//    fusion.Debug();
    if(fusion.Initialize(node,privateNode))
    {
        fusion.DoNext(node);
    }
}