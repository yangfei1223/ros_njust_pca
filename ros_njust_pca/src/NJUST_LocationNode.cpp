//
// Created by yangfei on 18-9-6.
//

#include "ros_njust_pca/NJUST_Location.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv,"location_node");
    ros::NodeHandle node;
    Location location;
    if(location.Initialize(node))
    {
        location.DoNext(node);
    }
}