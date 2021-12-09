#include <ros/ros.h>
#include "loam_velodyne/MultiScanRegistration.h"


/** Main node entry point. */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "scanRegistration");    //初始化节点
    ros::NodeHandle node;
    ros::NodeHandle privateNode("~");
    //创建类对象
    loam::MultiScanRegistration multiScan;

    // DoNext
    if (multiScan.setup(node, privateNode)) {
        // initialization successful
        ros::spin();
    }

    return 0;
}
