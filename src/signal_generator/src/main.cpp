#include <ros/ros.h>
#include "signal_generator/fy8300_node.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "fy8300_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    signal_generator::FY8300Node node(nh, pnh);
    
    if (!node.init()) {
        ROS_FATAL("Closing node due to connection failure.");
        return 1;
    }

    ros::spin();
    return 0;
}
