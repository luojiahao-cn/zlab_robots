#include "magnetic_localization/magnetic_simulation.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "magnetic_simulation");
    ros::NodeHandle nh("~");
    
    try {
        MagneticSimulation simulation;
        simulation.run();
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Simulation failed: " << e.what());
        return 1;
    }
    
    return 0;
}