#include "diana7_hardware/diana7_hw_interface.h"
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "diana7_hw_main");
  ros::NodeHandle nh;
  ros::NodeHandle robot_hw_nh("~");

  // 1. Create Hardware Interface
  diana7_hardware::Diana7HWInterface diana7_hw;
  if (!diana7_hw.init(nh, robot_hw_nh))
  {
    ROS_ERROR("Failed to initialize Diana7 Hardware Interface");
    return 1;
  }

  // 2. Create Controller Manager
  controller_manager::ControllerManager cm(&diana7_hw, nh);

  // 3. Control Loop
  // Diana API recommends 1000Hz for servoJ
  double loop_hz = 1000.0;
  robot_hw_nh.param<double>("loop_hz", loop_hz, 1000.0);
  
  ros::Rate rate(loop_hz);
  ros::AsyncSpinner spinner(2); // Use async spinner for services
  spinner.start();

  ros::Time last_time = ros::Time::now();

  while (ros::ok())
  {
    ros::Time current_time = ros::Time::now();
    ros::Duration period = current_time - last_time;
    last_time = current_time;

    // Read from robot
    diana7_hw.read(current_time, period);

    // Update controllers
    cm.update(current_time, period);

    // Write to robot
    diana7_hw.write(current_time, period);

    rate.sleep();
  }

  spinner.stop();
  return 0;
}
