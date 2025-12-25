#include "diana7_hardware/diana7_hw_interface.h"
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <time.h>

static const double BILLION = 1000000000.0;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "diana7_hardware");
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
  ros::Duration desired_update_period(1.0 / loop_hz);
  
  double cycle_time_error_threshold;
  robot_hw_nh.param<double>("cycle_time_error_threshold", cycle_time_error_threshold, desired_update_period.toSec() * 0.5);

  // Timing
  struct timespec last_time;
  struct timespec current_time;
  clock_gettime(CLOCK_MONOTONIC, &last_time);

  ros::AsyncSpinner spinner(2); // Use async spinner for services
  spinner.start();

  ROS_INFO("Starting diana7 control loop at %f Hz", loop_hz);

  while (ros::ok())
  {
    // Get monotonic time
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    
    // Calculate elapsed time
    ros::Duration elapsed_time(
        (current_time.tv_sec - last_time.tv_sec) + 
        (current_time.tv_nsec - last_time.tv_nsec) / BILLION);
    last_time = current_time;

    // Check cycle time
    const double cycle_time_error = (elapsed_time - desired_update_period).toSec();
    if (cycle_time_error > cycle_time_error_threshold)
    {
      ROS_WARN_THROTTLE(5.0, "[%s] Control loop missed desired period by %.4fs", 
                        nh.getNamespace().c_str(), cycle_time_error);
    }

    // Read from robot
    diana7_hw.read(ros::Time::now(), elapsed_time);

    // Update controllers
    cm.update(ros::Time::now(), elapsed_time);

    // Write to robot
    diana7_hw.write(ros::Time::now(), elapsed_time);

    // Sleep to maintain loop rate
    ros::Duration sleep_time = desired_update_period - elapsed_time;
    if (sleep_time.toSec() > 0.0)
    {
      sleep_time.sleep();
    }
  }

  spinner.stop();
  return 0;
}
