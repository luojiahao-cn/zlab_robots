#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <zlab_arm_hardware/zlab_arm_hw_interface.h>
#include <time.h>

static const double BILLION = 1000000000.0;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "zlab_arm_hardware");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // Create hardware interface
  zlab_arm_hardware::ZLabArmHWInterface hw(nh, private_nh);
  if (!hw.init())
  {
    ROS_ERROR("Failed to initialize hardware interface");
    return 1;
  }

  // Create controller manager
  // Use nh (group namespace) so controller_spawner can find it
  // In group ns="arm1", nh will be "/arm1", private_nh will be "/arm1/zlab_arm_hardware"
  controller_manager::ControllerManager cm(&hw, nh);

  // Get control loop parameters
  double loop_hz;
  private_nh.param("loop_hz", loop_hz, 60.0);
  ros::Duration desired_update_period(1.0 / loop_hz);
  
  double cycle_time_error_threshold;
  private_nh.param("cycle_time_error_threshold", cycle_time_error_threshold, 0.01);

  // Timing
  struct timespec last_time;
  struct timespec current_time;
  clock_gettime(CLOCK_MONOTONIC, &last_time);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ROS_INFO("Starting control loop at %f Hz", loop_hz);

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
    //   ROS_WARN("Control loop missed desired period by %.4fs", cycle_time_error);
    }

    // Read from hardware
    hw.read(ros::Time::now(), elapsed_time);

    // Update controllers
    cm.update(ros::Time::now(), elapsed_time);

    // Write to hardware
    hw.write(ros::Time::now(), elapsed_time);

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

