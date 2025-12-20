#ifndef DIANA7_HW_INTERFACE_H
#define DIANA7_HW_INTERFACE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <vector>
#include <string>

// Diana API
#include "DianaAPI.h"

namespace diana7_hardware
{

class Diana7HWInterface : public hardware_interface::RobotHW
{
public:
  Diana7HWInterface();
  virtual ~Diana7HWInterface();

  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override;

private:
  // ROS Interface
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;

  // Robot State
  std::vector<std::string> joint_names_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_position_command_;

  // Diana API Parameters
  std::string ip_address_;
  int control_freq_;
  
  // Helper to check API errors
  bool checkApiError(int result, const std::string& func_name);
};

} // namespace diana7_hardware

#endif // DIANA7_HW_INTERFACE_H
