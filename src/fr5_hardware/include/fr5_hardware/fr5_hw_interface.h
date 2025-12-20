#ifndef ZLAB_ARM_HARDWARE__ZLAB_ARM_HW_INTERFACE_H
#define ZLAB_ARM_HARDWARE__ZLAB_ARM_HW_INTERFACE_H

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <controller_manager/controller_manager.h>
#include <urdf/model.h>
#include <ros/ros.h>
#include <string>
#include <vector>

namespace fr5_hardware
{

class ZLabArmHWInterface : public hardware_interface::RobotHW
{
public:
  ZLabArmHWInterface(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  ~ZLabArmHWInterface();

  bool init();
  void read(const ros::Time& time, const ros::Duration& period);
  void write(const ros::Time& time, const ros::Duration& period);

private:
  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // Hardware resources
  std::vector<std::string> joint_names_;
  std::size_t num_joints_;

  // State
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;

  // Command
  std::vector<double> joint_position_command_;

  // Interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  joint_limits_interface::PositionJointSaturationInterface position_joint_saturation_interface_;

  // TCP communication
  std::string robot_ip_;
  int robot_port_;
  int socket_fd_;
  bool connected_;

  // Motion parameters
  double acc_;      // Acceleration
  double vel_;      // Velocity
  double cmd_t_;    // Command period
  double filter_t_; // Filter time
  double gain_;     // Gain

  // Protocol helpers
  bool connectToRobot();
  void disconnectFromRobot();
  bool sendCommand(const std::string& command, int cmd_id);
  bool receiveResponse(std::string& response);
  bool parseJointStates(const std::string& response);
  std::string buildServoJCommand();
  std::string buildProtocolFrame(const std::string& command, int cmd_id, int cmd_len);
};

} // namespace fr5_hardware

#endif // ZLAB_ARM_HARDWARE__ZLAB_ARM_HW_INTERFACE_H

