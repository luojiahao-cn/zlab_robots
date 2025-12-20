#include "diana7_hardware/diana7_hw_interface.h"

namespace diana7_hardware
{

Diana7HWInterface::Diana7HWInterface()
{
}

Diana7HWInterface::~Diana7HWInterface()
{
  // Stop the robot or release resources if needed
  // stop(ip_address_.c_str());
  destroySrv(ip_address_.c_str());
}

bool Diana7HWInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  // 1. Get Parameters
  if (!robot_hw_nh.getParam("ip_address", ip_address_))
  {
    ROS_ERROR("Diana7HWInterface: Parameter 'ip_address' not found");
    return false;
  }
  
  if (!robot_hw_nh.getParam("joints", joint_names_))
  {
    ROS_ERROR("Diana7HWInterface: Parameter 'joints' not found");
    return false;
  }

  if (joint_names_.size() != 7)
  {
    ROS_ERROR("Diana7HWInterface: Expected 7 joint names, got %lu", joint_names_.size());
    return false;
  }

  // 2. Initialize Diana API
  srv_net_st net_info;
  initSrvNetInfo(&net_info); // Initialize struct first!

  strcpy(net_info.SrvIp, ip_address_.c_str());
  net_info.LocHeartbeatPort = 0; // 0 means use default random port
  net_info.LocRobotStatePort = 0;
  net_info.LocSrvPort = 0;
  net_info.LocRealtimeSrvPort = 0;
  net_info.LocPassThroughSrvPort = 0;
  
  int ret = initSrv(nullptr, nullptr, &net_info);
  if (!checkApiError(ret, "initSrv")) return false;

  // Ensure we are in Position Control Mode
  ret = changeControlMode(T_MODE_POSITION, ip_address_.c_str());
  if (!checkApiError(ret, "changeControlMode")) return false;

  // Release brake (optional, be careful)
  // ret = releaseBrake(ip_address_.c_str());
  // if (!checkApiError(ret, "releaseBrake")) return false;

  ROS_INFO("Diana7HWInterface: Connected to robot at %s", ip_address_.c_str());

  // 3. Initialize Hardware Interfaces
  joint_position_.resize(7, 0.0);
  joint_velocity_.resize(7, 0.0);
  joint_effort_.resize(7, 0.0);
  joint_position_command_.resize(7, 0.0);

  // Read initial position to sync command
  double initial_joints[7];
  ret = getJointPos(initial_joints, ip_address_.c_str());
  if (ret == 0)
  {
    for (int i = 0; i < 7; ++i)
    {
      joint_position_[i] = initial_joints[i];
      joint_position_command_[i] = initial_joints[i]; // Sync command to current pos
    }
  }
  else
  {
    ROS_ERROR("Diana7HWInterface: Failed to get initial joint position");
    return false;
  }

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    // State Handle
    hardware_interface::JointStateHandle state_handle(
        joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
    joint_state_interface_.registerHandle(state_handle);

    // Position Command Handle
    hardware_interface::JointHandle pos_handle(
        joint_state_interface_.getHandle(joint_names_[i]), &joint_position_command_[i]);
    position_joint_interface_.registerHandle(pos_handle);
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);

  ROS_INFO("Diana7HWInterface: Initialized successfully");
  return true;
}

void Diana7HWInterface::read(const ros::Time& time, const ros::Duration& period)
{
  double joints[7];
  double vels[7];
  double torques[7];

  // Read Position
  int ret = getJointPos(joints, ip_address_.c_str());
  if (ret == 0)
  {
    for (int i = 0; i < 7; ++i) joint_position_[i] = joints[i];
  }

  // Read Velocity
  ret = getJointAngularVel(vels, ip_address_.c_str());
  if (ret == 0)
  {
    for (int i = 0; i < 7; ++i) joint_velocity_[i] = vels[i];
  }

  // Read Torque
  ret = getJointTorque(torques, ip_address_.c_str());
  if (ret == 0)
  {
    for (int i = 0; i < 7; ++i) joint_effort_[i] = torques[i];
  }
}

void Diana7HWInterface::write(const ros::Time& time, const ros::Duration& period)
{
  // Use servoJ for high-frequency position control
  // servoJ(double *joints, double time, double look_ahead_time, double gain, const char *strIpAddress)
  // time: expected time to reach target (e.g. 0.001s for 1000Hz)
  // look_ahead_time: usually slightly larger than time (e.g. 0.02)
  // gain: control gain (e.g. 100-300)
  
  double target_joints[7];
  for (int i = 0; i < 7; ++i)
  {
    target_joints[i] = joint_position_command_[i];
  }

  // Parameters need tuning based on real robot behavior
  double t = period.toSec(); // e.g. 0.001
  if (t < 0.002) t = 0.002;  // Clamp min time to 2ms (500Hz) or higher if needed
  
  double look_ahead = 0.05; 
  double gain = 300.0;

  int ret = servoJ(target_joints, t, look_ahead, gain, ip_address_.c_str());
  if (ret != 0)
  {
    // Don't spam error logs in high freq loop, maybe throttle
    ROS_WARN_THROTTLE(1.0, "Diana7HWInterface: servoJ failed: %d", ret);
  }
  
  // Debug print (throttle to 1Hz)
  ROS_INFO_THROTTLE(1.0, "Write: [%.4f, %.4f, ...], Read: [%.4f, %.4f, ...]", 
      target_joints[6], target_joints[6], joint_position_[6], joint_position_[6]);
}

bool Diana7HWInterface::checkApiError(int result, const std::string& func_name)
{
  if (result != 0)
  {
    ROS_ERROR("Diana7HWInterface: %s failed with error code %d", func_name.c_str(), result);
    // You can use formatError(result) if available
    return false;
  }
  return true;
}

} // namespace diana7_hardware
