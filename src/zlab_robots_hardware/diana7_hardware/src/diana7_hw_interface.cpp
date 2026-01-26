#include "diana7_hardware/diana7_hw_interface.h"

namespace diana7_hardware
{

// Callback functions for DianaAPI
void errorControlCallback(int e, const char *strIpAddress)
{
  // ROS_ERROR("DianaAPI Error: %d", e);
}

void robotStateCallback(StrRobotStateInfo *pinfo, const char *strIpAddress)
{
  // Heartbeat or state update
}

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
  
  int ret = initSrv(errorControlCallback, robotStateCallback, &net_info);
  if (!checkApiError(ret, "initSrv")) return false;

  // Ensure we are in Position Control Mode
  ret = changeControlMode(T_MODE_POSITION, ip_address_.c_str());
  if (!checkApiError(ret, "changeControlMode")) return false;

  // Release brake (Required to move)
  ROS_INFO("Diana7HWInterface: Releasing brake...");
  ret = releaseBrake(ip_address_.c_str());
  if (!checkApiError(ret, "releaseBrake")) return false;
  
  // Wait for brake release (2 seconds as per reference)
  ros::Duration(2.0).sleep();

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
    ROS_INFO("Diana7HWInterface: Initial Joint Pos: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
             initial_joints[0], initial_joints[1], initial_joints[2], 
             initial_joints[3], initial_joints[4], initial_joints[5], initial_joints[6]);
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
  int ret_read = getJointPos(joints, ip_address_.c_str());
  if (ret_read == 0)
  {
    for (int i = 0; i < 7; ++i) joint_position_[i] = joints[i];
  }
  else
  {
    ROS_WARN_THROTTLE(1.0, "Diana7HWInterface: getJointPos failed: %d", ret_read);
  }

  // Read Velocity and Torque are commented out to reduce loop latency
  // getJointAngularVel(vels, ip_address_.c_str()); 
  // for (int i = 0; i < 7; ++i) joint_velocity_[i] = vels[i];

  // getJointTorque(torques, ip_address_.c_str());
  // for (int i = 0; i < 7; ++i) joint_effort_[i] = torques[i];
}

void Diana7HWInterface::write(const ros::Time& time, const ros::Duration& period)
{
  double target_joints[7];
  for (int i = 0; i < 7; ++i)
  {
    target_joints[i] = joint_position_command_[i];
  }

  // Use servoJ_ex for high-frequency position control
  // Parameters tuned based on successful standalone test
  double servo_period = 0.01;      // 100Hz
  double lookahead = 0.03;         // 3 * period
  double gain = 200.0;             // Stiffness
  bool reliable = true;            // Reliable UDP/TCP

  int ret_write = servoJ_ex(target_joints, servo_period, lookahead, gain, reliable, ip_address_.c_str());

  if (ret_write != 0)
  {
    ROS_WARN_THROTTLE(1.0, "Diana7HWInterface: servoJ_ex failed: %d", ret_write);
  }
  
  // Debug print (throttle to 1Hz)
  // ROS_INFO_THROTTLE(1.0, "Write: [%.4f...], Read: [%.4f...], RetWrite: %d", 
  //     target_joints[6], joint_position_[6], ret_write);
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
