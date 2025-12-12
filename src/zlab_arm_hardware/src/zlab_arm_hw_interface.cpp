#include <zlab_arm_hardware/zlab_arm_hw_interface.h>
#include <urdf/model.h>
#include <cmath>
#include <cstring>
#include <sstream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>

namespace zlab_arm_hardware
{

ZLabArmHWInterface::ZLabArmHWInterface(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
  : nh_(nh)
  , private_nh_(private_nh)
  , robot_port_(8080)
  , socket_fd_(-1)
  , connected_(false)
  , acc_(0.0)
  , vel_(0.0)
  , cmd_t_(0.002)
  , filter_t_(0.002)
  , gain_(0.0)
{
}

ZLabArmHWInterface::~ZLabArmHWInterface()
{
  disconnectFromRobot();
}

bool ZLabArmHWInterface::init()
{
  // Get robot IP
  if (!private_nh_.getParam("robot_ip", robot_ip_))
  {
    ROS_ERROR("Failed to get param 'robot_ip'");
    return false;
  }

  // Motion parameters are hardcoded (like frrobot implementation)
  // acc_, vel_, cmd_t_, filter_t_, gain_ are initialized in constructor

  // Get joint names from parameter server
  if (!private_nh_.getParam("hardware_interface/joints", joint_names_))
  {
    // Fallback to direct 'joints' parameter
    if (!private_nh_.getParam("joints", joint_names_))
    {
      ROS_ERROR("Failed to get param 'hardware_interface/joints' or 'joints'");
      return false;
    }
  }
  num_joints_ = joint_names_.size();

  if (num_joints_ != 6)
  {
    ROS_ERROR("Expected 6 joints, got %zu", num_joints_);
    return false;
  }

  // Resize vectors
  joint_position_.resize(num_joints_);
  joint_velocity_.resize(num_joints_, 0.0);
  joint_effort_.resize(num_joints_, 0.0);
  joint_position_command_.resize(num_joints_, 0.0);

  // Connect to robot
  if (!connectToRobot())
  {
    return false;
  }

  // Register joint state interface
  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    hardware_interface::JointStateHandle state_handle(
        joint_names_[i],
        &joint_position_[i],
        &joint_velocity_[i],
        &joint_effort_[i]);
    joint_state_interface_.registerHandle(state_handle);
  }
  registerInterface(&joint_state_interface_);

  // Register position joint interface
  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    hardware_interface::JointHandle joint_handle(
        joint_state_interface_.getHandle(joint_names_[i]),
        &joint_position_command_[i]);
    position_joint_interface_.registerHandle(joint_handle);
  }
  registerInterface(&position_joint_interface_);

  // Load URDF and register joint limits
  urdf::Model urdf_model;
  if (!urdf_model.initParam("robot_description"))
  {
    ROS_WARN("Failed to parse URDF, joint limits will not be enforced");
  }
  else
  {
    for (std::size_t i = 0; i < num_joints_; ++i)
    {
      joint_limits_interface::JointLimits limits;
      joint_limits_interface::getJointLimits(joint_names_[i], nh_, limits);
      
      if (urdf_model.getJoint(joint_names_[i]))
      {
        joint_limits_interface::getJointLimits(urdf_model.getJoint(joint_names_[i]), limits);
      }

      joint_limits_interface::PositionJointSaturationHandle sat_handle(
          position_joint_interface_.getHandle(joint_names_[i]), limits);
      position_joint_saturation_interface_.registerHandle(sat_handle);
    }
  }

  ROS_INFO("ZLabArm hardware interface initialized for robot at %s:%d", robot_ip_.c_str(), robot_port_);
  return true;
}

void ZLabArmHWInterface::read(const ros::Time& time, const ros::Duration& period)
{
  if (!connected_)
  {
    return;
  }

  // Request joint states: GetActualJointPosRadian()
  std::string command = "GetActualJointPosRadian()";
  std::string response;
  
  if (sendCommand(command, 375))
  {
    if (receiveResponse(response))
    {
      parseJointStates(response);
    }
  }
}

void ZLabArmHWInterface::write(const ros::Time& time, const ros::Duration& period)
{
  if (!connected_)
  {
    return;
  }

  // Enforce limits
  position_joint_saturation_interface_.enforceLimits(period);

  // Send ServoJ command
  std::string command = buildServoJCommand();
  sendCommand(command, 376);
  receiveResponse(command); // Discard response
}

bool ZLabArmHWInterface::connectToRobot()
{
  socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd_ < 0)
  {
    ROS_ERROR("Failed to create socket: %s", strerror(errno));
    return false;
  }

  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(robot_port_);
  
  if (inet_pton(AF_INET, robot_ip_.c_str(), &server_addr.sin_addr) <= 0)
  {
    ROS_ERROR("Invalid address: %s", robot_ip_.c_str());
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // Set timeout
  struct timeval timeout;
  timeout.tv_sec = 2;
  timeout.tv_usec = 0;
  setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

  ROS_INFO("Connecting to robot at %s:%d...", robot_ip_.c_str(), robot_port_);
  if (connect(socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
  {
    ROS_ERROR("Failed to connect to robot: %s", strerror(errno));
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  connected_ = true;
  ROS_INFO("Successfully connected to robot at %s:%d", robot_ip_.c_str(), robot_port_);
  return true;
}

void ZLabArmHWInterface::disconnectFromRobot()
{
  if (socket_fd_ >= 0)
  {
    close(socket_fd_);
    socket_fd_ = -1;
    connected_ = false;
    ROS_INFO("Disconnected from robot");
  }
}

bool ZLabArmHWInterface::sendCommand(const std::string& command, int cmd_id)
{
  if (!connected_ || socket_fd_ < 0)
  {
    return false;
  }

  std::string frame = buildProtocolFrame(command, cmd_id, command.length());
  ssize_t sent = send(socket_fd_, frame.c_str(), frame.length(), MSG_NOSIGNAL);
  
  if (sent < 0)
  {
    ROS_ERROR("Failed to send command: %s", strerror(errno));
    if (errno == EPIPE || errno == ECONNRESET)
    {
      connected_ = false;
    }
    return false;
  }
  
  return true;
}

bool ZLabArmHWInterface::receiveResponse(std::string& response)
{
  if (!connected_ || socket_fd_ < 0)
  {
    return false;
  }

  char buffer[1024];
  memset(buffer, 0, sizeof(buffer));
  
  ssize_t received = recv(socket_fd_, buffer, sizeof(buffer) - 1, 0);
  
  if (received < 0)
  {
    if (errno == EAGAIN || errno == EWOULDBLOCK)
    {
      ROS_WARN("Receive timeout");
      return false;
    }
    ROS_ERROR("Failed to receive response: %s", strerror(errno));
    if (errno == ECONNRESET || errno == ETIMEDOUT)
    {
      connected_ = false;
    }
    return false;
  }
  
  if (received == 0)
  {
    ROS_ERROR("Connection closed by robot");
    connected_ = false;
    return false;
  }

  response = std::string(buffer, received);
  return true;
}

bool ZLabArmHWInterface::parseJointStates(const std::string& response)
{
  try
  {
    // Parse protocol frame: /f/bIII123III375III{len}III{data}III/b/f
    size_t pos = 0;
    std::string temp = response;
    
    // Skip to data section
    for (int i = 0; i < 3; ++i)
    {
      pos = temp.find("III");
      if (pos == std::string::npos) return false;
      temp = temp.substr(pos + 3);
    }
    
    // Get data length
    pos = temp.find("III");
    if (pos == std::string::npos) return false;
    int data_len = std::stoi(temp.substr(0, pos));
    temp = temp.substr(pos + 3);
    
    // Extract data
    std::string data = temp.substr(0, data_len);
    
    // Parse comma-separated joint values
    std::istringstream iss(data);
    std::string token;
    std::size_t i = 0;
    
    while (std::getline(iss, token, ',') && i < num_joints_)
    {
      joint_position_[i++] = std::stod(token);
    }
    
    return (i == num_joints_);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Failed to parse joint states: %s", e.what());
    return false;
  }
}

std::string ZLabArmHWInterface::buildServoJCommand()
{
  // Convert radians to degrees
  std::ostringstream oss;
  oss << "ServoJ(";
  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    if (i > 0) oss << ",";
    oss << (joint_position_command_[i] * 180.0 / M_PI);
  }
  oss << "," << acc_ << "," << vel_ << "," << cmd_t_ << "," << filter_t_ << "," << gain_ << ")";
  return oss.str();
}

std::string ZLabArmHWInterface::buildProtocolFrame(const std::string& command, int cmd_id, int cmd_len)
{
  std::ostringstream oss;
  oss << "/f/bIII123III" << cmd_id << "III" << cmd_len << "III" << command << "III/b/f";
  return oss.str();
}

} // namespace zlab_arm_hardware

