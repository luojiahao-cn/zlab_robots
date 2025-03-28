/*
   Desc:   Example ros_control hardware interface blank template for the FrRobot
           For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include <frrobot_control/frrobot_hw_interface.h>
#include "frrobot_control/udp_node.h"
#include "frrobot_control/tool.h"
#include <math.h>
#include <unistd.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <frrobot_control/FRRobot.h>
#include <frrobot_control/RobotError.h>
#include <frrobot_control/RobotTypes.h>
// 添加线程相关头文件
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

#define MAXLINE 4096
#define PORT_CMD 8080

int confd;
int len;
int flag = 1;
socklen_t sendaddr_length;
char recvLine[MAXLINE];
char sendCmdLine[MAXLINE];
char sendStaLine[MAXLINE];
char recv_buf[MAXLINE];
char send_buf[MAXLINE-200];
struct sockaddr_in serverSendAddr;

namespace frrobot_control
{

  FrRobotHWInterface::FrRobotHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
      : ros_control_boilerplate::GenericHWInterface(nh, urdf_model),
        is_running_(false), need_reconnect_(false), has_new_command_(false)
  {
    std::string robot_ip;
    if (!nh.getParam("robot_ip", robot_ip))
    {
        ROS_ERROR("Failed to get param 'robot_ip'");
        robot_ip = "192.168.31.202";
        ROS_WARN("Param 'robot_ip' not found, using default IP: %s", robot_ip.c_str());
    }

    // 初始化控制参数
    a_ = 0.0;
    v_ = 0.0;
    t_ = 0.002;
    lat_ = 0.002;
    gain_ = 0.0;
    
    // 初始化关节位置缓存
    memset(cached_joint_positions_, 0, sizeof(cached_joint_positions_));
    memset(cached_joint_commands_, 0, sizeof(cached_joint_commands_));

    // 设置服务器地址
    memset(&serverSendAddr, 0, sizeof(serverSendAddr));
    serverSendAddr.sin_family = AF_INET;
    serverSendAddr.sin_addr.s_addr = inet_addr(robot_ip.c_str());
    serverSendAddr.sin_port = htons(PORT_CMD);
    sendaddr_length = sizeof(serverSendAddr);

    // 创建socket
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if ((confd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
      ROS_ERROR("socket() error");
      perror("socket() error");
      exit(1);
    }
    
    // 设置接收超时
    struct timeval timeout;
    timeout.tv_sec = 2;
    timeout.tv_usec = 0;
    if (setsockopt(confd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout, sizeof(timeout)) < 0)
    {
        ROS_ERROR("setsockopt() error");
        perror("setsockopt() error");
        exit(1);
    }

    ROS_INFO("Try connect to server IP: %s, port: %d", robot_ip.c_str(), PORT_CMD);
    if (connect(confd, (struct sockaddr *)&serverSendAddr, sizeof(serverSendAddr)) < 0)
    {
      ROS_ERROR("connect() error");
      perror("connect() error");
      exit(1);
    }

    ROS_INFO("Connected to server");
    
    // 启动通信线程
    is_running_ = true;
    comm_thread_ = std::thread(&FrRobotHWInterface::communicationThread, this);

    ROS_INFO_NAMED("frrobot_hw_interface", "FrRobotHWInterface Ready.");
  }
  
  // 析构函数中停止线程
  FrRobotHWInterface::~FrRobotHWInterface()
  {
    is_running_ = false;
    if (comm_thread_.joinable()) {
      comm_thread_.join();
    }
    
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (confd >= 0) {
      close(confd);
      confd = -1;
    }
  }

  // 通信线程主函数
  void FrRobotHWInterface::communicationThread()
  {
    ROS_INFO("Communication thread started");
    
    while (is_running_)
    {
      // 检查是否需要重连
      if (need_reconnect_)
      {
        socketReconnect();
        need_reconnect_ = false;
      }
      
      // 获取关节位置
      if (!getJointPositions())
      {
        need_reconnect_ = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
      
      // 检查是否有新命令需要发送
      {
        std::unique_lock<std::mutex> lock(joints_mutex_);
        if (has_new_command_)
        {
          std::string cmd, response;
          sprintf(send_buf, "ServoJ(%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f)", 
                 cached_joint_commands_[0], cached_joint_commands_[1], 
                 cached_joint_commands_[2], cached_joint_commands_[3], 
                 cached_joint_commands_[4], cached_joint_commands_[5], 
                 a_, v_, t_, lat_, gain_);
          
          cmd = std::string(send_buf);
          if (!sendCommand(cmd, response))
          {
            need_reconnect_ = true;
          }
          
          has_new_command_ = false;
          cmd_cv_.notify_one();
        }
      }
      
      // 控制通信频率
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    ROS_INFO("Communication thread stopped");
  }
  
  // 安全地重新连接socket
  void FrRobotHWInterface::socketReconnect()
  {
    ROS_INFO("Reconnecting to server...");
    
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (confd >= 0) {
      close(confd);
    }
    
    if ((confd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
      ROS_ERROR("socket() error during reconnect");
      perror("socket() error");
      return;
    }
    
    // 设置接收超时
    struct timeval timeout;
    timeout.tv_sec = 2;
    timeout.tv_usec = 0;
    if (setsockopt(confd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout, sizeof(timeout)) < 0)
    {
      ROS_ERROR("setsockopt() error during reconnect");
      perror("setsockopt() error");
      return;
    }
    
    if (connect(confd, (struct sockaddr *)&serverSendAddr, sizeof(serverSendAddr)) < 0)
    {
      ROS_ERROR("connect() error during reconnect");
      perror("connect() error");
      return;
    }
    
    ROS_INFO("Reconnected to server successfully");
  }
  
  // 发送命令并获取响应
  bool FrRobotHWInterface::sendCommand(const std::string& cmd, std::string& response)
  {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    
    int len = cmd.length();
    char sendCmdLine[MAXLINE];
    sprintf(sendCmdLine, "/f/bIII123III376III%dIII%sIII/b/f", len, cmd.c_str());
    
    int send_length = send(confd, sendCmdLine, strlen(sendCmdLine), 0);
    if (send_length < 0)
    {
      ROS_ERROR("Failed to send command: %s", cmd.c_str());
      return false;
    }
    
    char recvLine[MAXLINE];
    recvLine[MAXLINE - 1] = '\0';
    int recv_length = recv(confd, recvLine, sizeof(recvLine), 0);
    if (recv_length <= 0)
    {
      ROS_ERROR("Failed to receive response for command: %s", cmd.c_str());
      return false;
    }
    
    response = std::string(recvLine);
    return true;
  }
  
  // 获取关节位置
  bool FrRobotHWInterface::getJointPositions()
  {
    std::string cmd = "GetActualJointPosRadian()";
    std::string response;
    
    {
      std::lock_guard<std::mutex> lock(socket_mutex_);
      
      char sendStaLine[MAXLINE];
      sprintf(sendStaLine, "/f/bIII123III375III25IIIGetActualJointPosRadian()III/b/f");
      
      int send_length = send(confd, sendStaLine, strlen(sendStaLine), 0);
      if (send_length < 0)
      {
        ROS_ERROR("Failed to send joint position request");
        return false;
      }
      
      char recvLine[MAXLINE];
      recvLine[MAXLINE - 1] = '\0';
      int recv_length = recv(confd, recvLine, sizeof(recvLine), 0);
      if (recv_length <= 0)
      {
        ROS_ERROR("Failed to receive joint positions");
        return false;
      }
      
      response = std::string(recvLine);
    }
    
    // 解析响应
    int pos = 0;
    int jointsDataLen = 0;
    std::string tempJoints = response;
    for (int i = 0; i < 3; i++)
    {
      pos = tempJoints.find("III") + 3;
      tempJoints = tempJoints.substr(pos);
    }
    pos = tempJoints.find("III");
    jointsDataLen = stoi(tempJoints.substr(0, pos));
    tempJoints = tempJoints.substr(pos + 3, jointsDataLen);
    
    float temp_joints[6];
    for (int i = 0; i < 6; i++)
    {
      pos = tempJoints.find(",");
      temp_joints[i] = stof(tempJoints.substr(0, pos));
      tempJoints = tempJoints.substr(pos + 1);
    }
    
    // 更新缓存的关节位置
    {
      std::lock_guard<std::mutex> lock(joints_mutex_);
      memcpy(cached_joint_positions_, temp_joints, sizeof(temp_joints));
    }
    
    return true;
  }

  // 更新后的write函数
  void FrRobotHWInterface::write(ros::Duration &elapsed_time)
  {
    // 安全限制
    enforceLimits(elapsed_time);
    
    float temp_cmd[6];
    for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
    {
      temp_cmd[joint_id] = joint_position_command_[joint_id] * 180 / M_PI;
    }
    
    // 更新命令并通知通信线程
    {
      std::unique_lock<std::mutex> lock(joints_mutex_);
      memcpy(cached_joint_commands_, temp_cmd, sizeof(temp_cmd));
      has_new_command_ = true;
      
      // 等待命令发送完成或超时
      cmd_cv_.wait_for(lock, std::chrono::milliseconds(50), 
                       [this]{ return !has_new_command_; });
    }
  }

  // 更新后的read函数
  void FrRobotHWInterface::read(ros::Duration &elapsed_time)
  {
    // 从缓存的关节位置更新控制器状态
    {
      std::lock_guard<std::mutex> lock(joints_mutex_);
      for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
      {
        joint_position_[joint_id] = cached_joint_positions_[joint_id];
      }
    }
  }

  // 原有的reconnect函数不再直接调用，而是设置标志让通信线程处理
  void FrRobotHWInterface::reconnect()
  {
    need_reconnect_ = true;
  }

  void FrRobotHWInterface::enforceLimits(ros::Duration &period)
  {
    // ----------------------------------------------------
    // ----------------------------------------------------
    // ----------------------------------------------------
    //
    // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
    // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
    // DEPENDING ON YOUR CONTROL METHOD
    //
    // EXAMPLES:
    //
    // Saturation Limits ---------------------------
    //
    // Enforces position and velocity
    pos_jnt_sat_interface_.enforceLimits(period);
    //
    // Enforces velocity and acceleration limits
    // vel_jnt_sat_interface_.enforceLimits(period);
    //
    // Enforces position, velocity, and effort
    // eff_jnt_sat_interface_.enforceLimits(period);

    // Soft limits ---------------------------------
    //
    // pos_jnt_soft_limits_.enforceLimits(period);
    // vel_jnt_soft_limits_.enforceLimits(period);
    // eff_jnt_soft_limits_.enforceLimits(period);
    //
    // ----------------------------------------------------
    // ----------------------------------------------------
    // ----------------------------------------------------
  }

} // namespace frrobot_control
