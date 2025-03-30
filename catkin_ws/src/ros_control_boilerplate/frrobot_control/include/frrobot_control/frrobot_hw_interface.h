#ifndef FRROBOT_CONTROL__FRROBOT_HW_INTERFACE_H
#define FRROBOT_CONTROL__FRROBOT_HW_INTERFACE_H

#include <ros_control_boilerplate/generic_hw_interface.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

namespace frrobot_control
{

  /// @brief Hardware interface for a robot
  class FrRobotHWInterface : public ros_control_boilerplate::GenericHWInterface
  {
    public:
      /**
       * @brief Constructor
       * @param nh - Node handle for topics.
       */
      FrRobotHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL);

      /** @brief Read the state from the robot hardware. */
      virtual void read(ros::Duration &elapsed_time);

      /** @brief Write the command to the robot hardware. */
      virtual void write(ros::Duration &elapsed_time);

      /** @brief Enforce limits for all values before writing */
      virtual void enforceLimits(ros::Duration &period);

    private:
      // 线程相关成员
      std::thread comm_thread_;
      std::mutex joints_mutex_;
      std::mutex socket_mutex_;
      std::atomic<bool> is_running_;
      std::atomic<bool> need_reconnect_;
      std::condition_variable cmd_cv_;
      bool has_new_command_;
      
      // 通信线程相关函数
      void communicationThread();
      void socketReconnect();
      bool sendCommand(const std::string& cmd, std::string& response);
      bool getJointPositions();
      
      // 存储当前关节状态和命令
      float cached_joint_positions_[6];
      float cached_joint_commands_[6];
      float a_, v_, t_, lat_, gain_;

  }; // class

} // namespace frrobot_control

#endif
