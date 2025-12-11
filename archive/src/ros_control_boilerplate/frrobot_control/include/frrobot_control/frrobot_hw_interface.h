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
      // TCP通信相关
      std::string robot_ip_;
      int socket_fd_;
      
      // 运动参数
      float joints_[6];        // 关节命令值
      float joints_state_[6];  // 关节状态值
      float a_;                // 加速度
      float v_;                // 速度
      float t_;                // 周期
      float lat_;              // 滞后时间
      float gain_;             // 增益

  }; // class

} // namespace frrobot_control

#endif
