/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Records ros_control ControllerState datas to CSV for Matlab/etc analysis
*/

#include <ros_control_boilerplate/tools/csv_to_controller.h>

// Command line arguments
#include <gflags/gflags.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "csv_to_controller");
  ros::NodeHandle nh("~");  // 使用私有命名空间
  ROS_INFO_STREAM_NAMED("main", "Starting CSVToController...");

  // 从参数服务器读取参数
  std::string csv_path;
  std::string joint_trajectory_action;
  std::string controller_state_topic;

  // 获取参数，如果获取失败则使用默认值
  if (!nh.getParam("csv_path", csv_path))
  {
    ROS_ERROR_STREAM_NAMED("csv_to_controller", "No csv_path parameter provided");
    return 1;
  }

  if (!nh.getParam("joint_trajectory_action", joint_trajectory_action))
  {
    ROS_ERROR_STREAM_NAMED("csv_to_controller", "No joint_trajectory_action parameter provided");
    return 1;
  }

  if (!nh.getParam("controller_state_topic", controller_state_topic))
  {
    ROS_ERROR_STREAM_NAMED("csv_to_controller", "No controller_state_topic parameter provided");
    return 1;
  }

  ROS_INFO_STREAM_NAMED("csv_to_controller", "Reading from file " << csv_path);
  ROS_INFO_STREAM_NAMED("csv_to_controller", "Using action server: " << joint_trajectory_action);
  ROS_INFO_STREAM_NAMED("csv_to_controller", "Using controller state topic: " << controller_state_topic);

  // 允许动作服务器接收和发送ROS消息
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros_control_boilerplate::CSVToController converter(joint_trajectory_action, controller_state_topic);
  converter.loadAndRunCSV(csv_path);

  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::shutdown();

  return 0;
}
