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
   Desc:   Records ros_control ControllerState data to CSV for Matlab/etc analysis
*/

#include <ros_control_boilerplate/tools/controller_to_csv.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller_to_csv_node");
    ROS_INFO_STREAM_NAMED("main", "Starting ControllerToCSV...");

    // 创建节点句柄
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // 从参数服务器读取参数
    std::string csv_path;
    std::string topic;
    
    // 获取参数,设置默认值
    private_nh.param<std::string>("csv_path", csv_path, "/tmp/recorded_trajectory.csv");
    private_nh.param<std::string>("topic", topic, "/robot2/robot2_controller/state");

    ROS_INFO_STREAM_NAMED("main", "CSV path: " << csv_path);
    ROS_INFO_STREAM_NAMED("main", "Recording topic: " << topic);

    // 使用多线程spinner
    ros::MultiThreadedSpinner spinner(2);

    // 创建转换器并开始记录
    ros_control_boilerplate::ControllerToCSV converter(topic);
    converter.startRecording(csv_path);

    ROS_INFO_STREAM_NAMED("main","Type Ctrl-C to end and save");
  
    // 使用 spinner.spin() 替代 ros::spin()
    spinner.spin();

    ROS_INFO_STREAM_NAMED("main", "Shutting down.");
    ros::shutdown();

  return 0;
}
