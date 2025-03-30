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

#define MAXLINE 4096
#define PORT_CMD 8080

int confd;
int len;
int flag = 1;
float joints[6];
float a = 0.0;
float v = 0.0;
float t = 0.002;
float lat = 0.002;
float gain = 0.0;
float joints_sta[6] = {0, 0, 0, 0, 0, 0}; // 关节数组
socklen_t sendaddr_length;
char recvLine[MAXLINE];
char sendCmdLine[MAXLINE];
char sendStaLine[MAXLINE];
char recv_buf[MAXLINE];
char send_buf[MAXLINE - 50];
struct sockaddr_in serverSendAddr;

namespace frrobot_control
{

    FrRobotHWInterface::FrRobotHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
        : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
    {
        std::string robot_ip;
        if (!nh.getParam("robot_ip", robot_ip))
        {
            ROS_ERROR("Failed to get param 'robot_ip'");
            robot_ip = "192.168.31.202";
            ROS_WARN("Param 'robot_ip' not found, using default IP: %s", robot_ip.c_str());
        }

        memset(&serverSendAddr, 0, sizeof(serverSendAddr));
        serverSendAddr.sin_family = AF_INET;
        serverSendAddr.sin_addr.s_addr = inet_addr(robot_ip.c_str());
        serverSendAddr.sin_port = htons(PORT_CMD);

        if ((confd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
        {
            ROS_ERROR("Failed to create socket: %s", strerror(errno));
            exit(1);
        }

        // 设置 TCP keepalive
        int keepalive = 1;
        if (setsockopt(confd, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive)) < 0) {
            ROS_ERROR("Failed to set SO_KEEPALIVE: %s", strerror(errno));
        }

        // 设置发送和接收缓冲区大小
        int buffer_size = MAXLINE;
        if (setsockopt(confd, SOL_SOCKET, SO_SNDBUF, &buffer_size, sizeof(buffer_size)) < 0) {
            ROS_ERROR("Failed to set SO_SNDBUF: %s", strerror(errno));
        }
        if (setsockopt(confd, SOL_SOCKET, SO_RCVBUF, &buffer_size, sizeof(buffer_size)) < 0) {
            ROS_ERROR("Failed to set SO_RCVBUF: %s", strerror(errno));
        }

        // 设置接收超时
        struct timeval timeout;
        timeout.tv_sec = 2;
        timeout.tv_usec = 0;
        if (setsockopt(confd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0)
        {
            ROS_ERROR("Failed to set SO_RCVTIMEO: %s", strerror(errno));
            exit(1);
        }

        ROS_INFO("Connecting to robot at %s:%d", robot_ip.c_str(), PORT_CMD);
        if (connect(confd, (struct sockaddr *)&serverSendAddr, sizeof(serverSendAddr)) < 0)
        {
            ROS_ERROR("Failed to connect: %s", strerror(errno));
            exit(1);
        }

        ROS_INFO_NAMED("frrobot_hw_interface", "FrRobotHWInterface Ready.");
    }


    void FrRobotHWInterface::write(ros::Duration &elapsed_time)
    {
        enforceLimits(elapsed_time);

        // 清空发送缓冲区
        memset(send_buf, 0, sizeof(send_buf));
        memset(sendCmdLine, 0, sizeof(sendCmdLine));

        for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
        {
            joints[joint_id] = joint_position_command_[joint_id] * 180 / M_PI;
        }

        snprintf(send_buf, sizeof(send_buf), "ServoJ(%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f)", 
                joints[0], joints[1], joints[2], joints[3], joints[4], joints[5], 
                a, v, t, lat, gain);

        len = strlen(send_buf);
        snprintf(sendCmdLine, sizeof(sendCmdLine), "/f/bIII123III376III%dIII%sIII/b/f", len, send_buf);

        ssize_t send_length = send(confd, sendCmdLine, strlen(sendCmdLine), MSG_NOSIGNAL);
        if (send_length < 0)
        {
            ROS_ERROR("Send failed: %s", strerror(errno));
            return;
        }

        // 清空接收缓冲区
        memset(recvLine, 0, sizeof(recvLine));
        ssize_t recv_length = recv(confd, recvLine, sizeof(recvLine)-1, 0);
        if (recv_length < 0 && errno != EAGAIN)
        {
            ROS_ERROR("Receive failed: %s", strerror(errno));
            return;
        }
    }

    void FrRobotHWInterface::read(ros::Duration &elapsed_time)
    {
        // 清空发送缓冲区
        memset(sendStaLine, 0, sizeof(sendStaLine));
        snprintf(sendStaLine, sizeof(sendStaLine), "/f/bIII123III375III25IIIGetActualJointPosRadian()III/b/f");

        ssize_t send_length = send(confd, sendStaLine, strlen(sendStaLine), MSG_NOSIGNAL);
        if (send_length < 0)
        {
            ROS_ERROR("Send failed: %s", strerror(errno));
            return;
        }

        // 清空接收缓冲区
        memset(recvLine, 0, sizeof(recvLine));
        ssize_t recv_length = recv(confd, recvLine, sizeof(recvLine)-1, 0);
        if (recv_length > 0)
        {
            try {
                int pos = 0;
                int jointsDataLen = 0;
                std::string tempJoints = recvLine;
                for (int i = 0; i < 3; i++)
                {
                    pos = tempJoints.find("III") + 3;
                    tempJoints = tempJoints.substr(pos);
                }
                pos = tempJoints.find("III");
                jointsDataLen = std::stoi(tempJoints.substr(0, pos));
                tempJoints = tempJoints.substr(pos + 3, jointsDataLen);

                for (int i = 0; i < 6; i++)
                {
                    pos = tempJoints.find(",");
                    joints_sta[i] = std::stof(tempJoints.substr(0, pos));
                    tempJoints = tempJoints.substr(pos + 1);
                }

                for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
                {
                    joint_position_[joint_id] = joints_sta[joint_id];
                }
            } catch (const std::exception& e) {
                ROS_ERROR("Failed to parse joint data: %s", e.what());
            }
        }
        else if (recv_length < 0 && errno != EAGAIN)
        {
            ROS_ERROR("Receive failed: %s", strerror(errno));
            return;
        }
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
