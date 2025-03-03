#include <ros/ros.h>
#include <std_msgs/String.h>
#include <frcobot_socket/RobotCommand.h>  // 假设我们定义了一个自定义消息类型
#include <frcobot_socket/RobotService.h>  // 假设我们定义了一个自定义服务类型

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <sstream>
#include <thread>
#include <mutex>
#include <vector>
#include <cstring>

class FRCobotSocket {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // ROS服务和话题
    ros::ServiceServer robot_service_;
    ros::Publisher robot_state_pub_;
    
    // 套接字相关变量
    int socket_fd_;
    struct sockaddr_in server_addr_;
    std::string robot_ip_;
    int robot_port_;
    bool is_connected_;
    
    // 线程相关
    std::thread recv_thread_;
    std::mutex socket_mutex_;
    bool running_;

public:
    FRCobotSocket() : private_nh_("~"), is_connected_(false), running_(false) {
        // 从参数服务器获取参数
        private_nh_.param<std::string>("robot_ip", robot_ip_, "192.168.31.202");
        private_nh_.param<int>("robot_port", robot_port_, 8080);
        
        // 初始化ROS服务和话题
        robot_service_ = nh_.advertiseService("frcobot_command", &FRCobotSocket::handleRobotCommand, this);
        robot_state_pub_ = nh_.advertise<std_msgs::String>("frcobot_state", 10);
        
        // 连接到机械臂
        if (connectToRobot()) {
            ROS_INFO("Successfully connected to robot at %s:%d", robot_ip_.c_str(), robot_port_);
            // 启动接收线程
            running_ = true;
            recv_thread_ = std::thread(&FRCobotSocket::receiveLoop, this);
        } else {
            ROS_ERROR("Failed to connect to robot at %s:%d", robot_ip_.c_str(), robot_port_);
        }
    }
    
    ~FRCobotSocket() {
        disconnect();
        if (recv_thread_.joinable()) {
            running_ = false;
            recv_thread_.join();
        }
    }
    
    bool connectToRobot() {
        // 创建套接字
        socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (socket_fd_ < 0) {
            ROS_ERROR("Failed to create socket");
            return false;
        }
        
        // 设置服务器地址
        memset(&server_addr_, 0, sizeof(server_addr_));
        server_addr_.sin_family = AF_INET;
        server_addr_.sin_port = htons(robot_port_);
        
        // 转换IP地址
        if (inet_pton(AF_INET, robot_ip_.c_str(), &server_addr_.sin_addr) <= 0) {
            ROS_ERROR("Invalid address/ Address not supported");
            close(socket_fd_);
            return false;
        }
        
        // 连接到服务器
        if (connect(socket_fd_, (struct sockaddr *)&server_addr_, sizeof(server_addr_)) < 0) {
            ROS_ERROR("Connection Failed");
            close(socket_fd_);
            return false;
        }
        
        is_connected_ = true;
        return true;
    }
    
    void disconnect() {
        if (is_connected_) {
            std::lock_guard<std::mutex> lock(socket_mutex_);
            close(socket_fd_);
            is_connected_ = false;
            ROS_INFO("Disconnected from robot");
        }
    }
    
    bool handleRobotCommand(frcobot_socket::RobotService::Request &req, 
                           frcobot_socket::RobotService::Response &res) {
        // 检查连接状态
        if (!is_connected_) {
            ROS_WARN("Not connected to robot");
            res.success = false;
            res.message = "Not connected to robot";
            return true;
        }
        
        // 发送命令到机械臂
        std::string command = req.command;
        bool send_success = sendToRobot(command);
        
        if (!send_success) {
            res.success = false;
            res.message = "Failed to send command to robot";
            return true;
        }
        
        // 等待机械臂响应（简化版，实际应该有超时机制和命令ID跟踪）
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        res.success = true;
        res.message = "Command sent successfully";
        return true;
    }
    
    bool sendToRobot(const std::string &message) {
        if (!is_connected_) {
            return false;
        }
        
        std::lock_guard<std::mutex> lock(socket_mutex_);
        int sent_bytes = send(socket_fd_, message.c_str(), message.length(), 0);
        if (sent_bytes < 0) {
            ROS_ERROR("Failed to send data to robot");
            is_connected_ = false;
            return false;
        }
        
        return true;
    }
    
    void receiveLoop() {
        char buffer[1024];
        int bytes_read;
        
        while (running_ && ros::ok()) {
            if (!is_connected_) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }
            
            memset(buffer, 0, sizeof(buffer));
            
            {
                std::lock_guard<std::mutex> lock(socket_mutex_);
                bytes_read = recv(socket_fd_, buffer, sizeof(buffer) - 1, 0);
            }
            
            if (bytes_read > 0) {
                // 处理接收到的数据
                std_msgs::String msg;
                msg.data = std::string(buffer, bytes_read);
                robot_state_pub_.publish(msg);
                ROS_DEBUG("Received from robot: %s", msg.data.c_str());
            } else if (bytes_read == 0) {
                // 连接关闭
                ROS_WARN("Connection closed by robot");
                is_connected_ = false;
                
                // 尝试重新连接
                std::this_thread::sleep_for(std::chrono::seconds(5));
                if (connectToRobot()) {
                    ROS_INFO("Reconnected to robot");
                }
            } else {
                // 接收错误
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    // 非阻塞模式下没有数据可读
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                } else {
                    ROS_ERROR("Socket receive error: %s", strerror(errno));
                    is_connected_ = false;
                    
                    // 尝试重新连接
                    std::this_thread::sleep_for(std::chrono::seconds(5));
                    if (connectToRobot()) {
                        ROS_INFO("Reconnected to robot");
                    }
                }
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "frcobot_socket_node");
    
    FRCobotSocket node;
    
    ros::spin();
    
    return 0;
}