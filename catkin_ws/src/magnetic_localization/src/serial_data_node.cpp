#include <ros/ros.h>
#include <std_msgs/String.h>
#include <serial/serial.h>
#include <thread>
#include <vector>
#include <mutex>
#include <iostream>
#include <sstream>
#include <magnetic_localization/MagneticData.h>

class SerialHandler
{
public:
    /**
     * @brief 构造函数，初始化串口和ROS发布者
     * @param port 串口端口
     * @param baud_rate 波特率
     */
    SerialHandler(const std::string &port, uint32_t baud_rate)
    {
        try
        {
            ser.setPort(port);                                         // 设置串口端口
            ser.setBaudrate(baud_rate);                                // 设置波特率
            serial::Timeout to = serial::Timeout::simpleTimeout(1000); // 设置超时
            ser.setTimeout(to);                                        // 应用超时设置
            ser.open();                                                // 打开串口
            ROS_INFO("Connected to %s", port.c_str());                 // 打印连接成功信息
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR("Unable to open port %s", port.c_str()); // 打印错误信息
            throw e;                                           // 抛出异常
        }

        magnetic_data_pub = nh.advertise<magnetic_localization::MagneticData>("magnetic_data", 1000); // 发布磁场数据话题
    }

    /**
     * @brief 启动读取线程
     */
    void startReading()
    {
        read_thread = std::thread(&SerialHandler::readAndProcessData, this); // 启动读取线程
        read_thread.detach();                                                // 分离线程
    }

private:
    ros::NodeHandle nh;
    ros::Publisher magnetic_data_pub;
    serial::Serial ser;      // 串口对象
    std::mutex data_mutex;   // 数据互斥锁
    std::thread read_thread; // 读取线程

    /**
     * @brief 解析串口数据行
     * @param line 串口数据行
     * @return 传感器标签和原始数据向量
     */
    std::pair<int, std::vector<int>> parseSerialLine(const std::string &line)
    {
        std::istringstream iss(line);
        std::string part;
        std::getline(iss, part, ':');
        int sensor_label = std::stoi(part.substr(1, 2)); // 解析传感器标签
        std::vector<int> raw_vector(3);
        for (int &value : raw_vector)
        {
            iss >> value; // 读取原始数据
        }
        return {sensor_label, raw_vector}; // 返回传感器标签和原始数据向量
    }

    /**
     * @brief 读取和处理串口数据
     */
    void readAndProcessData()
    {
        while (ros::ok())
        {
            std::string line = ser.readline(65536, "\n"); // 从串口读取一行数据
            if (!line.empty())
            {
                try
                {
                    auto [sensor_label, raw_vector] = parseSerialLine(line); // 解析串口数据
                    // 发布磁场数据
                    magnetic_localization::MagneticData magnetic_msg;
                    magnetic_msg.header.stamp = ros::Time::now();
                    magnetic_msg.sensor_label = sensor_label;
                    magnetic_msg.x = raw_vector[0];
                    magnetic_msg.y = raw_vector[1];
                    magnetic_msg.z = raw_vector[2];
                    magnetic_data_pub.publish(magnetic_msg);
                }
                catch (const std::exception &e)
                {
                    ROS_ERROR("Error parsing line: %s -> %s", line.c_str(), e.what()); // 打印解析错误信息
                }
            }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_data_node");
    ros::NodeHandle nh;

    std::string port;
    int baud_rate;

    nh.param<std::string>("port", port, "/dev/ttyUSB0");
    nh.param<int>("baud_rate", baud_rate, 921600);

    try
    {
        SerialHandler serial_handler(port, baud_rate);
        serial_handler.startReading();
        ros::spin();
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Exception: %s", e.what());
    }

    return 0;
}