#include <ros/ros.h>
#include <std_msgs/String.h>
#include <serial/serial.h>
#include <thread>
#include <vector>
#include <mutex>
#include <iostream>
#include <sstream>
#include <numeric>
#include <magnetic_localization/MagneticData.h>

class SerialHandler
{
public:
    /**
     * @brief 构造函数，初始化串口和ROS发布者
     * @param port 串口端口
     * @param baud_rate 波特率
     * @param sensor_N 传感器数量
     * @param compensation_mode 补偿模式
     */
    SerialHandler(const std::string &port, uint32_t baud_rate, int sensor_N, const std::string &compensation_mode)
        : sensor_N(sensor_N), compensation_mode(compensation_mode), latest_data(sensor_N, std::vector<double>(3, 0.0)),
          earth_magnetic_field(sensor_N, std::vector<double>(3, 0.0)), global_earth_magnetic_field(3, 0.0)
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

    /**
     * @brief 测量地磁场，根据补偿模式选择全局或个体补偿
     * @param num_cycles 测量周期数
     */
    void measureEarthMagneticField(int num_cycles = 10)
    {
        if (compensation_mode == "global")
        {
            measureGlobalEarthMagneticField(num_cycles);
        }
        else if (compensation_mode == "individual")
        {
            measureIndividualEarthMagneticField(num_cycles);
        }
        else if (compensation_mode == "raw")
        {
            // 保留原始数据，不进行任何补偿
        }
        else
        {
            throw std::runtime_error("Invalid compensation mode: " + compensation_mode); // 无效的补偿模式
        }
    }

private:
    ros::NodeHandle nh;
    ros::Publisher magnetic_data_pub;
    serial::Serial ser;                                    // 串口对象
    int sensor_N;                                          // 传感器数量
    std::string compensation_mode;                         // 补偿模式
    std::vector<std::vector<double>> latest_data;          // 最新数据
    std::vector<std::vector<double>> earth_magnetic_field; // 地磁场数据
    std::vector<double> global_earth_magnetic_field;       // 全局地磁场数据
    std::mutex data_mutex;                                 // 数据互斥锁
    std::thread read_thread;                               // 读取线程

    /**
     * @brief 将原始数据转换为高斯单位
     * @param raw_data 原始数据
     * @return 转换后的高斯数据
     */
    std::vector<double> convertToGauss(const std::vector<int> &raw_data)
    {
        std::vector<double> gauss_data(3);
        for (size_t i = 0; i < 3; ++i)
        {
            gauss_data[i] = (raw_data[i] / 32768.0) * 32.0; // 将原始数据转换为高斯单位
        }
        return gauss_data;
    }

    /**
     * @brief 更新最新数据
     * @param sensor_label 传感器标签
     * @param vector 数据向量
     */
    void updateLatestData(int sensor_label, const std::vector<double> &vector)
    {
        std::lock_guard<std::mutex> lock(data_mutex); // 加锁
        latest_data[sensor_label] = vector;           // 更新最新数据
    }

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
        int sensor_label = std::stoi(part.substr(1, 2)) - 1; // 解析传感器标签
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
                // ROS_INFO("Received: %s", line.c_str());
                try
                {
                    auto [sensor_label, raw_vector] = parseSerialLine(line); // 解析串口数据
                    if (sensor_label >= 0 && sensor_label < sensor_N)
                    {
                        std::vector<double> vector = convertToGauss(raw_vector); // 转换为高斯单位
                        if (compensation_mode == "global")
                        {
                            for (size_t i = 0; i < 3; ++i)
                            {
                                vector[i] -= global_earth_magnetic_field[i]; // 全局补偿
                            }
                        }
                        else if (compensation_mode == "individual")
                        {
                            for (size_t i = 0; i < 3; ++i)
                            {
                                vector[i] -= earth_magnetic_field[sensor_label][i]; // 个体补偿
                            }
                        }
                        else if (compensation_mode == "raw")
                        {
                            // 保留原始数据，不进行任何补偿
                        }
                        else
                        {
                            throw std::runtime_error("Invalid compensation mode: " + compensation_mode); // 无效的补偿模式
                        }
                        updateLatestData(sensor_label, vector); // 更新最新数据

                        // 发布磁场数据
                        magnetic_localization::MagneticData magnetic_msg;
                        magnetic_msg.header.stamp = ros::Time::now();
                        magnetic_msg.sensor_label = sensor_label;
                        magnetic_msg.x = raw_vector[0];
                        magnetic_msg.y = raw_vector[1];
                        magnetic_msg.z = raw_vector[2];
                        magnetic_data_pub.publish(magnetic_msg);
                    }
                }
                catch (const std::exception &e)
                {
                    ROS_ERROR("Error parsing line: %s -> %s", line.c_str(), e.what()); // 打印解析错误信息
                }
            }
        }
    }

    /**
     * @brief 测量全局地磁场
     * @param num_cycles 测量周期数
     */
    void measureGlobalEarthMagneticField(int num_cycles)
    {
        std::vector<double> magnetic_field_sum(3, 0.0);
        int total_samples = 0; // 用于记录有效采样总数

        for (int i = 0; i < num_cycles * sensor_N; ++i)
        {
            std::string line = ser.readline(65536, "\n"); // 从串口读取一行数据
            if (!line.empty())
            {
                try
                {
                    auto [sensor_label, raw_vector] = parseSerialLine(line); // 解析串口数据
                    if (sensor_label >= 0 && sensor_label < sensor_N)
                    {
                        std::vector<double> vector = convertToGauss(raw_vector); // 转换为高斯单位
                        magnetic_field_sum[0] += vector[0];
                        magnetic_field_sum[1] += vector[1];
                        magnetic_field_sum[2] += vector[2];
                        ++total_samples;
                    }
                }
                catch (const std::exception &e)
                {
                    ROS_ERROR("Error parsing line: %s -> %s", line.c_str(), e.what()); // 打印解析错误信息
                }
            }
        }

        if (total_samples > 0)
        {
            global_earth_magnetic_field[0] = magnetic_field_sum[0] / total_samples;
            global_earth_magnetic_field[1] = magnetic_field_sum[1] / total_samples;
            global_earth_magnetic_field[2] = magnetic_field_sum[2] / total_samples;
            ROS_INFO("Measured Global Earth Magnetic Field: [%f, %f, %f]",
                     global_earth_magnetic_field[0], global_earth_magnetic_field[1], global_earth_magnetic_field[2]);
        }
        else
        {
            ROS_WARN("No valid data received during global earth magnetic field measurement.");
        }
    }

    /**
     * @brief 测量个体地磁场
     * @param num_cycles 测量周期数
     */
    void measureIndividualEarthMagneticField(int num_cycles)
    {
        std::vector<std::vector<double>> magnetic_field_sum(sensor_N, std::vector<double>(3, 0.0));
        std::vector<int> count(sensor_N, 0); // 用于计数每个传感器的有效数据

        for (int i = 0; i < num_cycles * sensor_N; ++i)
        {
            std::string line = ser.readline(65536, "\n"); // 从串口读取一行数据
            if (!line.empty())
            {
                try
                {
                    auto [sensor_label, raw_vector] = parseSerialLine(line); // 解析串口数据
                    if (sensor_label >= 0 && sensor_label < sensor_N)
                    {
                        std::vector<double> vector = convertToGauss(raw_vector); // 转换为高斯单位
                        magnetic_field_sum[sensor_label][0] += vector[0];
                        magnetic_field_sum[sensor_label][1] += vector[1];
                        magnetic_field_sum[sensor_label][2] += vector[2];
                        ++count[sensor_label];
                    }
                }
                catch (const std::exception &e)
                {
                    ROS_ERROR("Error parsing line: %s -> %s", line.c_str(), e.what()); // 打印解析错误信息
                }
            }
        }

        for (int i = 0; i < sensor_N; ++i)
        {
            if (count[i] > 0)
            {
                earth_magnetic_field[i][0] = magnetic_field_sum[i][0] / count[i];
                earth_magnetic_field[i][1] = magnetic_field_sum[i][1] / count[i];
                earth_magnetic_field[i][2] = magnetic_field_sum[i][2] / count[i];
                ROS_INFO("Measured Earth Magnetic Field for Sensor %d: [%f, %f, %f]",
                         i + 1, earth_magnetic_field[i][0], earth_magnetic_field[i][1], earth_magnetic_field[i][2]);
            }
            else
            {
                ROS_WARN("No data received for Sensor %d", i + 1);
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
    int sensor_N;
    std::string compensation_mode;

    nh.param<std::string>("port", port, "/dev/ttyUSB0");
    nh.param<int>("baud_rate", baud_rate, 921600);
    nh.param<int>("sensor_N", sensor_N, 25);
    nh.param<std::string>("compensation_mode", compensation_mode, "individual");

    try
    {
        SerialHandler serial_handler(port, baud_rate, sensor_N, compensation_mode);
        serial_handler.measureEarthMagneticField(); // 测量地磁场
        serial_handler.startReading();
        ros::spin();
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Exception: %s", e.what());
    }

    return 0;
}