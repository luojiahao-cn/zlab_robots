#include <ros/ros.h>
#include "magnetic_localization/magnetic_field_solver.h"
#include "magnetic_localization/MagneticData.h"
#include "magnetic_localization/MagnetEstimation.h"
#include <vector>
#include <map>
#include <yaml-cpp/yaml.h>
#include <fstream>

class MagneticFieldSolverNode
{
public:
    MagneticFieldSolverNode() : nh_("~")
    {
        // 初始化传感器布局和求解器
        solver_ = std::make_unique<MagnetFieldSolver>(initializeSensors());

        // 订阅磁场数据
        sub_ = nh_.subscribe("/magnetic/sensor_data", 100, &MagneticFieldSolverNode::callback, this);
        // 发布磁体位置估计结果
        pub_ = nh_.advertise<magnetic_localization::MagnetEstimation>("/magnetic/magnet_pose", 10);

        ROS_INFO("Starting background magnetic field calibration with 10 samples...");

        is_calibrating_ = true;
    }

private:
    
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    std::unique_ptr<MagnetFieldSolver> solver_;
    std::map<int, Eigen::Vector3d> sensor_data_; // 缓存传感器数据

    int sensor_count_;
    bool is_calibrating_{false};            // 是否正在进行校准
    int calibration_count_{0};              // 当前校准样本数量
    int required_calibration_samples_ = 10; // 需要收集的校准样本数量


    Eigen::MatrixXd initializeSensors()
    {
        // 定义文件路径
        std::string sensor_definition_file;
        nh_.param<std::string>("sensor_definition_file", sensor_definition_file, "sensor_definitions.yaml");

        // 加载 YAML 文件
        YAML::Node config = YAML::LoadFile(sensor_definition_file);

        // 检查是否包含传感器定义
        if (!config["sensors"])
        {
            throw std::runtime_error("Sensor definition file does not contain 'sensors' key.");
        }

        // 读取传感器定义
        const auto &sensors = config["sensors"];
        std::map<int, Eigen::Vector3d> sensor_map;

        for (const auto &sensor : sensors)
        {
            if (!sensor["label"] || !sensor["position"])
            {
                throw std::runtime_error("Invalid sensor definition format in YAML file.");
            }

            int label = sensor["label"].as<int>();
            auto position = sensor["position"].as<std::vector<double>>();

            if (position.size() != 3)
            {
                throw std::runtime_error("Position must have exactly 3 elements (x, y, z).");
            }

            // 存储传感器位置
            sensor_map[label] = Eigen::Vector3d(position[0], position[1], position[2]);
        }

        // 设置传感器数量
        sensor_count_ = sensor_map.size();

        // 将传感器位置映射到矩阵
        Eigen::MatrixXd positions(sensor_map.size(), 3);
        int row = 0;
        for (const auto &[label, position] : sensor_map)
        {
            positions.row(row++) = position;
        }

        return positions;
    }

    void callback(const magnetic_localization::MagneticData::ConstPtr &msg)
    {
        try
        {
            // 缓存传感器数据
            Eigen::Vector3d field(msg->x, msg->y, msg->z);
            sensor_data_[msg->sensor_label] = field;

            // 检查是否收集到一组完整的传感器数据
            if (sensor_data_.size() >= sensor_count_)
            {
                // 将 map 数据转换为矩阵形式
                Eigen::MatrixXd field_matrix(sensor_data_.size(), 3);
                int row = 0;
                for (const auto &[label, data] : sensor_data_)
                {
                    field_matrix.row(row++) = data;
                }

                if (is_calibrating_)
                {
                    // 收集校准数据
                    solver_->addCalibrationSample(field_matrix);
                    calibration_count_++;
                
                    if (calibration_count_ >= required_calibration_samples_)
                    {
                        ROS_INFO("Calibration sample collection complete, attempting to calibrate background magnetic field...");
                        // 尝试完成校准
                        if (solver_->calibrateInitialFields(required_calibration_samples_))
                        {
                            ROS_INFO("Background magnetic field calibration completed successfully");
                            is_calibrating_ = false;
                        }
                        else
                        {
                            ROS_WARN("Calibration failed, restarting calibration process...");
                            calibration_count_ = 0;
                        }
                    }
                }
                else
                {
                    // 正常的磁体定位求解
                    auto result = solver_->solve(field_matrix);
                    if (result.success)
                    {
                        // 发布结果
                        magnetic_localization::MagnetEstimation est;
                        est.header.stamp = ros::Time::now();
                        est.header.frame_id = "world";
                        est.position.x = result.position.x();
                        est.position.y = result.position.y();
                        est.position.z = result.position.z();
                        est.direction.x = result.orientation.x();
                        est.direction.y = result.orientation.y();
                        est.direction.z = result.orientation.z();
                        est.moment = result.moment;
                        pub_.publish(est);
                    }
                }

                // 清除数据缓存，准备下一次计算
                sensor_data_.clear();
            }
        }
        catch (const std::exception &e)
        {
            ROS_ERROR_STREAM("Error processing magnetic data: " << e.what());
        }
    }


};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "magnetic_solver");
    
    try
    {
        MagneticFieldSolverNode node;
        ros::spin();
    }
    catch (const std::exception &e)
    {
        ROS_FATAL_STREAM("Fatal error: " << e.what());
        return 1;
    }

    return 0;
}