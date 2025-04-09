#include "magnetic_localization/sensor_array.h"

SensorArray::SensorArray(ros::NodeHandle& nh) : nh_(nh)
{
    loadFromYaml();
}

void SensorArray::loadFromYaml()
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
    const auto& sensors = config["sensors"];
    std::map<int, Eigen::Vector3d> sensor_map;

    for (const auto& sensor : sensors)
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

        sensor_map[label] = Eigen::Vector3d(position[0], position[1], position[2]);
    }

    // 设置传感器数量和位置矩阵
    sensor_count_ = sensor_map.size();
    positions_.resize(sensor_count_, 3);

    int row = 0;
    for (const auto& [label, position] : sensor_map)
    {
        positions_.row(row++) = position;
    }
}