#include "magnetic_localization/sensor_array.h"

SensorArray::SensorArray(ros::NodeHandle& nh) : nh_(nh)
{
    loadFromYaml();
}

void SensorArray::loadFromYaml()
{
    std::string sensor_definition_file;
    nh_.param<std::string>("sensor_definition_file", sensor_definition_file, "sensor_definitions.yaml");

    YAML::Node config = YAML::LoadFile(sensor_definition_file);

    if (!config["sensors"]) {
        throw std::runtime_error("Sensor definition file does not contain 'sensors' key.");
    }

    // 清空现有数据
    sensor_poses_.clear();
    sensor_ids_.clear();

    // 读取传感器定义
    for (const auto& sensor : config["sensors"]) {
        if (!sensor["label"] || !sensor["position"]) {
            throw std::runtime_error("Invalid sensor definition format in YAML file.");
        }

        int label = sensor["label"].as<int>();
        auto position = sensor["position"].as<std::vector<double>>();
        
        if (position.size() != 3) {
            throw std::runtime_error("Position must have exactly 3 elements (x, y, z).");
        }

        // 读取姿态，如果未指定则默认为0
        std::vector<double> orientation = {0.0, 0.0, 0.0};
        if (sensor["orientation"]) {
            orientation = sensor["orientation"].as<std::vector<double>>();
            if (orientation.size() != 3) {
                throw std::runtime_error("Orientation must have exactly 3 elements (roll, pitch, yaw).");
            }
        }

        // 存储传感器位姿
        SensorPose pose;
        pose.position = Eigen::Vector3d(position[0], position[1], position[2]);
        pose.orientation = Eigen::Vector3d(orientation[0], orientation[1], orientation[2]);
        
        sensor_poses_[label] = pose;
        sensor_ids_.push_back(label);
    }

    sensor_count_ = sensor_poses_.size();
}

Eigen::Vector3d SensorArray::getSensorPosition(int id) const 
{
    if (!hasSensor(id)) {
        throw std::runtime_error("Sensor ID not found: " + std::to_string(id));
    }
    return sensor_poses_.at(id).position;
}

Eigen::Vector3d SensorArray::getSensorOrientation(int id) const 
{
    if (!hasSensor(id)) {
        throw std::runtime_error("Sensor ID not found: " + std::to_string(id));
    }
    return sensor_poses_.at(id).orientation;
}

SensorPose SensorArray::getSensorPose(int id) const 
{
    if (!hasSensor(id)) {
        throw std::runtime_error("Sensor ID not found: " + std::to_string(id));
    }
    return sensor_poses_.at(id);
}

bool SensorArray::hasSensor(int id) const 
{
    return sensor_poses_.find(id) != sensor_poses_.end();
}