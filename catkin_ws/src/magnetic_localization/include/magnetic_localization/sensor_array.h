#pragma once

#include <Eigen/Dense>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <string>
#include <map>
#include <vector>
#include <memory>

struct SensorPose {
    Eigen::Vector3d position;
    Eigen::Vector3d orientation;  // RPY format
};

class SensorArray {
public:
    explicit SensorArray(ros::NodeHandle& nh);
    
    // 获取所有传感器ID
    std::vector<int> getSensorIds() const { return sensor_ids_; }
    
    // 获取传感器数量
    int getSensorCount() const { return sensor_count_; }

    // 根据ID获取传感器位置
    Eigen::Vector3d getSensorPosition(int id) const;
    
    // 根据ID获取传感器姿态(RPY)
    Eigen::Vector3d getSensorOrientation(int id) const;
    
    // 根据ID获取传感器完整位姿
    SensorPose getSensorPose(int id) const;
    
    // 检查传感器ID是否存在
    bool hasSensor(int id) const;

private:
    void loadFromYaml();
    
    ros::NodeHandle& nh_;
    std::map<int, SensorPose> sensor_poses_;    // ID到位姿的映射
    std::vector<int> sensor_ids_;               // 所有传感器ID列表
    int sensor_count_{0};
};