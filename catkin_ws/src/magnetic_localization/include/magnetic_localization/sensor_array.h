#pragma once

#include <Eigen/Dense>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <string>
#include <map>
#include <memory>

class SensorArray {
public:
    explicit SensorArray(ros::NodeHandle& nh);
    
    // 获取传感器位置矩阵
    Eigen::MatrixXd getSensorPositions() const { return positions_; }
    
    // 获取传感器数量
    int getSensorCount() const { return sensor_count_; }

private:
    void loadFromYaml();
    
    ros::NodeHandle& nh_;
    Eigen::MatrixXd positions_;
    int sensor_count_{0};
};