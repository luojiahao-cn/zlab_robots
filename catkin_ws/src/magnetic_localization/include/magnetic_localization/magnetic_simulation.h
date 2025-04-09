#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>
#include "magnetic_localization/magnetic_field_solver.h"

class MagneticSimulation {
public:
    MagneticSimulation();
    void run();

private:
    // 常量定义
    const double INITIAL_MAGNETIC_SIZE = 13000.0;
    const int NUM_STEPS = 100;
    const int SENSOR_N = 5;
    const double SENSOR_INTERVAL = 1.0;
    const double ROTATION_ANGLE_PER_STEP = 5.0;  // 度
    
    // 成员变量
    Eigen::Vector3d initial_magnetic_direction_;
    Eigen::Vector3d start_position_;
    Eigen::Vector3d end_position_;
    std::unique_ptr<MagnetFieldSolver> solver_;
    
    // 辅助函数
    Eigen::MatrixXd initializeSensorPositions();
};