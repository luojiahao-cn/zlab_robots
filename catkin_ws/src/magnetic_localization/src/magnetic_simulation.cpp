#include "magnetic_localization/magnetic_simulation.h"
#include <ros/ros.h>
#include <cmath>

MagneticSimulation::MagneticSimulation()
{
    // 初始化磁矩方向和位置
    initial_magnetic_direction_ = Eigen::Vector3d(0, 0, -1);
    start_position_ = Eigen::Vector3d(0, 0, 2);
    end_position_ = Eigen::Vector3d(4, 4, 2);

    // 初始化传感器
    Eigen::MatrixXd sensor_positions = initializeSensorPositions();
    solver_ = std::make_unique<MagnetFieldSolver>(sensor_positions);
}

Eigen::MatrixXd MagneticSimulation::initializeSensorPositions()
{
    const int sensor_count = SENSOR_N * SENSOR_N;
    Eigen::MatrixXd positions(sensor_count, 3);

    for (int i = 0; i < SENSOR_N; ++i)
    {
        for (int j = 0; j < SENSOR_N; ++j)
        {
            int index = i * SENSOR_N + j;
            positions(index, 0) = j * SENSOR_INTERVAL;                  // x坐标
            positions(index, 1) = (SENSOR_N - 1 - i) * SENSOR_INTERVAL; // y坐标
            positions(index, 2) = 0;                                    // z坐标
        }
    }
    return positions;
}

void MagneticSimulation::run()
{
    Eigen::Vector3d current_direction = initial_magnetic_direction_;

    // 创建轨迹点
    for (int i = 0; i < NUM_STEPS; ++i)
    {
        double t = static_cast<double>(i) / (NUM_STEPS - 1);
        Eigen::Vector3d current_position = start_position_ + t * (end_position_ - start_position_);

        // 更新磁矩方向（绕X轴旋转）
        double angle = ROTATION_ANGLE_PER_STEP * M_PI / 180.0 * i;
        Eigen::Matrix3d rotation;
        rotation = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX());
        current_direction = rotation * initial_magnetic_direction_;

        // 计算观测磁场
        Eigen::MatrixXd observed_fields = solver_->calculateField(
            current_position,
            current_direction * INITIAL_MAGNETIC_SIZE,
            solver_->getSensorPositions());

        // 求解反问题
        MagnetSolution result = solver_->solve(observed_fields);

        ROS_INFO_THROTTLE(1.0, "Simulation progress: %d%%",
                          static_cast<int>(100.0 * i / NUM_STEPS));
    }

    ROS_INFO("Simulation completed");
}