#pragma once

#include <Eigen/Dense>
#include <vector>

struct MagnetSolution {
    bool success;
    Eigen::Vector3d position;    // 磁体位置
    Eigen::Vector3d orientation; // 磁体方向
    double moment;              // 磁矩大小
    double error;              // 拟合误差
};

class MagnetFieldSolver {
public:
    explicit MagnetFieldSolver(const Eigen::MatrixXd& sensor_positions);

    // 计算给定位置和方向的磁偶极子产生的磁场
    Eigen::Vector3d calculateField(const Eigen::Vector3d& position,
                                 const Eigen::Vector3d& moment,
                                 const Eigen::Vector3d& observation_point) const;

    // 使用测量数据估算磁体位置和参数
    MagnetSolution solve(const Eigen::MatrixXd& measured_fields);

private:
    // 计算残差和雅可比矩阵
    void calculateResidualAndJacobian(const Eigen::VectorXd& params,
                                    Eigen::VectorXd& residual,
                                    Eigen::MatrixXd& jacobian,
                                    const Eigen::MatrixXd& measured_fields) const;

    Eigen::MatrixXd sensor_positions_; // 传感器位置矩阵
    static constexpr double MU0 = 1.25663706212e-6; // 真空磁导率
    Eigen::VectorXd previous_solution_;
    bool has_previous_solution_{false};
};