#include "magnetic_localization/magnetic_field_solver.h"
#include <cmath>
#include <ros/ros.h>

/**
 * @brief 磁场求解器构造函数
 * 
 * @param sensor_positions Nx3矩阵，存储N个传感器的位置坐标 [x, y, z]
 */
MagnetFieldSolver::MagnetFieldSolver(const Eigen::MatrixXd &sensor_positions)
    : sensor_positions_(sensor_positions)
{
}

/**
 * @brief 计算磁偶极子在观测点产生的磁场
 * 
 * 使用磁偶极子模型计算给定位置和磁矩的磁体在观测点产生的磁场。
 * 计算公式: B = (μ0/4π) * [3(m·r)r/r^5 - m/r^3]
 * 其中:
 * - m: 磁矩向量
 * - r: 观测点相对于磁体位置的位移向量
 * - r: 位移向量的模
 * - μ0: 真空磁导率
 * 
 * @param position 磁体位置 [x, y, z]
 * @param moment 磁矩向量 [mx, my, mz]
 * @param observation_point 观测点位置 [x, y, z]
 * @return Eigen::Vector3d 磁场向量 [Bx, By, Bz]
 */
Eigen::Vector3d MagnetFieldSolver::calculateField(
    const Eigen::Vector3d &position,
    const Eigen::Vector3d &moment,
    const Eigen::Vector3d &observation_point) const
{

    Eigen::Vector3d r = observation_point - position;
    double r_norm = r.norm();
    double r_norm5 = std::pow(r_norm, 5);

    // 磁偶极子场的计算公式
    return (MU0 / (4 * M_PI)) *
           (3 * r * r.dot(moment) / r_norm5 - moment / (r_norm * r_norm * r_norm));
}

/**
 * @brief 求解磁体位置和参数
 * 
 * 使用Levenberg-Marquardt算法通过测量的磁场数据反向估算磁体的位置和参数。
 * 优化参数向量包含7个分量：[x, y, z, mx, my, mz, |m|]
 * 
 * @param measured_fields Nx3矩阵，N个传感器测量的磁场数据 [Bx, By, Bz]
 * @return MagnetSolution 求解结果，包含:
 *         - position: 磁体位置
 *         - orientation: 磁矩方向（单位向量）
 *         - moment: 磁矩大小
 *         - error: 拟合误差
 *         - success: 是否成功收敛
 */
MagnetSolution MagnetFieldSolver::solve(const Eigen::MatrixXd &measured_fields)
{
    MagnetSolution solution;
    const int max_iterations = 100;
    const double convergence_threshold = 1e-6;

    // 初始猜测值设置
    Eigen::VectorXd params(7); // [x, y, z, mx, my, mz, |m|]
    
    if (has_previous_solution_) {
        // 使用上一次的解作为初始值
        params = previous_solution_;
    } else {
        // 默认初始值
        params.setZero();
        params.segment<3>(0) = Eigen::Vector3d(0, 0, 1);    // 位置初始值
        params.segment<3>(3) = Eigen::Vector3d(0, 0, 1);    // 方向初始值
        params[6] = 3000.0;                                 // 磁矩大小初始值
    }

    // Levenberg-Marquardt 优化
    double lambda = 0.01;
    Eigen::VectorXd residual;
    Eigen::MatrixXd jacobian;

    for (int iter = 0; iter < max_iterations; ++iter)
    {
        calculateResidualAndJacobian(params, residual, jacobian, measured_fields);

        // 打印当前预测位置
        Eigen::Vector3d current_position = params.segment<3>(0);
        ROS_INFO_STREAM("Iteration " << iter 
                        << " Position: [" << current_position.x() 
                        << ", " << current_position.y() 
                        << ", " << current_position.z() << "]");

        Eigen::MatrixXd H = jacobian.transpose() * jacobian;
        H.diagonal() += lambda * H.diagonal();

        Eigen::VectorXd g = -jacobian.transpose() * residual;
        Eigen::VectorXd delta = H.ldlt().solve(g);

        if (delta.norm() < convergence_threshold)
        {
            solution.success = true;
            // 打印最终收敛位置
            ROS_INFO_STREAM("Converged! Final position: [" 
                            << current_position.x() 
                            << ", " << current_position.y() 
                            << ", " << current_position.z() << "]");
            break;
        }

        // 更新参数
        params += delta;
    }

    // 保存本次结果作为下次的初始值
    previous_solution_ = params;
    has_previous_solution_ = true;

    // 设置结果
    solution.position = params.segment<3>(0);
    solution.orientation = params.segment<3>(3).normalized();
    solution.moment = params[6];
    solution.error = residual.norm();

    return solution;
}

/**
 * @brief 计算残差向量和雅可比矩阵
 * 
 * 用于Levenberg-Marquardt优化算法的核心函数：
 * 1. 计算残差：预测磁场与测量磁场之差
 * 2. 数值计算雅可比矩阵：参数微小变化对预测磁场的影响
 * 
 * @param params 当前优化参数 [x, y, z, mx, my, mz, |m|]
 * @param residual 输出参数，存储计算的残差向量
 * @param jacobian 输出参数，存储计算的雅可比矩阵
 * @param measured_fields 测量的磁场数据
 * 
 * @note 雅可比矩阵使用数值差分方法计算，eps为差分步长
 */
void MagnetFieldSolver::calculateResidualAndJacobian(
    const Eigen::VectorXd &params,
    Eigen::VectorXd &residual,
    Eigen::MatrixXd &jacobian,
    const Eigen::MatrixXd &measured_fields) const
{
    const int n_sensors = sensor_positions_.rows();
    residual.resize(n_sensors * 3);
    jacobian.resize(n_sensors * 3, 7);

    // 添加参数有效性检查
    if (params.size() != 7) {
        ROS_ERROR("Invalid parameter vector size");
        return;
    }

    Eigen::Vector3d position = params.segment<3>(0);
    Eigen::Vector3d moment_dir = params.segment<3>(3);
    double moment_magnitude = std::abs(params[6]); // 确保磁矩大小为正
    
    // 检查磁矩方向向量是否为零向量
    double dir_norm = moment_dir.norm();
    if (dir_norm < 1e-10) {
        moment_dir = Eigen::Vector3d(0, 0, 1); // 设置默认方向
    } else {
        moment_dir.normalize();
    }
    
    Eigen::Vector3d moment = moment_dir * moment_magnitude;

    // 打印初始参数信息
    ROS_DEBUG_STREAM("Parameters: ");
    ROS_DEBUG_STREAM("Position: [" << position.transpose() << "]");
    ROS_DEBUG_STREAM("Moment direction: [" << moment_dir.transpose() << "]");
    ROS_DEBUG_STREAM("Moment magnitude: " << moment_magnitude);

    for (int i = 0; i < n_sensors; ++i) {
        Eigen::Vector3d sensor_pos = sensor_positions_.row(i);
        Eigen::Vector3d r = sensor_pos - position;
        
        // 打印传感器相关信息
        ROS_DEBUG_STREAM("Sensor " << i << ":");
        ROS_DEBUG_STREAM("  Position: [" << sensor_pos.transpose() << "]");
        ROS_DEBUG_STREAM("  Distance to magnet: " << r.norm());

        if (r.norm() < 1e-10) {
            ROS_WARN("Sensor and magnet positions are too close");
            r = Eigen::Vector3d(0, 0, 1e-10); // 添加小偏移
        }
        
        Eigen::Vector3d predicted_field = calculateField(position, moment, sensor_pos);
        Eigen::Vector3d measured_field = measured_fields.row(i);

        // 打印磁场信息
        ROS_DEBUG_STREAM("  Predicted field: [" << predicted_field.transpose() << "]");
        ROS_DEBUG_STREAM("  Measured field: [" << measured_field.transpose() << "]");
        ROS_DEBUG_STREAM("  Residual: [" << (predicted_field - measured_field).transpose() << "]");

        // 检查预测磁场是否有效
        if (!predicted_field.allFinite()) {
            ROS_ERROR("Invalid predicted field");
            continue;
        }

        residual.segment<3>(i * 3) = predicted_field - measured_field;

        // 数值计算雅可比矩阵时添加保护
        const double eps = 1e-7;
        for (int j = 0; j < 7; ++j) {
            Eigen::VectorXd params_perturbed = params;
            params_perturbed[j] += eps;

            Eigen::Vector3d position_p = params_perturbed.segment<3>(0);
            Eigen::Vector3d moment_dir_p = params_perturbed.segment<3>(3);
            
            // 确保扰动后的磁矩方向也是规范化的
            if (moment_dir_p.norm() > 1e-10) {
                moment_dir_p.normalize();
            } else {
                moment_dir_p = Eigen::Vector3d(0, 0, 1);
            }
            
            double moment_magnitude_p = std::abs(params_perturbed[6]);
            Eigen::Vector3d moment_p = moment_dir_p * moment_magnitude_p;

            Eigen::Vector3d field_perturbed = calculateField(position_p, moment_p, sensor_pos);
            
            // 检查计算结果是否有效
            if (field_perturbed.allFinite() && predicted_field.allFinite()) {
                jacobian.block<3, 1>(i * 3, j) = (field_perturbed - predicted_field) / eps;
            } else {
                jacobian.block<3, 1>(i * 3, j).setZero();
            }
        }
    }

    // 打印优化相关信息
    double res_norm = residual.norm();
    if (std::isfinite(res_norm)) {
        ROS_INFO("Residual norm: %f", res_norm);
    } else {
        ROS_ERROR("Invalid residual norm");
    }
    ROS_DEBUG_STREAM("Residual norm: " << res_norm);

}