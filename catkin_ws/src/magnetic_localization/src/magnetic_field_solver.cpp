#include "magnetic_localization/magnetic_field_solver.h"
#include <cmath>
#include <ros/ros.h>

/**
 * @brief 磁场求解器构造函数
 * 
 * @param sensor_positions Nx3矩阵，存储N个传感器的位置坐标 [x, y, z]
 */
MagnetFieldSolver::MagnetFieldSolver(const Eigen::MatrixXd &sensor_positions)
    : sensor_positions_(sensor_positions), has_initial_fields_(false)
{
    // 初始化默认参数向量 [x, y, z, mx, my, mz, |m|]
    default_params_.resize(7);
    default_params_.setZero();
    default_params_.segment<3>(0) = Eigen::Vector3d(0, 0, 1); // 默认位置
    default_params_.segment<3>(3) = Eigen::Vector3d(0, 0, 1); // 默认方向
    default_params_[6] = 3000.0;                             // 默认磁矩大小
}

/**
 * @brief 设置初始磁场（如地磁场）
 * 
 * @param initial_fields Nx3矩阵，N个传感器位置处的背景磁场
 */
void MagnetFieldSolver::setInitialFields(const Eigen::MatrixXd &initial_fields)
{
    if (initial_fields.rows() != sensor_positions_.rows() || initial_fields.cols() != 3)
    {
        ROS_ERROR("Initial fields matrix dimensions do not match sensor positions");
        return;
    }
    initial_fields_ = initial_fields;
    has_initial_fields_ = true;
    ROS_INFO("Initial fields (earth magnetic field) set");
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
    const int max_iterations = 1000;
    const double convergence_threshold = 1e-6;

    // 1. 初始化优化参数向量 [x, y, z, mx, my, mz, |m|]
    Eigen::VectorXd params(7);
    params = has_previous_solution_ ? previous_solution_ : default_params_; // 使用上一次解或默认参数

    // 2. Levenberg-Marquardt 优化主循环
    double lambda = 0.01; // 阻尼因子
    Eigen::VectorXd residual;
    Eigen::MatrixXd jacobian;

    for (int iter = 0; iter < max_iterations; ++iter)
    {
        // 2.1 计算当前参数下的残差和雅可比矩阵
        calculateResidualAndJacobian(params, residual, jacobian, measured_fields);

        // 2.2 构造并求解LM方程
        Eigen::MatrixXd H = jacobian.transpose() * jacobian;
        H.diagonal() += lambda * H.diagonal();
        Eigen::VectorXd g = -jacobian.transpose() * residual;
        Eigen::VectorXd delta = H.ldlt().solve(g);

        // 2.3 检查收敛性
        if (delta.norm() < convergence_threshold)
        {
            solution.success = true;
            break;
        }

        // 2.4 更新参数
        params += delta;
    }

    // 3. 保存结果
    previous_solution_ = params;
    has_previous_solution_ = false;

    // 4. 设置返回结果
    solution.position = params.segment<3>(0);
    solution.orientation = params.segment<3>(3).normalized(); 
    solution.moment = params[6];
    solution.error = residual.norm();

    ROS_INFO("Iteration: %d, Error: %f", max_iterations, solution.error);

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
    // 1. 初始化残差向量和雅可比矩阵
    const int n_sensors = sensor_positions_.rows();
    residual.resize(n_sensors * 3);
    jacobian.resize(n_sensors * 3, 7);

    if (params.size() != 7)
    {
        ROS_ERROR("Invalid parameter vector size");
        return;
    }

    // 2. 提取当前参数
    Eigen::Vector3d position = params.segment<3>(0);
    Eigen::Vector3d moment_dir = params.segment<3>(3);
    double moment_magnitude = std::abs(params[6]);

    // 规范化磁矩方向
    moment_dir = moment_dir.norm() < 1e-10 ? Eigen::Vector3d(0, 0, 1) : moment_dir.normalized();
    Eigen::Vector3d moment = moment_dir * moment_magnitude;

    // 3. 计算每个传感器的残差和雅可比矩阵
    const double eps = 1e-7; // 数值微分步长
    for (int i = 0; i < n_sensors; ++i)
    {
        // 3.1 计算传感器到磁体的位移向量
        Eigen::Vector3d sensor_pos = sensor_positions_.row(i);
        Eigen::Vector3d r = sensor_pos - position;
        if (r.norm() < 1e-10)
        {
            r = Eigen::Vector3d(0, 0, 1e-10);
        }

        // 3.2 计算预测磁场和残差
        Eigen::Vector3d predicted_field = calculateField(position, moment, sensor_pos);
        if (!predicted_field.allFinite())  
            continue;

        Eigen::Vector3d measured_field = measured_fields.row(i);
        // 减去地磁场的影响
        if (has_initial_fields_) {
            measured_field -= initial_fields_.row(i);
        }
        residual.segment<3>(i * 3) = predicted_field - measured_field;

        // 3.3 数值计算雅可比矩阵
        for (int j = 0; j < 7; ++j)
        {
            // 对参数进行扰动
            Eigen::VectorXd params_perturbed = params;
            params_perturbed[j] += eps;

            // 计算扰动后的磁矩
            Eigen::Vector3d position_p = params_perturbed.segment<3>(0);
            Eigen::Vector3d moment_dir_p = params_perturbed.segment<3>(3);
            moment_dir_p = moment_dir_p.norm() > 1e-10 ? moment_dir_p.normalized() : Eigen::Vector3d(0, 0, 1);
            double moment_magnitude_p = std::abs(params_perturbed[6]);
            Eigen::Vector3d moment_p = moment_dir_p * moment_magnitude_p;

            // 计算扰动后的磁场和雅可比矩阵元素
            Eigen::Vector3d field_perturbed = calculateField(position_p, moment_p, sensor_pos);
            if (field_perturbed.allFinite())
            {
                jacobian.block<3, 1>(i * 3, j) = (field_perturbed - predicted_field) / eps;
            }
            else
            {
                jacobian.block<3, 1>(i * 3, j).setZero();
            }
        }
    }
}

/** 
* @brief 通过前N次测量值计算背景磁场
* 
* 计算平均值并设置为初始磁场。
* 
* @param n_samples 采样次数
* @return 是否成功设置背景磁场  
*/
bool MagnetFieldSolver::calibrateInitialFields(int n_samples)
{
    if (calibration_samples_.empty() || calibration_samples_.size() < n_samples) {
        ROS_WARN("Not enough calibration samples");
        return false;
    }

    // 计算平均值
    Eigen::MatrixXd avg_fields = Eigen::MatrixXd::Zero(sensor_positions_.rows(), 3);
    for (const auto& sample : calibration_samples_) {
        avg_fields += sample;
    }
    avg_fields /= calibration_samples_.size();

    // 设置为初始磁场
    setInitialFields(avg_fields);
    ROS_INFO("Background magnetic field calibrated using %d samples", (int)calibration_samples_.size());

    // 清空校准数据
    calibration_samples_.clear();
    is_calibrating_ = false;
    
    return true;
}

/**
 * @brief 添加校准样本
 * 
 * @param sample 校准样本
 */
void MagnetFieldSolver::addCalibrationSample(const Eigen::MatrixXd &sample) 
{
    if (sample.rows() != sensor_positions_.rows() || sample.cols() != 3) {
        ROS_ERROR("Invalid calibration sample dimensions");
        return;
    }
    calibration_samples_.push_back(sample);
}