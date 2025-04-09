#pragma once

#include <Eigen/Dense>
#include <vector>

/**
 * @brief 磁体定位求解结果结构体
 */
struct MagnetSolution
{
    bool success;                // 是否成功收敛
    Eigen::Vector3d position;    // 磁体位置 [x, y, z]
    Eigen::Vector3d orientation; // 磁体方向（单位向量）
    double moment;               // 磁矩大小
    double error;                // 拟合误差
};

/**
 * @brief 磁场求解器类
 *
 * 用于求解磁偶极子的位置和方向。使用Levenberg-Marquardt算法
 * 通过多个传感器测量的磁场数据反向估算磁体的位置和参数。
 */
class MagnetFieldSolver
{
public:
    // 物理常量
    static constexpr double MU0 = 1.25663706212e-6; // 真空磁导率 (T⋅m/A)

    /**
     * @brief 构造函数
     * @param sensor_positions Nx3矩阵，存储N个传感器的位置坐标 [x, y, z]
     */
    explicit MagnetFieldSolver(const Eigen::MatrixXd &sensor_positions);

    /**
     * @brief 设置初始磁场（如地磁场）
     * @param initial_fields Nx3矩阵，N个传感器位置处的背景磁场
     */
    void setInitialFields(const Eigen::MatrixXd &initial_fields);

    /**
     * @brief 计算磁偶极子在观测点产生的磁场
     * @param position 磁体位置 [x, y, z]
     * @param moment 磁矩向量 [mx, my, mz]
     * @param observation_point 观测点位置 [x, y, z]
     * @return 磁场向量 [Bx, By, Bz]
     */
    Eigen::Vector3d calculateField(
        const Eigen::Vector3d &position,
        const Eigen::Vector3d &moment,
        const Eigen::Vector3d &observation_point) const;

    /**
     * @brief 求解磁体位置和参数
     * @param measured_fields Nx3矩阵，N个传感器测量的磁场数据
     * @return 求解结果，包含位置、方向、磁矩大小和误差
     */
    MagnetSolution solve(const Eigen::MatrixXd &measured_fields);

    /**
     * @brief 通过前N次测量值计算背景磁场
     * @param n_samples 采样次数
     * @return 是否成功设置背景磁场
     */
    bool calibrateInitialFields(int n_samples = 10);

    /**
     * @brief 添加一次磁场测量数据用于校准
     * @param fields 一次磁场测量数据
     */
    void addCalibrationSample(const Eigen::MatrixXd& fields);

private:
    /**
     * @brief 计算残差向量和雅可比矩阵
     * @param params 当前优化参数 [x, y, z, mx, my, mz, |m|]
     * @param residual 输出参数，存储计算的残差向量
     * @param jacobian 输出参数，存储计算的雅可比矩阵
     * @param measured_fields 测量的磁场数据
     */
    void calculateResidualAndJacobian(
        const Eigen::VectorXd &params,
        Eigen::VectorXd &residual,
        Eigen::MatrixXd &jacobian,
        const Eigen::MatrixXd &measured_fields) const;

    // 成员变量
    Eigen::MatrixXd sensor_positions_;  // 传感器位置矩阵 (Nx3)
    Eigen::MatrixXd initial_fields_;    // 初始磁场值 (Nx3)
    Eigen::VectorXd previous_solution_; // 上一次求解结果
    Eigen::VectorXd default_params_;    // 默认参数向量

    std::vector<Eigen::MatrixXd> calibration_samples_;  // 用于校准的采样数据
    bool is_calibrating_{false};                        // 是否正在校准

    // 状态标志
    bool has_initial_fields_{false};    // 是否设置了初始磁场
    bool has_previous_solution_{false}; // 是否有上一次求解结果
};