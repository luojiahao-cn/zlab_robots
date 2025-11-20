#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <algorithm>  // for std::reverse
#include <geometry_msgs/Pose.h>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>

/**
 * @brief 移动到指定的命名状态
 * @param group MoveGroupInterface对象
 * @param state_name 命名状态名称（如"ready", "up"等）
 * @param velocity_scaling_factor 速度缩放因子 (0.0-1.0)
 * @return 是否成功
 */
bool moveToNamedState(moveit::planning_interface::MoveGroupInterface &group, 
                      const std::string &state_name, 
                      double velocity_scaling_factor = 0.5)
{
    ROS_INFO("Moving to named state: %s", state_name.c_str());
    group.setMaxVelocityScalingFactor(velocity_scaling_factor);
    group.setNamedTarget(state_name);
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    if (success)
    {
        group.move();
        ROS_INFO("Successfully moved to state: %s", state_name.c_str());
        return true;
    }
    else
    {
        ROS_ERROR("Failed to plan or move to state: %s", state_name.c_str());
        return false;
    }
}

/**
 * @brief 移动到指定的关节配置
 * @param group MoveGroupInterface对象
 * @param joint_values 关节角度值向量（弧度）
 * @param velocity_scaling_factor 速度缩放因子 (0.0-1.0)
 * @return 是否成功
 */
bool moveToJointConfig(moveit::planning_interface::MoveGroupInterface &group,
                       const std::vector<double> &joint_values,
                       double velocity_scaling_factor = 0.5)
{
    ROS_INFO("Moving to joint configuration");
    group.setMaxVelocityScalingFactor(velocity_scaling_factor);
    group.setJointValueTarget(joint_values);
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    if (success)
    {
        group.move();
        ROS_INFO("Successfully moved to joint configuration");
        return true;
    }
    else
    {
        ROS_ERROR("Failed to plan or move to joint configuration");
        return false;
    }
}

/**
 * @brief 获取初始关节配置
 * @param group MoveGroupInterface对象
 * @return 初始关节配置值向量（弧度）
 */
std::vector<double> getInitialJointConfig(moveit::planning_interface::MoveGroupInterface &group)
{
    // 初始关节配置（度）：[1.982, -88.878, -96.52, -83.606, 90.505, 0.083]
    // 转换为弧度
    std::vector<double> initial_config_deg = {1.982, -88.878, -96.52, -83.606, 90.505, 0.083};
    std::vector<double> initial_config;
    
    for (size_t i = 0; i < initial_config_deg.size(); ++i)
    {
        initial_config.push_back(initial_config_deg[i] * M_PI / 180.0);
    }
    
    ROS_INFO("Initial joint configuration (degrees): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
             initial_config_deg[0], initial_config_deg[1], initial_config_deg[2],
             initial_config_deg[3], initial_config_deg[4], initial_config_deg[5]);
    ROS_INFO("Initial joint configuration (radians): [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
             initial_config[0], initial_config[1], initial_config[2],
             initial_config[3], initial_config[4], initial_config[5]);
    
    return initial_config;
}

/**
 * @brief 动作A：让最后一个关节（frrobot_j6）旋转170度
 * @param group MoveGroupInterface对象
 * @return 是否成功
 */
bool executeActionA(moveit::planning_interface::MoveGroupInterface &group)
{
    ROS_INFO("========== Executing Action A: Rotating last joint by 170 degrees ==========");
    
    // 获取当前关节值
    std::vector<double> current_joint_values = group.getCurrentJointValues();
    if (current_joint_values.size() < 6)
    {
        ROS_ERROR("Failed to get current joint values or insufficient joints!");
        return false;
    }
    
    // 保存当前关节值
    std::vector<double> target_joint_values = current_joint_values;
    
    // 最后一个关节（索引5，frrobot_j6）旋转170度
    // 170度转换为弧度：170 * π / 180 ≈ 2.96706 弧度
    const double rotation_angle_rad = 170.0 * M_PI / 180.0;
    target_joint_values[5] += rotation_angle_rad;
    
    ROS_INFO("Current joint 6 value: %.4f rad (%.2f deg)", 
             current_joint_values[5], current_joint_values[5] * 180.0 / M_PI);
    ROS_INFO("Target joint 6 value: %.4f rad (%.2f deg)", 
             target_joint_values[5], target_joint_values[5] * 180.0 / M_PI);
    
    // 移动到目标关节配置
    return moveToJointConfig(group, target_joint_values, 0.3);
}

/**
 * @brief 动作A的逆动作：让最后一个关节（frrobot_j6）反向旋转170度
 * @param group MoveGroupInterface对象
 * @return 是否成功
 */
bool executeActionAInverse(moveit::planning_interface::MoveGroupInterface &group)
{
    ROS_INFO("========== Executing Action A Inverse: Rotating last joint by -170 degrees ==========");
    
    // 获取当前关节值
    std::vector<double> current_joint_values = group.getCurrentJointValues();
    if (current_joint_values.size() < 6)
    {
        ROS_ERROR("Failed to get current joint values or insufficient joints!");
        return false;
    }
    
    // 保存当前关节值
    std::vector<double> target_joint_values = current_joint_values;
    
    // 最后一个关节（索引5，frrobot_j6）反向旋转170度
    // 170度转换为弧度：170 * π / 180 ≈ 2.96706 弧度
    const double rotation_angle_rad = 170.0 * M_PI / 180.0;
    target_joint_values[5] -= rotation_angle_rad;
    
    ROS_INFO("Current joint 6 value: %.4f rad (%.2f deg)", 
             current_joint_values[5], current_joint_values[5] * 180.0 / M_PI);
    ROS_INFO("Target joint 6 value: %.4f rad (%.2f deg)", 
             target_joint_values[5], target_joint_values[5] * 180.0 / M_PI);
    
    // 移动到目标关节配置
    return moveToJointConfig(group, target_joint_values, 0.3);
}

/**
 * @brief 生成圆弧路径点，y轴指向圆心，x轴沿切线方向
 * @param start_pose 起始位姿
 * @param end_pose 结束位姿
 * @param center 圆心位置
 * @param num_points 路径点数量
 * @param reverse 是否反向（从终点到起点）
 * @return 路径点序列
 */
std::vector<geometry_msgs::Pose> generateArcWaypointsWithOrientation(
    const geometry_msgs::Pose& start_pose,
    const geometry_msgs::Pose& end_pose,
    const Eigen::Vector3d& center,
    int num_points = 50,
    bool reverse = false)
{
    std::vector<geometry_msgs::Pose> waypoints;
    
    // 起点和终点位置
    Eigen::Vector3d start_pos(start_pose.position.x, start_pose.position.y, start_pose.position.z);
    Eigen::Vector3d end_pos(end_pose.position.x, end_pose.position.y, end_pose.position.z);
    
    // 计算从圆心到起点和终点的向量
    Eigen::Vector3d vec_to_start = start_pos - center;
    Eigen::Vector3d vec_to_end = end_pos - center;
    
    // 计算半径（应该相等）
    double radius_start = vec_to_start.norm();
    double radius_end = vec_to_end.norm();
    double radius = (radius_start + radius_end) / 2.0;
    
    // 计算角度
    double start_angle = atan2(vec_to_start.y(), vec_to_start.x());
    double end_angle = atan2(vec_to_end.y(), vec_to_end.x());
    
    // 如果反向，交换起点和终点角度
    if (reverse)
    {
        std::swap(start_angle, end_angle);
    }
    
    // 处理角度跨越问题（确保选择较短的路径）
    double angle_diff = end_angle - start_angle;
    if (angle_diff > M_PI)
        angle_diff -= 2 * M_PI;
    else if (angle_diff < -M_PI)
        angle_diff += 2 * M_PI;
    
    ROS_INFO("Arc parameters: center=[%.4f, %.4f, %.4f], radius=%.4f, angle_range=[%.4f, %.4f], diff=%.4f",
             center.x(), center.y(), center.z(), radius, start_angle, end_angle, angle_diff);
    
    // 生成路径点
    for (int i = 0; i <= num_points; ++i)
    {
        double t = static_cast<double>(i) / num_points;
        double angle = start_angle + t * angle_diff;
        
        // 计算位置（圆弧上的点）
        geometry_msgs::Pose pose;
        pose.position.x = center.x() + radius * cos(angle);
        pose.position.y = center.y() + radius * sin(angle);
        pose.position.z = center.z();
        
        // 计算姿态：y轴指向圆心，x轴沿切线方向
        // 从当前位置指向圆心的向量（y轴负方向）
        Eigen::Vector3d to_center = center - Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
        to_center.normalize();
        
        // 切线方向（x轴正方向）：垂直于半径方向，在xy平面内
        // 切线方向与半径方向垂直，在xy平面内
        Eigen::Vector3d tangent(-sin(angle), cos(angle), 0.0);
        if (reverse)
        {
            tangent = -tangent; // 反向时切线方向相反
        }
        tangent.normalize();
        
        // z轴：垂直于xy平面（保持原有z方向）
        Eigen::Vector3d z_axis(0.0, 0.0, 1.0);
        
        // y轴：指向圆心（负方向）
        Eigen::Vector3d y_axis = -to_center;
        
        // x轴：切线方向
        Eigen::Vector3d x_axis = tangent;
        
        // 确保坐标系正交
        y_axis = y_axis - y_axis.dot(x_axis) * x_axis;
        y_axis.normalize();
        z_axis = x_axis.cross(y_axis);
        z_axis.normalize();
        
        // 构建旋转矩阵并转换为四元数
        Eigen::Matrix3d rotation_matrix;
        rotation_matrix.col(0) = x_axis;
        rotation_matrix.col(1) = y_axis;
        rotation_matrix.col(2) = z_axis;
        
        Eigen::Quaterniond quat(rotation_matrix);
        quat.normalize();
        
        pose.orientation.x = quat.x();
        pose.orientation.y = quat.y();
        pose.orientation.z = quat.z();
        pose.orientation.w = quat.w();
        
        waypoints.push_back(pose);
    }
    
    return waypoints;
}

// 全局变量保存动作B的圆心和终点位置（用于逆动作）
static Eigen::Vector3d action_b_center;
static geometry_msgs::Pose action_b_end_pose;
static geometry_msgs::Pose action_b_start_pose;
// 全局变量保存动作B执行后的j6值（用于动作C的逆动作）
static double action_b_end_j6_value = 0.0;
// 全局变量保存动作B执行后的完整关节配置（用于动作B逆动作前的恢复）
static std::vector<double> action_b_end_joint_config;
// 全局变量保存动作B的轨迹（用于逆动作反向执行）
static moveit_msgs::RobotTrajectory action_b_trajectory;
static bool action_b_trajectory_saved = false;

/**
 * @brief 动作B：pm_tcp_link沿圆弧移动，x增加180mm，y增加180mm，z不变
 *         y轴指向圆心，x轴沿切线方向
 *         圆心位置：x + 180mm（相对于起点）
 * @param group MoveGroupInterface对象
 * @return 是否成功
 */
bool executeActionB(moveit::planning_interface::MoveGroupInterface &group)
{
    ROS_INFO("========== Executing Action B: Arc movement with orientation ==========");
    
    // 使用默认的末端执行器链接（不需要强制设置pm_tcp_link）
    std::string eef_link = group.getEndEffectorLink();
    ROS_INFO("Using end effector link: %s", eef_link.c_str());
    
    // 获取当前末端执行器的位姿
    geometry_msgs::PoseStamped current_pose_stamped = group.getCurrentPose();
    action_b_start_pose = current_pose_stamped.pose;
    
    ROS_INFO("Start pose: position=[%.4f, %.4f, %.4f], orientation=[%.4f, %.4f, %.4f, %.4f]",
             action_b_start_pose.position.x, action_b_start_pose.position.y, action_b_start_pose.position.z,
             action_b_start_pose.orientation.x, action_b_start_pose.orientation.y, 
             action_b_start_pose.orientation.z, action_b_start_pose.orientation.w);
    
    // 计算终点位置（x增加180mm，y增加180mm，z不变）
    const double delta_x = 0.18;  // 180mm
    const double delta_y = 0.18;  // 180mm
    
    action_b_end_pose = action_b_start_pose;
    action_b_end_pose.position.x += delta_x;
    action_b_end_pose.position.y += delta_y;
    
    // 圆心位置：x + 180mm（相对于起点），y和z与起点相同
    action_b_center = Eigen::Vector3d(
        action_b_start_pose.position.x + delta_x,
        action_b_start_pose.position.y,
        action_b_start_pose.position.z
    );
    
    ROS_INFO("End pose: position=[%.4f, %.4f, %.4f]",
             action_b_end_pose.position.x, action_b_end_pose.position.y, action_b_end_pose.position.z);
    ROS_INFO("Arc center: [%.4f, %.4f, %.4f]",
             action_b_center.x(), action_b_center.y(), action_b_center.z());
    
    // 生成圆弧路径点
    const int num_points = 50;
    std::vector<geometry_msgs::Pose> waypoints = generateArcWaypointsWithOrientation(
        action_b_start_pose, action_b_end_pose, action_b_center, num_points, false);
    
    ROS_INFO("Generated %zu waypoints for arc path", waypoints.size());
    
    // 计算笛卡尔路径
    moveit_msgs::RobotTrajectory trajectory;
    const double eef_step = 0.005; // 5mm步长，保证路径平滑
    double fraction = group.computeCartesianPath(waypoints, eef_step, trajectory);
    
    ROS_INFO("Path planning fraction: %.2f%%", fraction * 100.0);
    
    if (fraction < 0.9)
    {
        ROS_ERROR("Path planning failed, only %.2f%% of the path was completed", fraction * 100.0);
        return false;
    }
    
    // 保存轨迹供逆动作使用
    action_b_trajectory = trajectory;
    action_b_trajectory_saved = true;
    ROS_INFO("Saved Action B trajectory with %zu waypoints", trajectory.joint_trajectory.points.size());
    ROS_INFO("Trajectory joint names count: %zu", trajectory.joint_trajectory.joint_names.size());
    if (!trajectory.joint_trajectory.points.empty())
    {
        ROS_INFO("First saved point has %zu positions, %zu velocities, %zu accelerations",
                 trajectory.joint_trajectory.points[0].positions.size(),
                 trajectory.joint_trajectory.points[0].velocities.size(),
                 trajectory.joint_trajectory.points[0].accelerations.size());
        ROS_INFO("First saved point time_from_start: %.4f seconds",
                 trajectory.joint_trajectory.points[0].time_from_start.toSec());
        ROS_INFO("Last saved point time_from_start: %.4f seconds",
                 trajectory.joint_trajectory.points.back().time_from_start.toSec());
    }
    
    // 执行轨迹
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    group.setMaxVelocityScalingFactor(0.3);
    
    bool success = (group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    if (success)
    {
        ROS_INFO("Successfully executed Action B arc movement");
        
        // 保存动作B执行后的完整关节配置，供动作B逆动作前的恢复使用
        std::vector<double> current_joint_values = group.getCurrentJointValues();
        if (current_joint_values.size() >= 6)
        {
            action_b_end_joint_config = current_joint_values;
            action_b_end_j6_value = current_joint_values[5];
            ROS_INFO("Saved joint config after Action B:");
            ROS_INFO("  j6 value: %.4f rad (%.2f deg)", 
                     action_b_end_j6_value, action_b_end_j6_value * 180.0 / M_PI);
        }
    }
    else
    {
        ROS_ERROR("Failed to execute Action B arc movement");
        action_b_trajectory_saved = false;
    }
    
    return success;
}

/**
 * @brief 动作B的逆动作：反向执行动作B保存的轨迹
 * @param group MoveGroupInterface对象
 * @return 是否成功
 */
bool executeActionBInverse(moveit::planning_interface::MoveGroupInterface &group)
{
    ROS_INFO("========== Executing Action B Inverse: Reversing saved Action B trajectory ==========");
    
    // 检查是否保存了动作B的轨迹
    if (!action_b_trajectory_saved || action_b_trajectory.joint_trajectory.points.empty())
    {
        ROS_ERROR("Action B trajectory not saved! Cannot execute inverse.");
        ROS_ERROR("  action_b_trajectory_saved: %d", action_b_trajectory_saved);
        ROS_ERROR("  trajectory points size: %zu", action_b_trajectory.joint_trajectory.points.size());
        return false;
    }
    
    ROS_INFO("Original trajectory has %zu points", action_b_trajectory.joint_trajectory.points.size());
    if (!action_b_trajectory.joint_trajectory.points.empty())
    {
        ROS_INFO("First point positions: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                 action_b_trajectory.joint_trajectory.points[0].positions[0],
                 action_b_trajectory.joint_trajectory.points[0].positions[1],
                 action_b_trajectory.joint_trajectory.points[0].positions[2],
                 action_b_trajectory.joint_trajectory.points[0].positions[3],
                 action_b_trajectory.joint_trajectory.points[0].positions[4],
                 action_b_trajectory.joint_trajectory.points[0].positions[5]);
        size_t last_idx = action_b_trajectory.joint_trajectory.points.size() - 1;
        ROS_INFO("Last point positions: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                 action_b_trajectory.joint_trajectory.points[last_idx].positions[0],
                 action_b_trajectory.joint_trajectory.points[last_idx].positions[1],
                 action_b_trajectory.joint_trajectory.points[last_idx].positions[2],
                 action_b_trajectory.joint_trajectory.points[last_idx].positions[3],
                 action_b_trajectory.joint_trajectory.points[last_idx].positions[4],
                 action_b_trajectory.joint_trajectory.points[last_idx].positions[5]);
    }
    
    // 获取当前关节位置
    std::vector<double> current_joint_values = group.getCurrentJointValues();
    ROS_INFO("Current joint values: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
             current_joint_values[0], current_joint_values[1], current_joint_values[2],
             current_joint_values[3], current_joint_values[4], current_joint_values[5]);
    
    // 使用默认的末端执行器链接
    std::string eef_link = group.getEndEffectorLink();
    ROS_INFO("Using end effector link: %s (inverse)", eef_link.c_str());
    
    // 创建反向轨迹：反转点的顺序
    moveit_msgs::RobotTrajectory reverse_trajectory = action_b_trajectory;
    
    // 反转joint_trajectory中的点
    std::reverse(reverse_trajectory.joint_trajectory.points.begin(), 
                 reverse_trajectory.joint_trajectory.points.end());
    
    // 反转multi_dof_joint_trajectory中的点（如果有）
    if (!reverse_trajectory.multi_dof_joint_trajectory.joint_names.empty())
    {
        std::reverse(reverse_trajectory.multi_dof_joint_trajectory.points.begin(),
                     reverse_trajectory.multi_dof_joint_trajectory.points.end());
    }
    
    // 调整时间戳和速度/加速度
    if (!reverse_trajectory.joint_trajectory.points.empty())
    {
        // 获取原始轨迹的总时间
        double total_time = action_b_trajectory.joint_trajectory.points.back().time_from_start.toSec();
        ROS_INFO("Original trajectory total time: %.4f seconds", total_time);
        
        // 重新计算每个点的时间戳（从0开始，保持原始时间间隔）
        double accumulated_time = 0.0;
        for (size_t i = 0; i < reverse_trajectory.joint_trajectory.points.size(); ++i)
        {
            trajectory_msgs::JointTrajectoryPoint& point = reverse_trajectory.joint_trajectory.points[i];
            
            // 第一个点的时间戳为0
            if (i == 0)
            {
                point.time_from_start = ros::Duration(0.0);
                accumulated_time = 0.0;
            }
            else
            {
                // 计算当前点对应的原始轨迹点的索引
                size_t orig_idx = action_b_trajectory.joint_trajectory.points.size() - 1 - i;
                size_t prev_orig_idx = orig_idx + 1;
                
                // 计算时间间隔（使用原始轨迹中相邻点的时间差）
                double time_diff = action_b_trajectory.joint_trajectory.points[orig_idx].time_from_start.toSec() - 
                                   action_b_trajectory.joint_trajectory.points[prev_orig_idx].time_from_start.toSec();
                accumulated_time += std::abs(time_diff);
                point.time_from_start = ros::Duration(accumulated_time);
            }
            
            // 反转速度和加速度方向
            for (size_t j = 0; j < point.velocities.size(); ++j)
            {
                point.velocities[j] *= -1.0;
            }
            for (size_t j = 0; j < point.accelerations.size(); ++j)
            {
                point.accelerations[j] *= -1.0;
            }
        }
        
        // 打印反向轨迹的第一个和最后一个点信息
        ROS_INFO("Reversed trajectory first point positions: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                 reverse_trajectory.joint_trajectory.points[0].positions[0],
                 reverse_trajectory.joint_trajectory.points[0].positions[1],
                 reverse_trajectory.joint_trajectory.points[0].positions[2],
                 reverse_trajectory.joint_trajectory.points[0].positions[3],
                 reverse_trajectory.joint_trajectory.points[0].positions[4],
                 reverse_trajectory.joint_trajectory.points[0].positions[5]);
        size_t rev_last_idx = reverse_trajectory.joint_trajectory.points.size() - 1;
        ROS_INFO("Reversed trajectory last point positions: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                 reverse_trajectory.joint_trajectory.points[rev_last_idx].positions[0],
                 reverse_trajectory.joint_trajectory.points[rev_last_idx].positions[1],
                 reverse_trajectory.joint_trajectory.points[rev_last_idx].positions[2],
                 reverse_trajectory.joint_trajectory.points[rev_last_idx].positions[3],
                 reverse_trajectory.joint_trajectory.points[rev_last_idx].positions[4],
                 reverse_trajectory.joint_trajectory.points[rev_last_idx].positions[5]);
    }
    
    ROS_INFO("Reversed trajectory with %zu waypoints", reverse_trajectory.joint_trajectory.points.size());
    
    // 执行反向轨迹
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = reverse_trajectory;
    group.setMaxVelocityScalingFactor(0.3);
    
    bool success = (group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    if (success)
    {
        ROS_INFO("Successfully executed Action B Inverse by reversing saved trajectory");
    }
    else
    {
        ROS_ERROR("Failed to execute Action B Inverse trajectory");
    }
    
    return success;
}

/**
 * @brief 动作C：让最后一个关节（frrobot_j6）旋转到当前值-180度
 * @param group MoveGroupInterface对象
 * @return 是否成功
 */
bool executeActionC(moveit::planning_interface::MoveGroupInterface &group)
{
    ROS_INFO("========== Executing Action C: Rotating last joint to current - 180 degrees ==========");
    
    // 获取当前关节值（应该是动作B执行后的值）
    std::vector<double> current_joint_values = group.getCurrentJointValues();
    if (current_joint_values.size() < 6)
    {
        ROS_ERROR("Failed to get current joint values or insufficient joints!");
        return false;
    }
    
    // 保存当前关节值
    std::vector<double> target_joint_values = current_joint_values;
    
    // 最后一个关节（索引5，frrobot_j6）旋转到当前值-180度
    // 180度转换为弧度：180 * π / 180 = π 弧度
    const double rotation_angle_rad = 180.0 * M_PI / 180.0;
    target_joint_values[5] -= rotation_angle_rad;
    
    ROS_INFO("Current joint 6 value: %.4f rad (%.2f deg)", 
             current_joint_values[5], current_joint_values[5] * 180.0 / M_PI);
    ROS_INFO("Target joint 6 value: %.4f rad (%.2f deg)", 
             target_joint_values[5], target_joint_values[5] * 180.0 / M_PI);
    
    // 移动到目标关节配置
    return moveToJointConfig(group, target_joint_values, 0.3);
}

/**
 * @brief 动作C的逆动作：让最后一个关节（frrobot_j6）转回动作B执行后的值
 * @param group MoveGroupInterface对象
 * @return 是否成功
 */
bool executeActionCInverse(moveit::planning_interface::MoveGroupInterface &group)
{
    ROS_INFO("========== Executing Action C Inverse: Rotating last joint back to Action B end value ==========");
    
    // 获取当前关节值
    std::vector<double> current_joint_values = group.getCurrentJointValues();
    if (current_joint_values.size() < 6)
    {
        ROS_ERROR("Failed to get current joint values or insufficient joints!");
        return false;
    }
    
    // 保存当前关节值
    std::vector<double> target_joint_values = current_joint_values;
    
    // 最后一个关节（索引5，frrobot_j6）转回动作B执行后的值
    target_joint_values[5] = action_b_end_j6_value;
    
    ROS_INFO("Current joint 6 value: %.4f rad (%.2f deg)", 
             current_joint_values[5], current_joint_values[5] * 180.0 / M_PI);
    ROS_INFO("Target joint 6 value (Action B end): %.4f rad (%.2f deg)", 
             target_joint_values[5], target_joint_values[5] * 180.0 / M_PI);
    
    // 移动到目标关节配置
    return moveToJointConfig(group, target_joint_values, 0.3);
}

/**
 * @brief 主函数：执行动作序列
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "action_sequence_node");
    ros::NodeHandle node_handle;
    
    // 使用异步spinner，避免阻塞
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    // 等待MoveIt服务启动
    ros::Duration(2.0).sleep();
    
    // 创建MoveGroupInterface对象，使用单臂配置的组名
    moveit::planning_interface::MoveGroupInterface arm_group("fr5v6_arm");
    
    // 配置MoveGroupInterface
    arm_group.setPoseReferenceFrame("world");
    arm_group.allowReplanning(true);
    arm_group.setPlanningTime(10.0);
    arm_group.setGoalPositionTolerance(0.001);
    arm_group.setGoalOrientationTolerance(0.01);
    
    ROS_INFO("Action Sequence Node started");
    ROS_INFO("Planning group: %s", arm_group.getName().c_str());
    ROS_INFO("Planning frame: %s", arm_group.getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", arm_group.getEndEffectorLink().c_str());
    
    // 列出所有可用的链接
    std::vector<std::string> link_names = arm_group.getLinkNames();
    ROS_INFO("Available links in planning group (%zu total):", link_names.size());
    for (size_t i = 0; i < link_names.size() && i < 20; ++i) {
        ROS_INFO("  - %s", link_names[i].c_str());
    }
    if (link_names.size() > 20) {
        ROS_INFO("  ... and %zu more links", link_names.size() - 20);
    }
    
    // 获取初始关节配置
    std::vector<double> initial_joint_config = getInitialJointConfig(arm_group);
    ROS_INFO("Initial joint configuration: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
             initial_joint_config[0], initial_joint_config[1], initial_joint_config[2],
             initial_joint_config[3], initial_joint_config[4], initial_joint_config[5]);
    
    bool success = true;
    
    try
    {
        // ========== 步骤1: 到达初始关节配置 ==========
        ROS_INFO("\n========== Step 1: Moving to initial joint configuration ==========");
        success = moveToJointConfig(arm_group, initial_joint_config, 0.3);
        if (!success)
        {
            ROS_ERROR("Failed to move to initial joint configuration!");
            return -1;
        }
        ros::Duration(1.0).sleep();
        
        // ========== 步骤2: 执行动作A ==========
        ROS_INFO("\n========== Step 2: Executing Action A ==========");
        success = executeActionA(arm_group);
        if (!success)
        {
            ROS_ERROR("Failed to execute Action A!");
            return -1;
        }
        ros::Duration(1.0).sleep();
        
        // ========== 步骤3: 执行动作A的逆动作，回到初始关节配置 ==========
        ROS_INFO("\n========== Step 3: Executing Action A Inverse ==========");
        success = executeActionAInverse(arm_group);
        if (!success)
        {
            ROS_ERROR("Failed to execute Action A Inverse!");
            return -1;
        }
        // 确保回到初始配置
        moveToJointConfig(arm_group, initial_joint_config, 0.3);
        ros::Duration(1.0).sleep();
        
        // ========== 步骤4: 执行动作B ==========
        ROS_INFO("\n========== Step 4: Executing Action B ==========");
        success = executeActionB(arm_group);
        if (!success)
        {
            ROS_ERROR("Failed to execute Action B!");
            return -1;
        }
        // ========== 步骤5: 执行动作C ==========
        ROS_INFO("\n========== Step 5: Executing Action C ==========");
        success = executeActionC(arm_group);
        if (!success)
        {
            ROS_ERROR("Failed to execute Action C!");
            return -1;
        }
        ros::Duration(1.0).sleep();
        
        // ========== 步骤6: 执行动作C的逆动作 ==========
        ROS_INFO("\n========== Step 6: Executing Action C Inverse ==========");
        success = executeActionCInverse(arm_group);
        if (!success)
        {
            ROS_ERROR("Failed to execute Action C Inverse!");
            return -1;
        }
        ros::Duration(1.0).sleep();
        
        // ========== 步骤7: 执行动作B的逆动作，回到初始关节配置 ==========
        ROS_INFO("\n========== Step 7: Executing Action B Inverse ==========");
        success = executeActionBInverse(arm_group);
        if (!success)
        {
            ROS_ERROR("Failed to execute Action B Inverse!");
            return -1;
        }
        // 确保回到初始配置
        // moveToJointConfig(arm_group, initial_joint_config, 0.3);
        ros::Duration(1.0).sleep();
        
        ROS_INFO("\n========== Action Sequence Completed Successfully! ==========");
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Exception occurred: %s", e.what());
        return -1;
    }
    
    ros::shutdown();
    return 0;
}

