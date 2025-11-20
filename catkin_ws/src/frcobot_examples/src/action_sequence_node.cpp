#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <cmath>

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
 * @return 初始关节配置值向量
 */
std::vector<double> getInitialJointConfig(moveit::planning_interface::MoveGroupInterface &group)
{
    // 从参数服务器或SRDF中获取初始配置
    // 这里先使用"ready"状态作为初始配置，后续可以根据需要修改
    std::vector<double> initial_config;
    
    // 获取当前关节值作为初始配置
    std::vector<double> current_joint_values = group.getCurrentJointValues();
    
    // 如果当前关节值有效，使用当前值；否则使用"ready"状态
    if (current_joint_values.size() > 0)
    {
        initial_config = current_joint_values;
        ROS_INFO("Using current joint values as initial configuration");
    }
    else
    {
        // 使用"ready"状态的关节值
        // 根据SRDF文件，ready状态的关节值为：
        // j1=0, j2=-1.4189, j3=-1.5152, j4=-0.0015, j5=1.6031, j6=0
        initial_config = {0.0, -1.4189, -1.5152, -0.0015, 1.6031, 0.0};
        ROS_INFO("Using default 'ready' state as initial configuration");
    }
    
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
 * @brief 动作B：待实现
 * @param group MoveGroupInterface对象
 * @return 是否成功
 */
bool executeActionB(moveit::planning_interface::MoveGroupInterface &group)
{
    ROS_INFO("========== Executing Action B ==========");
    // TODO: 实现动作B的具体逻辑
    
    ros::Duration(0.5).sleep();
    ROS_WARN("Action B is not yet implemented!");
    
    return true;
}

/**
 * @brief 动作B的逆动作：待实现
 * @param group MoveGroupInterface对象
 * @return 是否成功
 */
bool executeActionBInverse(moveit::planning_interface::MoveGroupInterface &group)
{
    ROS_INFO("========== Executing Action B Inverse ==========");
    // TODO: 实现动作B的逆动作
    
    ros::Duration(0.5).sleep();
    ROS_WARN("Action B Inverse is not yet implemented!");
    
    return true;
}

/**
 * @brief 动作C：让最后一个关节（frrobot_j6）旋转170度
 * @param group MoveGroupInterface对象
 * @return 是否成功
 */
bool executeActionC(moveit::planning_interface::MoveGroupInterface &group)
{
    ROS_INFO("========== Executing Action C: Rotating last joint by 170 degrees ==========");
    
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
 * @brief 动作C的逆动作：让最后一个关节（frrobot_j6）反向旋转170度
 * @param group MoveGroupInterface对象
 * @return 是否成功
 */
bool executeActionCInverse(moveit::planning_interface::MoveGroupInterface &group)
{
    ROS_INFO("========== Executing Action C Inverse: Rotating last joint by -170 degrees ==========");
    
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
        ros::Duration(1.0).sleep();
        
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
        moveToJointConfig(arm_group, initial_joint_config, 0.3);
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

