#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Pose.h>

// 函数：末端执行器沿指定方向以指定速度移动指定距离
bool moveInDirection(
    moveit::planning_interface::MoveGroupInterface& group, 
    const std::vector<double>& direction,  // x, y, z 方向
    double distance,                       // 移动距离(米)
    double velocity_scale = 0.5)           // 速度比例(0.0-1.0)
{
    // 获取当前位姿
    geometry_msgs::Pose start_pose = group.getCurrentPose().pose;
    
    // 计算目标位姿
    geometry_msgs::Pose target_pose = start_pose;
    target_pose.position.x += direction[0] * distance;
    target_pose.position.y += direction[1] * distance;
    target_pose.position.z += direction[2] * distance;

    // 打印当前位姿
    ROS_INFO("Current position: [%.2f, %.2f, %.2f]", start_pose.position.x, start_pose.position.y, start_pose.position.z);
    // 打印目标位姿
    ROS_INFO("Target position: [%.2f, %.2f, %.2f]", target_pose.position.x, target_pose.position.y, target_pose.position.z);

    // 创建路径点
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose);
    
    // 设置参考坐标系
    group.setPoseReferenceFrame("robot_base");

    // 设置规划时间
    group.setPlanningTime(10.0); // 设置为10秒

    // 计算笛卡尔路径
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01; // 1cm步长
    double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    
    if (fraction < 0.9) {
        ROS_ERROR("Path planning failed, only %.2f%% of the path was completed", fraction * 100.0);
        return false;
    }
    
    // 创建运动规划
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    
    // 设置速度
    group.setMaxVelocityScalingFactor(velocity_scale);
    
    // 执行运动
    ROS_INFO("Executing movement in direction [%.2f, %.2f, %.2f]", direction[0], direction[1], direction[2]);
    return group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "end_effector_direction_control");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    // 使用机械臂组
    // 根据您的配置选择正确的组名："robot1"、"robot2"或"fr5v6_arm"
    moveit::planning_interface::MoveGroupInterface arm_group("fr5v6_arm");
    
    // 设置规划参考坐标系
    arm_group.setPoseReferenceFrame("robot_base");
    
    // 设置末端执行器
    arm_group.setEndEffectorLink(arm_group.getEndEffectorLink());

    while (ros::ok())
    {
        // 设置起始状态为当前状态
        arm_group.setStartStateToCurrentState();
        
        // 示例0: 运动到ready状态
        arm_group.setNamedTarget("ready");
        arm_group.move();
        // ros::Duration(2.0).sleep();

        // 示例1: 沿 +X 方向移动10cm，速度为最大速度的30%
        std::vector<double> direction_x = {1.0, 0.0, 0.0};
        moveInDirection(arm_group, direction_x, 0.1, 0.2);
        // ros::Duration(2.0).sleep();

        // 示例2: 沿 +Y 方向移动10cm，速度为最大速度的50%
        std::vector<double> direction_y = {0.0, 1.0, 0.0};
        moveInDirection(arm_group, direction_y, 0.1, 0.2);
        // ros::Duration(2.0).sleep();

        // 示例3: 沿 +Z 方向移动10cm，速度为最大速度的40%
        std::vector<double> direction_z = {0.0, 0.0, 1.0};
        moveInDirection(arm_group, direction_z, 0.1, 0.2);
        // ros::Duration(2.0).sleep();

        // 示例4: 沿对角线方向移动，速度为最大速度的25%
        std::vector<double> direction_diag = {0.577, 0.577, 0.577}; // 归一化的对角线方向
        moveInDirection(arm_group, direction_diag, 0.1, 0.2);
        ros::Duration(2.0).sleep();
    }
    
    ros::shutdown();
    return 0;
}
