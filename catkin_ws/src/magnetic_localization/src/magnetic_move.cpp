#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "magnetic_move");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 设置move_group
    moveit::planning_interface::MoveGroupInterface arm("fr5v6_arm");
    std::string end_effector_link = arm.getEndEffectorLink();
    ROS_INFO("End effector link: %s", end_effector_link.c_str());
    std::string reference_frame = "world";
    arm.setPoseReferenceFrame(reference_frame);

    arm.allowReplanning(true); // 允许重新规划

    //设置位置（单位：米）和姿态（单位：弧度）的容差
    arm.setGoalPositionTolerance(0.001); // 位置容差
    arm.setGoalOrientationTolerance(0.01); // 姿态容差

    // 设置最大速度和加速度比例
    arm.setMaxVelocityScalingFactor(0.1);     // 设置最大速度比例

    // 设置目标位置
    geometry_msgs::Pose target_pose;
    target_pose.position.x = 0.6;
    target_pose.position.y = 0.0;
    target_pose.position.z = 1.0;

    // 使用RPY设置姿态(单位是弧度)
    tf::Quaternion q;
    q.setRPY(M_PI, 0, 0); // Roll=PI, Pitch=0, Yaw=0 使末端朝下
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();

    // 移动到起始位置
    arm.setPoseTarget(target_pose);
    arm.move();

    ROS_INFO("Moved to start position");
    sleep(1.0);

    std::vector<geometry_msgs::Pose> waypoints;
    // 添加起始位置
    waypoints.push_back(target_pose);

    // 边长
    double side_length = 0.03;
    // 循环四次
    for (int i = 0; i < 5; ++i) {
        // 计算下一个目标位置
        target_pose.position.x += side_length; // 向右移动
        waypoints.push_back(target_pose);
        target_pose.position.y += side_length; // 向前移动
        waypoints.push_back(target_pose);
        target_pose.position.x -= side_length; // 向左移动
        waypoints.push_back(target_pose);
        target_pose.position.y -= side_length; // 向后移动
        waypoints.push_back(target_pose);
    }

    // 笛卡尔空间下的路径规划
    moveit_msgs::RobotTrajectory trajectory;
    const double eef_step = 0.00002;                   // 末端执行器步长
    double fraction = 0.0;                          // 规划成功的路径比例

    fraction = arm.computeCartesianPath(waypoints, eef_step, trajectory);   // 计算路径
    ROS_INFO("Path planning fraction: %.2f%%", fraction * 100.0);
    if (fraction < 0.9) {
        ROS_WARN("Path planning failed, fraction < 0.8");
        return -1;
    }

    // 生成机械臂的运动规划数据
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    arm.setMaxVelocityScalingFactor(0.01);
    // 执行运动
    arm.execute(plan);

    ROS_INFO("Executed trajectory");
    sleep(1.0);

    arm.setMaxVelocityScalingFactor(0.1);
    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("ready");
    arm.move();
    sleep(1);


    ros::shutdown();
    return 0;
}