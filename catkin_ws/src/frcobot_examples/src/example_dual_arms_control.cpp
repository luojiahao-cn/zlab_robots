#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

void moveToState(moveit::planning_interface::MoveGroupInterface &group, const std::string &state_name, double velocity_scaling_factor = 1.0)
{
    group.setMaxVelocityScalingFactor(velocity_scaling_factor);
    group.setNamedTarget(state_name);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        group.move();
        ROS_INFO("Moved to state: %s", state_name.c_str());
    }
    else
    {
        ROS_ERROR("Failed to move to state: %s", state_name.c_str());
    }
}

// 使用分离参数的方式移动到指定位姿
void moveToPose(moveit::planning_interface::MoveGroupInterface &group, 
                double x, double y, double z, 
                double qx, double qy, double qz, double qw, 
                double velocity_scaling_factor = 1.0)
{
    group.setMaxVelocityScalingFactor(velocity_scaling_factor);
    
    // 创建目标位姿
    geometry_msgs::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    target_pose.orientation.x = qx;
    target_pose.orientation.y = qy;
    target_pose.orientation.z = qz;
    target_pose.orientation.w = qw;
    
    // 设置目标位姿
    group.setPoseTarget(target_pose);
    
    // 规划运动
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    // 执行运动
    if (success)
    {
        group.move();
        ROS_INFO("Moved to pose: [%f, %f, %f] [%f, %f, %f, %f]", 
                 x, y, z, qx, qy, qz, qw);
    }
    else
    {
        ROS_ERROR("Failed to move to pose: [%f, %f, %f] [%f, %f, %f, %f]", 
                  x, y, z, qx, qy, qz, qw);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "dual_arm_control");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface group_dual("dual_robots");
    moveit::planning_interface::MoveGroupInterface group_robot1("robot1");
    moveit::planning_interface::MoveGroupInterface group_robot2("robot2");

    ros::Rate rate(0.3); // 0.5 Hz, i.e., one cycle every 2 seconds

    while (ros::ok())
    {
        // Move both arms to 'zero' state
        moveToState(group_dual, "up");
        rate.sleep();

        // Move robot1 to 'up' state
        moveToState(group_robot1, "ready",0.5);
        rate.sleep();

        // Move robot2 to 'up' state
        moveToState(group_robot2, "ready",0.5);
        rate.sleep();

        moveToPose(group_robot1, 0.4, 0.2, 0.5, 0.0, 0.0, 0.0, 1.0, 0.3);
        rate.sleep();
    }

    ros::shutdown();
    return 0;
}