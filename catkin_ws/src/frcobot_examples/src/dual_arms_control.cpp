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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dual_arm_control");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface group_multi("multi_robots");
    moveit::planning_interface::MoveGroupInterface group_robot1("robot1");
    moveit::planning_interface::MoveGroupInterface group_robot2("robot2");

    ros::Rate rate(0.5); // 0.5 Hz, i.e., one cycle every 2 seconds

    while (ros::ok())
    {
        // Move both arms to 'zero' state
        moveToState(group_multi, "up");
        rate.sleep();

        // Move robot1 to 'up' state
        moveToState(group_robot1, "ready",0.5);
        rate.sleep();

        // Move robot2 to 'up' state
        moveToState(group_robot2, "ready",0.5);
        rate.sleep();
    }

    ros::shutdown();
    return 0;
}