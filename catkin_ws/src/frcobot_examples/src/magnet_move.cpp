#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include "frcobot_examples/path_library.h"
#include <visualization_msgs/Marker.h>

// 发布末端轨迹到RViz
void publish_end_effector_trajectory(ros::Publisher &pub, const std::vector<geometry_msgs::Pose> &waypoints, const std::string &frame_id)
{
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = frame_id;
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "end_effector_trajectory";
    line_strip.id = 0;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.scale.x = 0.002;
    line_strip.color.r = 1.0;
    line_strip.color.g = 0.0;
    line_strip.color.b = 0.0;
    line_strip.color.a = 1.0;

    for (const auto &pose : waypoints)
    {
        geometry_msgs::Point p;
        p.x = pose.position.x;
        p.y = pose.position.y;
        p.z = pose.position.z;
        line_strip.points.push_back(p);
    }

    pub.publish(line_strip);
    ROS_INFO("End effector trajectory published to RViz");
}

// 初始化目标位姿
geometry_msgs::Pose init_target_pose()
{
    geometry_msgs::Pose pose;
    pose.position.x = 0.6;
    pose.position.y = 0.0;
    pose.position.z = 1.01;
    tf::Quaternion q;
    q.setRPY(M_PI, 0, 0);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
}

// 初始化MoveIt接口
void init_moveit(moveit::planning_interface::MoveGroupInterface &arm, const std::string &reference_frame)
{
    arm.setPoseReferenceFrame(reference_frame);
    arm.allowReplanning(true);
    arm.setGoalPositionTolerance(0.001);
    arm.setGoalOrientationTolerance(0.01);
    arm.setMaxVelocityScalingFactor(0.1);
    arm.setNamedTarget("ready");
    arm.move();
    ros::Duration(1.0).sleep();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "magnetic_move");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("end_effector_trajectory", 1, true);

    moveit::planning_interface::MoveGroupInterface arm("fr5v6_arm");
    std::string reference_frame = "world";
    init_moveit(arm, reference_frame);

    geometry_msgs::Pose target_pose = init_target_pose();
    double current_z = target_pose.position.z;

    while (ros::ok() && current_z <= 1.5)
    {
        // 移动到起始位置
        arm.setPoseTarget(target_pose);
        arm.move();
        ROS_INFO("Moved to start position");
        ros::Duration(1.0).sleep();

        // 生成路径（可切换不同路径类型）
        // std::vector<geometry_msgs::Pose> waypoints = generate_square_waypoints(target_pose);
        // std::vector<geometry_msgs::Pose> waypoints = generate_spiral_waypoints(target_pose, 0.02, 0.01, 5, 100);
        std::vector<geometry_msgs::Pose> waypoints = generate_archimedean_spiral_waypoints(target_pose, 0.01, 0.001, 0.00, 13, 500);

        // 笛卡尔空间路径规划
        moveit_msgs::RobotTrajectory trajectory;
        const double eef_step = 0.00003; 
        double fraction = arm.computeCartesianPath(waypoints, eef_step, trajectory);
        ROS_INFO("Path planning fraction: %.2f%%", fraction * 100.0);
        if (fraction < 0.9)
        {
            ROS_WARN("Path planning failed, fraction < 0.9");
            return -1;
        }

        // 发布末端轨迹到RViz
        publish_end_effector_trajectory(marker_pub, waypoints, reference_frame);

        // 构造以current_z为名的bag文件
        char bag_name[128];
        snprintf(bag_name, sizeof(bag_name), "/home/lawkaho/workshop/zlab_robots/catkin_ws/data/magnetic_data/tf_record_%.3f", current_z);
        std::string bag_cmd = "rosbag record -O " + std::string(bag_name) + " /tf /tf_static __name:=tf_bag_recorder_" + std::to_string(int(current_z * 1000)) + " &";
        system(bag_cmd.c_str());
        ROS_INFO("Started rosbag recording: %s.bag", bag_name);

        // 执行运动
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory; 
        arm.execute(plan);
        ROS_INFO("Executed trajectory");
        ros::Duration(1.0).sleep();

        // 结束rosbag
        std::string kill_cmd = "rosnode kill /tf_bag_recorder_" + std::to_string(int(current_z * 1000));
        system(kill_cmd.c_str());
        ROS_INFO("Stopped rosbag recording: %s.bag", bag_name);

        // 增加高度
        current_z += 0.01;
        target_pose.position.z = current_z;
        arm.setMaxVelocityScalingFactor(0.01);
    }

    // 回到初始化位置
    arm.setMaxVelocityScalingFactor(0.1);
    arm.setNamedTarget("ready");
    arm.move();
    ros::Duration(1.0).sleep();

    ros::shutdown();
    return 0;
}