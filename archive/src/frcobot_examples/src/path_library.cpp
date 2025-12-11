#include "frcobot_examples/path_library.h"
#include <cmath>

/**
 * @brief 生成正方形路径的关键点序列
 * @param start_pose 起始位姿
 * @param side_length 正方形边长（单位：米），默认0.03
 * @return 路径点序列
 */
std::vector<geometry_msgs::Pose> generate_square_waypoints(const geometry_msgs::Pose& start_pose, double side_length)
{
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose target_pose = start_pose;
    
    waypoints.push_back(target_pose);

    target_pose.position.x += side_length;
    waypoints.push_back(target_pose);
    target_pose.position.y += side_length;
    waypoints.push_back(target_pose);
    target_pose.position.x -= side_length;
    waypoints.push_back(target_pose);
    target_pose.position.y -= side_length;
    waypoints.push_back(target_pose);
    
    return waypoints;
}

/**
 * @brief 生成螺旋路径的关键点序列
 * @param start_pose 起始位姿
 * @param radius 螺旋半径（单位：米）
 * @param pitch 每圈上升高度（单位：米）
 * @param num_turns 螺旋圈数
 * @param points_per_turn 每圈的路径点数
 * @return 路径点序列
 */
std::vector<geometry_msgs::Pose> generate_spiral_waypoints(const geometry_msgs::Pose& start_pose, double radius, double pitch, int num_turns, int points_per_turn)
{
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose pose = start_pose;

    double theta = 0.0;
    double d_theta = 2 * M_PI / points_per_turn;
    int total_points = num_turns * points_per_turn;

    for (int i = 0; i < total_points; ++i)
    {
        pose.position.x = start_pose.position.x + radius * cos(theta);
        pose.position.y = start_pose.position.y + radius * sin(theta);
        pose.position.z = start_pose.position.z + pitch * theta / (2 * M_PI);

        waypoints.push_back(pose);
        theta += d_theta;
    }
    return waypoints;
}

/**
 * @brief 生成阿基米德螺线路径点
 * @param start_pose 起始位姿
 * @param a 螺线初始半径
 * @param b 螺线增长系数
 * @param pitch 每圈上升高度
 * @param num_turns 螺线圈数
 * @param points_per_turn 每圈路径点数
 * @return 路径点序列
 */
std::vector<geometry_msgs::Pose> generate_archimedean_spiral_waypoints(const geometry_msgs::Pose &start_pose, double a, double b, double pitch, int num_turns, int points_per_turn)
{
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose pose = start_pose;

    double theta = 0.0;
    for (int turn = 0; turn < num_turns; ++turn)
    {
        // 当前圈的半径
        double r = a + b * theta;
        // 本圈的采样点数，随半径线性增加
        int points_this_turn = static_cast<int>(points_per_turn * (1.0 + r / a));
        if (points_this_turn < points_per_turn)
            points_this_turn = points_per_turn;
        double d_theta = 2 * M_PI / points_this_turn;

        for (int i = 0; i < points_this_turn; ++i)
        {
            double r_now = a + b * theta;
            pose.position.x = start_pose.position.x + r_now * cos(theta);
            pose.position.y = start_pose.position.y + r_now * sin(theta);
            pose.position.z = start_pose.position.z + pitch * theta / (2 * M_PI);

            waypoints.push_back(pose);
            theta += d_theta;
        }
    }
    return waypoints;
}