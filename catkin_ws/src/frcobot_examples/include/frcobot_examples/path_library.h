#pragma once
#include <vector>
#include <geometry_msgs/Pose.h>

/**
 * @brief 生成正方形路径的关键点序列
 * @param start_pose 起始位姿
 * @param side_length 正方形边长（单位：米），默认0.03
 * @return 路径点序列
 */
std::vector<geometry_msgs::Pose> generate_square_waypoints(const geometry_msgs::Pose& start_pose, double side_length = 0.03);

/**
 * @brief 生成螺旋路径的关键点序列
 * @param start_pose 起始位姿
 * @param radius 螺旋半径（单位：米）
 * @param pitch 每圈上升高度（单位：米）
 * @param num_turns 螺旋圈数
 * @param points_per_turn 每圈的路径点数
 * @return 路径点序列
 */
std::vector<geometry_msgs::Pose> generate_spiral_waypoints(const geometry_msgs::Pose& start_pose, double radius, double pitch, int num_turns, int points_per_turn);

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
std::vector<geometry_msgs::Pose> generate_archimedean_spiral_waypoints(const geometry_msgs::Pose &start_pose, double a, double b, double pitch, int num_turns, int points_per_turn);