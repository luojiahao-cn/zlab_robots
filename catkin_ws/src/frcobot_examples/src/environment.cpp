#include <ros/ros.h>
#include <moveit_msgs/PlanningScene.h>
#include <geometric_shapes/shape_operations.h>
#include <signal.h>

// 全局变量，用于信号处理
ros::Publisher *g_planning_scene_publisher = nullptr;
std::vector<std::string> g_wall_ids;

// 信号处理函数
void sigintHandler(int sig)
{
    ROS_INFO("Received shutdown signal, removing walls...");

    if (g_planning_scene_publisher)
    {
        // 创建清除墙壁的消息
        moveit_msgs::PlanningScene planning_scene;

        // 准备移除所有墙壁
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.resize(g_wall_ids.size());

        for (size_t i = 0; i < g_wall_ids.size(); ++i)
        {
            collision_objects[i].id = g_wall_ids[i];
            collision_objects[i].operation = moveit_msgs::CollisionObject::REMOVE;
        }

        planning_scene.world.collision_objects = collision_objects;
        planning_scene.is_diff = true;

        // 发布移除墙壁的消息
        g_planning_scene_publisher->publish(planning_scene);
        ROS_INFO("Walls removal message sent");

        // 给一点时间让消息传递
        ros::Duration(0.5).sleep();
    }

    // 退出程序
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "environment", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 注册我们自己的信号处理程序
    signal(SIGINT, sigintHandler);

    // 创建规划场景发布者
    ros::Publisher planning_scene_diff_publisher =
        nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    // 将发布者指针赋给全局变量，以便信号处理函数使用
    g_planning_scene_publisher = &planning_scene_diff_publisher;

    // 等待发布者连接
    ROS_INFO("Waiting for planning scene publisher to connect...");
    ros::Duration(2.0).sleep();

    // 创建完整的规划场景消息
    moveit_msgs::PlanningScene planning_scene;

    // 创建三面墙壁和一个桌子的碰撞对象
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(4);  // 修改为4，增加桌子

    // 保存墙壁和桌子ID到全局变量，用于清除
    g_wall_ids.push_back("back_wall");
    g_wall_ids.push_back("right_side_wall");
    g_wall_ids.push_back("left_side_wall");
    g_wall_ids.push_back("table");  // 添加桌子ID

    // 设置参考坐标系
    std::string base_frame = "world"; // 根据您的机器人修改

    // 1. 背面墙壁 (X-方向)
    collision_objects[0].header.frame_id = base_frame;
    collision_objects[0].id = "back_wall";

    shape_msgs::SolidPrimitive back_wall;
    back_wall.type = back_wall.BOX;
    back_wall.dimensions.resize(3);
    back_wall.dimensions[0] = 0.05; // 厚度(x方向)
    back_wall.dimensions[1] = 2.0; // 宽度(y方向)
    back_wall.dimensions[2] = 2.0; // 高度(z方向)

    geometry_msgs::Pose back_pose;
    back_pose.orientation.w = 1.0;
    back_pose.position.x = -0.65; // 背面-0.65米处设置墙壁（负值表示背面）
    back_pose.position.y = 0.0;
    back_pose.position.z = 1.0; // 墙壁中心高度

    collision_objects[0].primitives.push_back(back_wall);
    collision_objects[0].primitive_poses.push_back(back_pose);
    collision_objects[0].operation = collision_objects[0].ADD;

    // 2. 右侧墙壁 (Y+方向)
    collision_objects[1].header.frame_id = base_frame;
    collision_objects[1].id = "right_side_wall";

    shape_msgs::SolidPrimitive side_wall;
    side_wall.type = side_wall.BOX;
    side_wall.dimensions.resize(3);
    side_wall.dimensions[0] = 2.0; // 长度(x方向)
    side_wall.dimensions[1] = 0.05; // 厚度(y方向)
    side_wall.dimensions[2] = 2.0; // 高度(z方向)

    geometry_msgs::Pose side_pose;
    side_pose.orientation.w = 1.0;
    side_pose.position.x = 0.0;
    side_pose.position.y = 0.9; // 右侧0.9米处设置墙壁
    side_pose.position.z = 1.0; // 墙壁中心高度

    collision_objects[1].primitives.push_back(side_wall);
    collision_objects[1].primitive_poses.push_back(side_pose);
    collision_objects[1].operation = collision_objects[1].ADD;

    // 3. 左侧墙壁 (Y-方向)
    collision_objects[2].header.frame_id = base_frame;
    collision_objects[2].id = "left_side_wall";

    shape_msgs::SolidPrimitive left_wall;
    left_wall.type = left_wall.BOX;
    left_wall.dimensions.resize(3);
    left_wall.dimensions[0] = 2.0; // 长度(x方向)
    left_wall.dimensions[1] = 0.05; // 厚度(y方向)
    left_wall.dimensions[2] = 2.0; // 高度(z方向)

    geometry_msgs::Pose left_pose;
    left_pose.orientation.w = 1.0;
    left_pose.position.x = 0.0;
    left_pose.position.y = -0.9; // 左侧-0.9米处设置墙壁（负值表示左侧）
    left_pose.position.z = 1.0;  // 墙壁中心高度

    collision_objects[2].primitives.push_back(left_wall);
    collision_objects[2].primitive_poses.push_back(left_pose);
    collision_objects[2].operation = collision_objects[2].ADD;

    // 4. 桌子 (位于机械臂前方)
    collision_objects[3].header.frame_id = base_frame;
    collision_objects[3].id = "table";

    shape_msgs::SolidPrimitive table;
    table.type = table.BOX;
    table.dimensions.resize(3);
    table.dimensions[0] = 0.6;  // 长度(x方向) 60cm
    table.dimensions[1] = 1.5;  // 宽度(y方向) 150cm
    table.dimensions[2] = 0.5;  // 高度(z方向) 50cm

    geometry_msgs::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = 0.5;  // 前方0.6米处设置桌子
    table_pose.position.y = 0.0;  // 居中放置
    table_pose.position.z = 0.4; // 桌子中心点高度

    collision_objects[3].primitives.push_back(table);
    collision_objects[3].primitive_poses.push_back(table_pose);
    collision_objects[3].operation = collision_objects[3].ADD;

    // 添加颜色信息，使墙壁和桌子在RViz中更加可见
    std::vector<moveit_msgs::ObjectColor> colors;
    colors.resize(4);  // 修改为4，增加桌子颜色

    // 灰色背墙
    colors[0].id = collision_objects[0].id;
    colors[0].color.r = 0.5;
    colors[0].color.g = 0.5;
    colors[0].color.b = 0.5;
    colors[0].color.a = 0.7; // 半透明

    // 灰色右侧墙
    colors[1].id = collision_objects[1].id;
    colors[1].color.r = 0.5;
    colors[1].color.g = 0.5;
    colors[1].color.b = 0.5;
    colors[1].color.a = 0.7; // 半透明

    // 灰色左侧墙
    colors[2].id = collision_objects[2].id;
    colors[2].color.r = 0.5;
    colors[2].color.g = 0.5;
    colors[2].color.b = 0.5;
    colors[2].color.a = 0.7; // 半透明

    // 深灰色桌子 (颜色稍深以便区分)
    colors[3].id = collision_objects[3].id;
    colors[3].color.r = 0.3;
    colors[3].color.g = 0.3;
    colors[3].color.b = 0.3;
    colors[3].color.a = 0.9; // 稍不透明

    // 将墙壁添加到规划场景消息中
    planning_scene.world.collision_objects = collision_objects;
    planning_scene.object_colors = colors;
    planning_scene.is_diff = true;

    // 发布到规划场景话题
    planning_scene_diff_publisher.publish(planning_scene);
    ROS_INFO("Environment published");

    // 定期重新发布墙壁，确保它们保持可见
    ros::Rate rate(1); // 1Hz
    while (ros::ok())
    {
        planning_scene_diff_publisher.publish(planning_scene);
        rate.sleep();
    }

    // 程序正常退出时也清除墙壁（通常不会执行到这里，因为会收到SIGINT信号）
    sigintHandler(0);

    return 0;
}