#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

// 生成扫描层的路径点
std::vector<geometry_msgs::Pose> generateScanWaypoints(
    double x_min, double x_max, double y_min, double y_max, double z,
    double step, const geometry_msgs::Quaternion &orientation)
{
    std::vector<geometry_msgs::Pose> waypoints;
    bool reverse = false;
    for (double y = y_min; y <= y_max; y += step)
    {
        geometry_msgs::Pose pose;
        pose.position.y = y;
        pose.position.z = z;
        pose.orientation = orientation;
        if (!reverse)
        {
            for (double x = x_min; x <= x_max; x += step)
            {
                pose.position.x = x;
                waypoints.push_back(pose);
            }
        }
        else
        {
            for (double x = x_max; x >= x_min; x -= step)
            {
                pose.position.x = x;
                waypoints.push_back(pose);
            }
        }
        reverse = !reverse;
    }
    return waypoints;
}

// 发布扫描空间Marker
void publishScanAreaMarker(ros::Publisher &marker_pub,
                           double x_min, double x_max,
                           double y_min, double y_max,
                           double z_start, double z_end)
{
    visualization_msgs::Marker box;
    box.header.frame_id = "world";
    box.header.stamp = ros::Time::now();
    box.ns = "scan_area";
    box.id = 0;
    box.type = visualization_msgs::Marker::CUBE;
    box.action = visualization_msgs::Marker::ADD;
    box.pose.position.x = (x_min + x_max) / 2.0;
    box.pose.position.y = (y_min + y_max) / 2.0;
    box.pose.position.z = (z_start + z_end) / 2.0;
    box.pose.orientation.w = 1.0;
    box.scale.x = x_max - x_min;
    box.scale.y = y_max - y_min;
    box.scale.z = z_end - z_start;
    box.color.r = 0.0f;
    box.color.g = 1.0f;
    box.color.b = 1.0f;
    box.color.a = 0.2f; // 透明度
    box.lifetime = ros::Duration(0); // 永久显示

    marker_pub.publish(box);
}


// 磁场数据采集完成回调
bool mag_data_done_flag = false;
void magDataDoneCallback(const std_msgs::Bool::ConstPtr& msg) {
    mag_data_done_flag = msg->data;
}

// 控制机械臂扫描并采集磁场数据
void scanWithArmAndCollectMagData(moveit::planning_interface::MoveGroupInterface &arm,
                 double x_min, double x_max, double y_min, double y_max,
                 double z_start, double z_end, double z_step, double xy_step,
                 const geometry_msgs::Quaternion &down_orientation,
                 ros::Publisher &mag_data_pub, ros::NodeHandle &nh)
{
    ros::Subscriber mag_data_done_sub = nh.subscribe("/mag_data_done", 1, magDataDoneCallback);

    for (double z = z_start; z <= z_end; z += z_step)
    {
        std::vector<geometry_msgs::Pose> waypoints = generateScanWaypoints(
            x_min, x_max, y_min, y_max, z, xy_step, down_orientation);

        for (const auto& pose : waypoints)
        {
            arm.setPoseTarget(pose);
            bool success = (arm.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (success)
            {
                // 发布磁场数据采集请求
                std_msgs::Empty req;
                mag_data_pub.publish(req);

                // 等待采集完成
                mag_data_done_flag = false;
                ros::Rate rate(10);
                int wait_count = 0;
                while (ros::ok() && !mag_data_done_flag && wait_count < 100) // 最多等10秒
                {
                    ros::spinOnce();
                    rate.sleep();
                    wait_count++;
                }
                if (mag_data_done_flag)
                    ROS_INFO("磁场数据采集完成，继续下一个点");
                else
                    ROS_WARN("磁场数据采集超时，继续下一个点");
            }
            else
            {
                ROS_WARN("机械臂运动失败，跳过该点");
            }
        }
    }
}

// 主流程函数
void scan_process()
{
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 读取参数
    double x_min, x_max, y_min, y_max, z_start, z_end, z_step, xy_step;
    nh.getParam("scan_area/x_min", x_min);
    nh.getParam("scan_area/x_max", x_max);
    nh.getParam("scan_area/y_min", y_min);
    nh.getParam("scan_area/y_max", y_max);
    nh.getParam("scan_area/z_start", z_start);
    nh.getParam("scan_area/z_end", z_end);
    nh.getParam("scan_area/z_step", z_step);
    nh.getParam("scan_area/xy_step", xy_step);

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("scan_area_marker", 1, true);
    ros::Publisher mag_data_pub = nh.advertise<std_msgs::Empty>("/collect_mag_data", 1);

    // 发布扫描空间 Marker
    publishScanAreaMarker(marker_pub, x_min, x_max, y_min, y_max, z_start, z_end);

    moveit::planning_interface::MoveGroupInterface arm("fr5v6_arm");
    arm.setPoseReferenceFrame("world");
    arm.setEndEffectorLink(arm.getEndEffectorLink());
    arm.setNamedTarget("ready");
    arm.move();
    ros::Duration(1.0).sleep();

    // 末端竖直向下
    geometry_msgs::Quaternion down_orientation;
    down_orientation.x = 0.0;
    down_orientation.y = 1.0;
    down_orientation.z = 0.0;
    down_orientation.w = 0.0;

    // 先移动到扫描起点
    std::vector<geometry_msgs::Pose> first_layer = generateScanWaypoints(
        x_min, x_max, y_min, y_max, z_start, xy_step, down_orientation);
    if (!first_layer.empty()) {
        arm.setPoseTarget(first_layer.front());
        arm.move();
    }

    scanWithArmAndCollectMagData(arm, x_min, x_max, y_min, y_max, z_start, z_end, z_step, xy_step, down_orientation, mag_data_pub, nh);

    ros::shutdown();
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "arm_motion_control");
    scan_process();
    return 0;
}