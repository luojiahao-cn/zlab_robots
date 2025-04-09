#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <magnetic_localization/MagneticData.h>
#include <magnetic_localization/MagnetEstimation.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <ros/package.h>

class MagneticVisualizer {
public:
    MagneticVisualizer() {
        // 初始化传感器位置矩阵
        initializeSensorPositions();
        
        // 创建发布者用于发布标记
        marker_pub = nh.advertise<visualization_msgs::MarkerArray>("magnetic_markers", 10);
        // 订阅磁场数据
        magnetic_sub = nh.subscribe("/magnetic/sensor_data", 1000,
                                    &MagneticVisualizer::magneticCallback, this);
        // 订阅磁铁位置估计数据
        magnet_sub = nh.subscribe("/magnetic/magnet_pose", 10,
                                  &MagneticVisualizer::magnetEstimationCallback, this);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher marker_pub;
    ros::Subscriber magnetic_sub;
    ros::Subscriber magnet_sub; // 磁铁位置订阅者

    std::map<int, geometry_msgs::Point> sensor_positions; // 使用 map 存储标签和位置的对应关系

    void initializeSensorPositions()
    {
        // 使用 ros::package 获取包路径
        std::string package_path = ros::package::getPath("magnetic_localization");
        std::string default_config_path = package_path + "/config/sensor_definitions.yaml";

        // 定义 YAML 文件路径参数
        std::string sensor_positions_file;
        nh.param<std::string>("sensor_definition_file", sensor_positions_file, default_config_path);

        // 加载 YAML 文件
        YAML::Node config = YAML::LoadFile(sensor_positions_file);

        // 检查是否包含传感器定义
        if (!config["sensors"])
        {
            throw std::runtime_error("Sensor positions file does not contain 'sensors' key.");
        }

        // 读取传感器定义
        const auto &sensors = config["sensors"];
        sensor_positions.clear(); // 清空之前的内容

        for (const auto &sensor : sensors)
        {
            if (!sensor["label"] || !sensor["position"])
            {
                throw std::runtime_error("Invalid sensor definition format in YAML file.");
            }

            int label = sensor["label"].as<int>();
            auto position = sensor["position"].as<std::vector<double>>();

            if (position.size() != 3)
            {
                throw std::runtime_error("Position must have exactly 3 elements (x, y, z).");
            }

            // 存储传感器位置
            geometry_msgs::Point point;
            point.x = position[0];
            point.y = position[1];
            point.z = position[2];
            sensor_positions[label] = point;
        }
    }

    void magneticCallback(const magnetic_localization::MagneticData::ConstPtr &msg)
    {
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker marker;

        // 设置箭头标记的基本属性
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "magnetic_field";
        marker.id = msg->sensor_label;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        auto sensor_it = sensor_positions.find(msg->sensor_label);
        if (sensor_it != sensor_positions.end())
        {
            marker.pose.position = sensor_it->second;
        }
        else
        {
            ROS_WARN("Sensor label %d not found in configuration", msg->sensor_label);
            return;
        }
        
        // 计算箭头方向和大小
        double magnitude = sqrt(pow(msg->x, 2) + pow(msg->y, 2) + pow(msg->z, 2));
        if (magnitude > 0) {
            // 归一化向量
            double dx = msg->x / magnitude;
            double dy = msg->y / magnitude;
            double dz = msg->z / magnitude;

            // 计算旋转四元数
            tf2::Vector3 axis_z(0, 0, 1);       // 初始方向（沿z轴）
            tf2::Vector3 direction(dx, dy, dz); // 目标方向

            // 计算旋转轴（叉积）
            tf2::Vector3 rotation_axis = axis_z.cross(direction);

            // 如果旋转轴接近零向量，说明是平行或反平行
            tf2::Quaternion q; // 声明四元数变量
            if (rotation_axis.length2() < 1e-6)
            {
                if (dz > 0)
                {
                    // 如果平行，不需要旋转
                    q.setRPY(0, 0, 0);
                }
                else
                {
                    // 如果反平行，绕x轴旋转180度
                    q.setRPY(M_PI, 0, 0);
                }
            }
            else
            {
                // 计算旋转角度（点积的反余弦）
                double angle = acos(direction.dot(axis_z));
                rotation_axis.normalize();
                q.setRotation(rotation_axis, angle);
            }

            marker.pose.orientation.x = q.x();
            marker.pose.orientation.y = q.y();
            marker.pose.orientation.z = q.z();
            marker.pose.orientation.w = q.w();

            // 根据磁场强度设置箭头大小
            const double base_length = 0.03;     // 基础长度
            const double scale_factor = 0.00002;  // 缩放因子，可以根据实际数据调整
            
            // 设置箭头大小，长度随磁场强度变化
            marker.scale.x = base_length + magnitude * scale_factor;  // 箭头长度
            marker.scale.y = 0.003;  // 箭头宽度
            marker.scale.z = 0.003;  // 箭头高度

            // 根据磁场强度设置颜色
            double normalized_magnitude = std::min(magnitude * 0.001, 1.0);  // 归一化到0-1范围
            marker.color.r = normalized_magnitude;  // 强度越大越红
            marker.color.g = 1.0 - normalized_magnitude;  // 强度越小越绿
            marker.color.b = 0.0;
            marker.color.a = 1.0;
        }
        
        // 设置箭头持续时间
        marker.lifetime = ros::Duration(0.1);
        
        marker_array.markers.push_back(marker);
        marker_pub.publish(marker_array);
    }

    void magnetEstimationCallback(const magnetic_localization::MagnetEstimation::ConstPtr& msg)
    {
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker marker;

        // 设置磁铁标记的基本属性
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "magnet";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        // 设置位置
        marker.pose.position.x = msg->position.x;
        marker.pose.position.y = msg->position.y;
        marker.pose.position.z = msg->position.z;

        // 计算方向四元数
        tf2::Vector3 direction(msg->direction.x,
                               msg->direction.y,
                               msg->direction.z);
        direction.normalize();

        tf2::Vector3 axis_z(0, 0, 1);
        tf2::Vector3 rotation_axis = axis_z.cross(direction);
        
        tf2::Quaternion q;
        if (rotation_axis.length2() < 1e-6) {
            if (direction.z() > 0) {
                q.setRPY(0, 0, 0);
            } else {
                q.setRPY(M_PI, 0, 0);
            }
        } else {
            double angle = acos(direction.dot(axis_z));
            rotation_axis.normalize();
            q.setRotation(rotation_axis, angle);
        }

        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        // 设置磁铁箭头的大小
        marker.scale.x = 0.1;  // 长度
        marker.scale.y = 0.02; // 宽度
        marker.scale.z = 0.02; // 高度

        // 设置磁铁标记的颜色（红色）
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        // 设置持续时间
        marker.lifetime = ros::Duration(0.1);

        marker_array.markers.push_back(marker);
        marker_pub.publish(marker_array);
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "magnetic_visualizer");
    MagneticVisualizer visualizer;
    ros::spin();
    return 0;
}