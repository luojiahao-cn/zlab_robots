#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <magnetic_localization/MagneticData.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <vector>

class MagneticVisualizer {
public:
    MagneticVisualizer() {
        // 初始化传感器位置矩阵
        initializeSensorPositions();
        
        // 创建发布者用于发布标记
        marker_pub = nh.advertise<visualization_msgs::MarkerArray>("magnetic_markers", 10);
        // 订阅磁场数据
        magnetic_sub = nh.subscribe("magnetic_data", 1000, 
            &MagneticVisualizer::magneticCallback, this);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher marker_pub;
    ros::Subscriber magnetic_sub;
    
    const int sensor_n = 5;  // 5x5矩阵
    const double sensor_interval = 0.1;  // 间隔(米)
    std::vector<geometry_msgs::Point> sensor_positions;

    void initializeSensorPositions()
    {
        const int sensor_N = sensor_n * sensor_n;
        sensor_positions.resize(sensor_N + 1);

        // 从标签1开始初始化
        for (int label = 1; label <= sensor_N; ++label)
        {
            // 计算行和列（row从0到4，col从0到4）
            int row = (label - 1) / sensor_n; // 行号从上到下为0,1,2,3,4
            int col = (label - 1) % sensor_n; // 列号从左到右为0,1,2,3,4

            // 直接使用标签作为索引存储位置
            sensor_positions[label].x = col * sensor_interval;       // x从左到右增加
            sensor_positions[label].y = (4 - row) * sensor_interval; // y从上到下减少
            sensor_positions[label].z = 0.0;
        }
    }

    void magneticCallback(const magnetic_localization::MagneticData::ConstPtr& msg) {
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker marker;
        
        // 设置箭头标记的基本属性
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "magnetic_field";
        marker.id = msg->sensor_label;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        
        // 使用预计算的传感器位置
        if (msg->sensor_label > 0 && msg->sensor_label <= sensor_n * sensor_n)
        {
            marker.pose.position = sensor_positions[msg->sensor_label];
        }
        else
        {
            ROS_WARN("Invalid sensor label: %d", msg->sensor_label);
            return;
        }

        // 计算箭头方向和大小
        double magnitude = sqrt(pow(msg->x, 2) + pow(msg->y, 2) + pow(msg->z, 2));
        if (magnitude > 0) {
            // 归一化向量
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
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "magnetic_visualizer");
    MagneticVisualizer visualizer;
    ros::spin();
    return 0;
}