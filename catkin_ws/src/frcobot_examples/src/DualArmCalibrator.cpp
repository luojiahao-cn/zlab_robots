#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

class DualArmCalibrator
{
public:
    DualArmCalibrator(ros::NodeHandle &nh) : nh_(nh)
    {
        // 加载参数
        loadParameters();

        // 订阅 AprilTag 检测结果
        tag_sub_ = nh_.subscribe(apriltag_topic_, 1,
                                 &DualArmCalibrator::apriltagCallback, this);

        // 设置移动组接口
        left_arm_group_ = new moveit::planning_interface::MoveGroupInterface(left_arm_group_name_);
        right_arm_group_ = new moveit::planning_interface::MoveGroupInterface(right_arm_group_name_);

        // 初始化 TF 监听器
        tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);

        ROS_INFO("DualArmCalibrator 初始化完成");
    }

    ~DualArmCalibrator()
    {
        if (left_arm_group_)
            delete left_arm_group_;
        if (right_arm_group_)
            delete right_arm_group_;
        if (tf_listener_)
            delete tf_listener_;
    }

    // 移动机械臂到标定位置
    bool moveArmsToCalibrationPose()
    {
        // 设置左臂标定位姿
        geometry_msgs::PoseStamped left_pose;
        left_pose.header.frame_id = "robot1_base2_link";
        left_pose.pose.position.x = left_calibration_pose_[0];
        left_pose.pose.position.y = left_calibration_pose_[1];
        left_pose.pose.position.z = left_calibration_pose_[2];
        left_pose.pose.orientation.x = left_calibration_pose_[3];
        left_pose.pose.orientation.y = left_calibration_pose_[4];
        left_pose.pose.orientation.z = left_calibration_pose_[5];
        left_pose.pose.orientation.w = left_calibration_pose_[6];

        // 设置右臂标定位姿
        geometry_msgs::PoseStamped right_pose;
        right_pose.header.frame_id = "robot2_base2_link";
        right_pose.pose.position.x = right_calibration_pose_[0];
        right_pose.pose.position.y = right_calibration_pose_[1];
        right_pose.pose.position.z = right_calibration_pose_[2];
        right_pose.pose.orientation.x = right_calibration_pose_[3];
        right_pose.pose.orientation.y = right_calibration_pose_[4];
        right_pose.pose.orientation.z = right_calibration_pose_[5];
        right_pose.pose.orientation.w = right_calibration_pose_[6];

        ROS_INFO("正在移动机械臂到标定位姿...");

        // 设置左臂目标位姿
        left_arm_group_->setPoseTarget(left_pose);

        // 规划并执行左臂移动
        moveit::planning_interface::MoveGroupInterface::Plan left_plan;
        bool success = (left_arm_group_->plan(left_plan)) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
        if (!left_arm_group_->plan(left_plan))
        {
            ROS_ERROR("左臂运动规划失败");
            return false;
        }
        if (!left_arm_group_->execute(left_plan))
        {
            ROS_ERROR("左臂运动执行失败");
            return false;
        }

        // 设置右臂目标位姿
        right_arm_group_->setPoseTarget(right_pose);

        // 规划并执行右臂移动
        moveit::planning_interface::MoveGroupInterface::Plan right_plan;
        if (!right_arm_group_->plan(right_plan))
        {
            ROS_ERROR("右臂运动规划失败");
            return false;
        }
        if (!right_arm_group_->execute(right_plan))
        {
            ROS_ERROR("右臂运动执行失败");
            return false;
        }

        ROS_INFO("双臂已移动到标定位姿");
        return true;
    }

    // 执行标定
    bool calibrate()
    {
        ROS_INFO("开始双机械臂标定...");

        // 先移动机械臂到标定位置
        if (!moveArmsToCalibrationPose())
        {
            ROS_ERROR("移动机械臂到标定位姿失败");
            return false;
        }

        // 等待 AprilTag 检测结果
        ROS_INFO("等待AprilTag检测结果...");
        ros::Duration(2.0).sleep(); // 给相机一些时间检测tag

        // 检查是否已经检测到两个tag
        if (!left_tag_detected_ || !right_tag_detected_)
        {
            ROS_ERROR("未能检测到两个AprilTag。左臂: %s, 右臂: %s",
                      left_tag_detected_ ? "已检测" : "未检测",
                      right_tag_detected_ ? "已检测" : "未检测");
            return false;
        }

        // 计算两个tag之间的相对位置
        tf2::Transform left_to_right_transform = calculateRelativeTransform();

        // 输出标定结果
        printCalibrationResults(left_to_right_transform);

        // 保存标定结果
        saveCalibrationResults(left_to_right_transform);

        ROS_INFO("标定成功完成");
        return true;
    }

private:
    // 加载参数
    void loadParameters()
    {
        // 获取AprilTag话题
        if (!nh_.getParam("apriltag_topic", apriltag_topic_))
        {
            apriltag_topic_ = "/tag_detections";
            ROS_WARN("使用默认AprilTag话题: %s", apriltag_topic_.c_str());
        }

        // 获取左臂组名称
        if (!nh_.getParam("left_arm_group", left_arm_group_name_))
        {
            left_arm_group_name_ = "left_arm";
            ROS_WARN("使用默认左臂组名称: %s", left_arm_group_name_.c_str());
        }

        // 获取右臂组名称
        if (!nh_.getParam("right_arm_group", right_arm_group_name_))
        {
            right_arm_group_name_ = "right_arm";
            ROS_WARN("使用默认右臂组名称: %s", right_arm_group_name_.c_str());
        }

        // 获取左臂tag ID
        if (!nh_.getParam("left_arm_tag_id", left_arm_tag_id_))
        {
            left_arm_tag_id_ = 0;
            ROS_WARN("使用默认左臂tag ID: %d", left_arm_tag_id_);
        }

        // 获取右臂tag ID
        if (!nh_.getParam("right_arm_tag_id", right_arm_tag_id_))
        {
            right_arm_tag_id_ = 1;
            ROS_WARN("使用默认右臂tag ID: %d", right_arm_tag_id_);
        }

        // 获取左臂标定位姿
        if (!nh_.getParam("left_calibration_pose", left_calibration_pose_))
        {
            // 默认位姿 [x, y, z, qx, qy, qz, qw]
            left_calibration_pose_ = {0.4, 0.1, 0.5, 0.0, 1.0, 0.0, 0.0};
            ROS_WARN("使用默认左臂标定位姿");
        }

        // 获取右臂标定位姿
        if (!nh_.getParam("right_calibration_pose", right_calibration_pose_))
        {
            // 默认位姿 [x, y, z, qx, qy, qz, qw]
            right_calibration_pose_ = {0.4, -0.1, 0.5, 0.0, 1.0, 0.0, 0.0};
            ROS_WARN("使用默认右臂标定位姿");
        }

        // 获取结果保存路径
        if (!nh_.getParam("calibration_result_path", calibration_result_path_))
        {
            calibration_result_path_ = "dual_arm_calibration.yaml";
            ROS_WARN("使用默认标定结果保存路径: %s", calibration_result_path_.c_str());
        }
    }

    // AprilTag检测回调函数
    void apriltagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
    {
        for (const auto &detection : msg->detections)
        {
            // 检查是否是左臂tag
            if (detection.id[0] == left_arm_tag_id_)
            {
                left_tag_pose_.header = detection.pose.header;
                left_tag_pose_.pose = detection.pose.pose.pose;
                left_tag_detected_ = true;
                ROS_INFO_ONCE("左臂AprilTag已检测到");
            }
            // 检查是否是右臂tag
            else if (detection.id[0] == right_arm_tag_id_)
            {
                right_tag_pose_.header = detection.pose.header;
                right_tag_pose_.pose = detection.pose.pose.pose;
                right_tag_detected_ = true;
                ROS_INFO_ONCE("右臂AprilTag已检测到");
            }
        }

        if (left_tag_detected_ && right_tag_detected_)
        {
            ROS_INFO_ONCE("两个AprilTag均已检测到");
        }
    }

    // 计算两个tag之间的相对变换
    tf2::Transform calculateRelativeTransform()
    {
        // 从PoseStamped消息转换为tf2::Transform
        tf2::Transform left_tag_tf, right_tag_tf;

        tf2::fromMsg(left_tag_pose_.pose, left_tag_tf);
        tf2::fromMsg(right_tag_pose_.pose, right_tag_tf);

        // 计算从左tag到右tag的变换
        tf2::Transform left_to_right = left_tag_tf.inverse() * right_tag_tf;

        return left_to_right;
    }

    // 打印标定结果
    void printCalibrationResults(const tf2::Transform &transform)
    {
        tf2::Vector3 translation = transform.getOrigin();
        tf2::Quaternion rotation = transform.getRotation();

        ROS_INFO("标定结果:");
        ROS_INFO("平移: [%.3f, %.3f, %.3f] 米",
                 translation.x(), translation.y(), translation.z());
        ROS_INFO("旋转四元数: [%.3f, %.3f, %.3f, %.3f]",
                 rotation.x(), rotation.y(), rotation.z(), rotation.w());

        // 将四元数转换为RPY (Roll-Pitch-Yaw)
        double roll, pitch, yaw;
        tf2::Matrix3x3(rotation).getRPY(roll, pitch, yaw);

        ROS_INFO("旋转RPY: [%.3f, %.3f, %.3f] 弧度", roll, pitch, yaw);
        ROS_INFO("旋转RPY: [%.1f, %.1f, %.1f] 度",
                 roll * 180 / M_PI, pitch * 180 / M_PI, yaw * 180 / M_PI);
    }

    // 保存标定结果
    void saveCalibrationResults(const tf2::Transform &transform)
    {
        tf2::Vector3 translation = transform.getOrigin();
        tf2::Quaternion rotation = transform.getRotation();

        // 创建YAML文件
        std::ofstream file(calibration_result_path_);
        if (!file.is_open())
        {
            ROS_ERROR("无法打开文件进行写入: %s", calibration_result_path_.c_str());
            return;
        }

        // 写入标定结果
        file << "# 双机械臂标定结果 - 左臂到右臂的变换\n";
        file << "left_to_right_transform:\n";
        file << "  translation:\n";
        file << "    x: " << translation.x() << "\n";
        file << "    y: " << translation.y() << "\n";
        file << "    z: " << translation.z() << "\n";
        file << "  rotation:\n";
        file << "    x: " << rotation.x() << "\n";
        file << "    y: " << rotation.y() << "\n";
        file << "    z: " << rotation.z() << "\n";
        file << "    w: " << rotation.w() << "\n";

        double roll, pitch, yaw;
        tf2::Matrix3x3(rotation).getRPY(roll, pitch, yaw);

        file << "  rpy: # Roll-Pitch-Yaw (弧度)\n";
        file << "    roll: " << roll << "\n";
        file << "    pitch: " << pitch << "\n";
        file << "    yaw: " << yaw << "\n";

        file.close();
        ROS_INFO("标定结果已保存至: %s", calibration_result_path_.c_str());
    }

    // ROS节点句柄
    ros::NodeHandle nh_;

    // ROS订阅器
    ros::Subscriber tag_sub_;

    // MoveIt移动组接口
    moveit::planning_interface::MoveGroupInterface *left_arm_group_;
    moveit::planning_interface::MoveGroupInterface *right_arm_group_;

    // TF缓冲和监听器
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener *tf_listener_;

    // 参数
    std::string apriltag_topic_;
    std::string left_arm_group_name_;
    std::string right_arm_group_name_;
    std::string calibration_result_path_;
    int left_arm_tag_id_;
    int right_arm_tag_id_;
    std::vector<double> left_calibration_pose_;
    std::vector<double> right_calibration_pose_;

    // 检测到的AprilTag位姿
    geometry_msgs::PoseStamped left_tag_pose_;
    geometry_msgs::PoseStamped right_tag_pose_;
    bool left_tag_detected_ = false;
    bool right_tag_detected_ = false;
};

int main(int argc, char **argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "dual_arm_calibrator");
    ros::NodeHandle nh("~");

    ROS_INFO("启动双机械臂标定节点");

    DualArmCalibrator calibrator(nh);

    // 执行标定
    if (calibrator.calibrate())
    {
        ROS_INFO("双机械臂标定成功");
    }
    else
    {
        ROS_ERROR("双机械臂标定失败");
    }

    ros::shutdown();
    return 0;
}