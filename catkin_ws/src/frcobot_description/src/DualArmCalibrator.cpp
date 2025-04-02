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
        ROS_INFO("已订阅AprilTag话题: %s", apriltag_topic_.c_str());

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
        if (1)
        {
            ROS_INFO("跳过移动机械臂到标定位姿");
            return true;
        }
        
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

        // 设置规划时间
        left_arm_group_->setPlanningTime(2.0);
        // 设置左臂目标位姿
        left_arm_group_->setPoseTarget(left_pose);

        // 规划并执行左臂移动
        moveit::planning_interface::MoveGroupInterface::Plan left_plan;
        if (!left_arm_group_->plan(left_plan))
        {
            ROS_ERROR("左臂运动规划失败");
            return false;
        }
        if (!left_arm_group_->execute(left_plan))
        {
            ROS_ERROR("左臂运动执行失败");
            // return false;
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
            // return false;
        }

        ROS_INFO("双臂已移动到标定位姿");
        return true;
    }

    // 执行标定
    bool calibrate()
    {
        ROS_INFO("开始双机械臂标定...");

        // 移动机械臂到标定位置
        if (!moveArmsToCalibrationPose())
        {
            ROS_ERROR("移动机械臂到标定位姿失败");
            return false;
        }

        // 清空之前的测量结果
        measurements_.clear();
        
        ROS_INFO("开始持续检测，将在%.1f秒内每隔%.1f秒进行一次检测...", 
                 CALIBRATION_DURATION, DETECTION_INTERVAL);
        
        ros::Time start_time = ros::Time::now();
        ros::Rate rate(1.0/DETECTION_INTERVAL); // 2Hz，每0.5秒一次

        while (ros::ok())
        {
            // 检查是否超时
            if ((ros::Time::now() - start_time).toSec() > CALIBRATION_DURATION)
            {
                ROS_INFO("检测时间已到");
                break;
            }

            // 等待新的AprilTag检测结果
            ros::spinOnce();
            
            // 添加调试信息
            ROS_INFO_THROTTLE(1.0, "左标签状态: %s, 右标签状态: %s", 
                            left_tag_detected_ ? "已检测" : "未检测",
                            right_tag_detected_ ? "已检测" : "未检测");
            
            if (!left_tag_detected_ || !right_tag_detected_)
            {
                ROS_WARN_THROTTLE(1.0, "未检测到标签，继续尝试...");
                rate.sleep();
                continue;
            }
            
            // 计算两个tag之间的相对位置
            tf2::Transform transform = calculateRelativeTransform();
            
            // 打印当前测量的相对位置
            ROS_INFO("\n========== 第 %lu 次测量结果 ==========", measurements_.size() + 1);
            printCalibrationResults(transform);
            ROS_INFO("=====================================\n");
            
            // 保存测量结果
            measurements_.push_back(transform);
            
            rate.sleep();
        }

        // 检查是否获得足够的测量数据
        if (measurements_.size() < 10)  // 至少需要10次有效测量
        {
            ROS_ERROR("有效测量数据不足，标定失败。仅获得 %lu 次测量", measurements_.size());
            return false;
        }

        // 计算平均变换
        tf2::Transform average_transform = calculateAverageTransform(measurements_);

        // 输出平均结果
        ROS_INFO("总计完成%lu次有效测量", measurements_.size());
        printCalibrationResults(average_transform);
        
        // 获取两个机械臂末端变换
        geometry_msgs::TransformStamped left_base_to_tool_transform, right_base_to_tool_transform;
        if (!getArmEndEffectorTransforms(left_base_to_tool_transform, right_base_to_tool_transform)) {
            return false;
        }
        
        // 计算并打印两个机械臂基座之间的变换
        tf2::Transform left_to_right_base = calculateBaseTransform(
            average_transform, left_base_to_tool_transform, right_base_to_tool_transform);

        printCalibrationResults(left_to_right_base);

        // 保存标定结果
        saveCalibrationResults(left_to_right_base);

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

        // 获取测量次数
        if (!nh_.getParam("required_measurements", required_measurements_))
        {
            required_measurements_ = 10;
            ROS_WARN("使用默认测量次数: %d", required_measurements_);
        }

        // 获取测量间隔
        if (!nh_.getParam("measurement_interval", measurement_interval_))
        {
            measurement_interval_ = 0.5;
            ROS_WARN("使用默认测量间隔: %.1f秒", measurement_interval_);
        }
    }

    // AprilTag检测回调函数
    void apriltagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
    {
        ROS_INFO_THROTTLE(1.0, "收到AprilTag检测结果，检测到 %lu 个标签", msg->detections.size());
        
        // 重置检测状态
        left_tag_detected_ = false;
        right_tag_detected_ = false;

        for (const auto &detection : msg->detections)
        {
            ROS_INFO_THROTTLE(1.0, "检测到ID为 %d 的标签", detection.id[0]);
            
            // 检查是否是左臂tag
            if (detection.id[0] == left_arm_tag_id_)
            {
                left_tag_pose_.header = detection.pose.header;
                left_tag_pose_.pose = detection.pose.pose.pose;
                left_tag_detected_ = true;
                ROS_INFO_THROTTLE(1.0, "识别到左臂标签 (ID: %d)", left_arm_tag_id_);
            }
            // 检查是否是右臂tag
            else if (detection.id[0] == right_arm_tag_id_)
            {
                right_tag_pose_.header = detection.pose.header;
                right_tag_pose_.pose = detection.pose.pose.pose;
                right_tag_detected_ = true;
                ROS_INFO_THROTTLE(1.0, "识别到右臂标签 (ID: %d)", right_arm_tag_id_);
            }
        }
    }

    // 计算两个tag之间的相对变换
    tf2::Transform calculateRelativeTransform()
    {
        // 使用TF系统查询从左标签到右标签的变换
        tf2::Transform left_to_right;
        try
        {
            // 将标签ID转换为TF框架名称
            std::string left_frame = "tag_" + std::to_string(left_arm_tag_id_);
            std::string right_frame = "tag_" + std::to_string(right_arm_tag_id_);

            // 等待一段时间确保TF树更新
            if (!tf_buffer_.canTransform(left_frame, right_frame, ros::Time(0), ros::Duration(1.0)))
            {
                ROS_WARN("无法找到从 %s 到 %s 的变换，使用手动计算", left_frame.c_str(), right_frame.c_str());

                // 回退到手动计算
                tf2::Transform left_tag_tf, right_tag_tf;
                tf2::fromMsg(left_tag_pose_.pose, left_tag_tf);
                tf2::fromMsg(right_tag_pose_.pose, right_tag_tf);
                return left_tag_tf.inverse() * right_tag_tf;
            }

            // 查询TF变换
            geometry_msgs::TransformStamped transform_stamped =
                tf_buffer_.lookupTransform(left_frame, right_frame, ros::Time(0));

            // 转换为tf2::Transform
            tf2::fromMsg(transform_stamped.transform, left_to_right);

            ROS_INFO("使用TF系统获取的标签间变换");
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("TF查询异常: %s，使用手动计算", ex.what());

            // 回退到手动计算
            tf2::Transform left_tag_tf, right_tag_tf;
            tf2::fromMsg(left_tag_pose_.pose, left_tag_tf);
            tf2::fromMsg(right_tag_pose_.pose, right_tag_tf);
            left_to_right = left_tag_tf.inverse() * right_tag_tf;
        }

        return left_to_right;
    }

    // 打印标定结果
    void printCalibrationResults(const tf2::Transform &transform)
    {
        tf2::Vector3 translation = transform.getOrigin();
        tf2::Quaternion rotation = transform.getRotation();

        ROS_INFO("相对坐标结果:");
        ROS_INFO("平移: [%.3f, %.3f, %.3f] 米",
                 translation.x(), translation.y(), translation.z());
        ROS_INFO("旋转四元数: [%.3f, %.3f, %.3f, %.3f]",
                 rotation.x(), rotation.y(), rotation.z(), rotation.w());

        // 计算相对距离
        double distance = translation.length();
        ROS_INFO("相对距离: %.3f 米", distance);

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

    // 获取两个机械臂末端变换
    bool getArmEndEffectorTransforms(geometry_msgs::TransformStamped &left_transform, geometry_msgs::TransformStamped &right_transform)
    {
        left_transform = tf_buffer_.lookupTransform("robot1_base2_link", "robot1_tool_link", ros::Time(0));
        right_transform = tf_buffer_.lookupTransform("robot2_base2_link", "robot2_tool_link", ros::Time(0));
        return true;
    }

    // 计算两个机械臂基座之间的变换
    tf2::Transform calculateBaseTransform(const tf2::Transform &left_to_right_transform,
                                          const geometry_msgs::TransformStamped &left_base_to_tool_transform,
                                          const geometry_msgs::TransformStamped &right_base_to_tool_transform)
    {
        tf2::Transform left_base_to_tool, right_base_to_tool;
        tf2::fromMsg(left_base_to_tool_transform.transform, left_base_to_tool);
        tf2::fromMsg(right_base_to_tool_transform.transform, right_base_to_tool);

        // return left_to_right_transform * right_base_to_tool.inverse() * left_base_to_tool;
        return left_base_to_tool * left_to_right_transform * right_base_to_tool.inverse();
    }

    // 计算平均变换
    tf2::Transform calculateAverageTransform(const std::vector<tf2::Transform>& transforms)
    {
        if (transforms.empty()) {
            return tf2::Transform::getIdentity();
        }

        // 平均位置
        tf2::Vector3 avg_translation(0, 0, 0);
        
        // 四元数平均
        double avg_x = 0, avg_y = 0, avg_z = 0, avg_w = 0;

        for (const auto& transform : transforms)
        {
            // 累加位置
            avg_translation += transform.getOrigin();
            
            // 累加四元数
            tf2::Quaternion q = transform.getRotation();
            avg_x += q.x();
            avg_y += q.y();
            avg_z += q.z();
            avg_w += q.w();
        }

        // 计算平均位置
        avg_translation /= transforms.size();

        // 计算平均四元数并归一化
        tf2::Quaternion avg_rotation(
            avg_x / transforms.size(),
            avg_y / transforms.size(),
            avg_z / transforms.size(),
            avg_w / transforms.size()
        );
        avg_rotation.normalize();

        // 构造平均变换
        tf2::Transform avg_transform;
        avg_transform.setOrigin(avg_translation);
        avg_transform.setRotation(avg_rotation);

        return avg_transform;
    }

    // 等待按键输入
    bool waitForSpaceKey()
    {
        std::cout << "请按空格键进行下一次测量..." << std::endl;
        char key;
        while (ros::ok())
        {
            key = cv::waitKey(100);  // 100ms超时
            if (key == ' ')
            {
                return true;
            }
            else if (key == 27)  // ESC键
            {
                return false;
            }
            ros::spinOnce();
        }
        return false;
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

    // 添加变量用于存储多次测量结果
    std::vector<tf2::Transform> measurements_;
    int required_measurements_ = 10;  // 需要的测量次数
    double measurement_interval_ = 0.5;  // 测量间隔(秒)

    const double CALIBRATION_DURATION = 60.0; // 标定持续时间(秒)
    const double DETECTION_INTERVAL = 0.5;    // 检测间隔(秒)
};

int main(int argc, char **argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "dual_arm_calibrator");
    ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

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