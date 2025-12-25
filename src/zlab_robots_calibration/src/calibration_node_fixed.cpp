
// filepath: /home/chen/Downloads/zlab_robots-master/sss.cpp
#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>

#include <apriltag_ros/AprilTagDetectionArray.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class CalibrationNode
{
public:
  CalibrationNode(ros::NodeHandle &nh)
      : it_(nh), tf_listener_(tf_buffer_)
  {
    tag_sub_ = nh.subscribe(
        "/tag_detections", 1,
        &CalibrationNode::tagCallback, this);

    cv::namedWindow("Calibration View", cv::WINDOW_NORMAL);
    ROS_INFO("CalibrationNode initialized.");
  }

  ~CalibrationNode()
  {
    cv::destroyAllWindows();
  }

private:
  ros::Subscriber tag_sub_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber rgb_sub_;
  image_transport::Subscriber depth_sub_;

  cv::Mat latest_rgb_;
  cv::Mat latest_depth_;
  apriltag_ros::AprilTagDetectionArray latest_detections_;

  std::unordered_map<int, std::string> id_to_frame_ = {
  {109, "diana"},
  {208, "fr2"},
  {104, "fr1"} //tag_green
  };

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  int valid_count_ = 0;
  const int MAX_COUNT = 100;

  /* ================= Tag Callback ================= */

  void tagCallback(
      const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
  {
    // -------------------------------
    // 1. 检查是否同时检测到 3 个 tag
    // -------------------------------
    bool has_diana = false, has_fr1 = false, has_fr2 = false;

    geometry_msgs::Pose diana_pose, fr1_pose, fr2_pose;

    for (const auto &det : msg->detections)
    {
      int id = det.id[0];
      if (id == 109)  // diana
      {
        diana_pose = det.pose.pose.pose;
        has_diana = true;
      }
      else if (id == 104) // fr1
      {
        fr1_pose = det.pose.pose.pose;
        has_fr1 = true;
      }
      else if (id == 208) // fr2
      {
        fr2_pose = det.pose.pose.pose;
        has_fr2 = true;
      }
    }

    if (!(has_diana && has_fr1 && has_fr2))
      return;  // 必须三者同时存在

    // ----------------------------------------
    // 2. 构造 T_camera_tag (apriltag 输出)
    // ----------------------------------------
    
    auto poseToTf = [](const geometry_msgs::Pose &p)
    {
      tf2::Transform T;
      T.setOrigin(tf2::Vector3(p.position.x,
                              p.position.y,
                              p.position.z));
      T.setRotation(tf2::Quaternion(
          p.orientation.x,
          p.orientation.y,
          p.orientation.z,
          p.orientation.w));
      return T;
    };

    // 输出三个tag之间的欧式距离
    tf2::Vector3 diana_pos(diana_pose.position.x, diana_pose.position.y, diana_pose.position.z);
    tf2::Vector3 fr1_pos(fr1_pose.position.x, fr1_pose.position.y, fr1_pose.position.z);
    tf2::Vector3 fr2_pos(fr2_pose.position.x, fr2_pose.position.y, fr2_pose.position.z);
    double d_diana_fr1 = (diana_pos - fr1_pos).length();
    double d_diana_fr2 = (diana_pos - fr2_pos).length();
    double d_fr1_fr2   = (fr1_pos - fr2_pos).length();
    ROS_INFO_STREAM("Tag distances (m): diana-fr1 = " << d_diana_fr1
                     << ", diana-fr2 = " << d_diana_fr2
                     << ", fr1-fr2 = " << d_fr1_fr2);

    tf2::Transform T_cam_diana_ee = poseToTf(diana_pose);
    tf2::Transform T_cam_fr1_ee   = poseToTf(fr1_pose);
    tf2::Transform T_cam_fr2_ee   = poseToTf(fr2_pose);

    try
    {
      // --------------------------------------------------
      // 3. 从 TF 中获取 T_base_ee
      // --------------------------------------------------
      
      // [修改开始] 使用消息时间戳并等待 TF
      ros::Time lookup_time = msg->header.stamp;
      ros::Duration timeout(0.1);

      if (!tf_buffer_.canTransform("diana7_pedestal_link", "diana7_tcp", lookup_time, timeout) ||
          !tf_buffer_.canTransform("arm1_base0_link", "arm1_tcp", lookup_time, timeout) ||
          !tf_buffer_.canTransform("arm2_base0_link", "arm2_tcp", lookup_time, timeout))
      {
          // 如果这一帧的 TF 还没到，就跳过，不报错
          return;
      }

      auto T_diana_base_ee_msg =
          tf_buffer_.lookupTransform("diana7_pedestal_link", "diana7_tcp", lookup_time);
      auto T_fr1_base_ee_msg =
          tf_buffer_.lookupTransform("arm1_base0_link", "arm1_tcp", lookup_time);
      auto T_fr2_base_ee_msg =
          tf_buffer_.lookupTransform("arm2_base0_link", "arm2_tcp", lookup_time);
      // [修改结束]

      tf2::Transform T_diana_base_ee, T_fr1_base_ee, T_fr2_base_ee;
      tf2::fromMsg(T_diana_base_ee_msg.transform, T_diana_base_ee);
      tf2::fromMsg(T_fr1_base_ee_msg.transform,   T_fr1_base_ee);
      tf2::fromMsg(T_fr2_base_ee_msg.transform,   T_fr2_base_ee);

      // --------------------------------------------------
      // 4. 反解 T_camera_base
      //
      // T_camera_base = T_camera_ee * inverse(T_base_ee)
      // --------------------------------------------------
      tf2::Transform T_cam_diana_base =
          T_cam_diana_ee * T_diana_base_ee.inverse();

      tf2::Transform T_cam_fr1_base =
          T_cam_fr1_ee * T_fr1_base_ee.inverse();

      tf2::Transform T_cam_fr2_base =
          T_cam_fr2_ee * T_fr2_base_ee.inverse();

      // --------------------------------------------------
      // 5. 以 diana_base 为 world
      //
      // T_world_frX_base =
      // inverse(T_camera_diana_base) * T_camera_frX_base
      // --------------------------------------------------
      tf2::Transform T_world_fr1_base =
          T_cam_diana_base.inverse() * T_cam_fr1_base;

      tf2::Transform T_world_fr2_base =
          T_cam_diana_base.inverse() * T_cam_fr2_base;

      // --------------------------------------------------
      // 6. 输出一次标定结果
      // --------------------------------------------------
      auto printTf = [](const std::string &name,
                        const tf2::Transform &T)
      {
        auto t = T.getOrigin();
        auto q = T.getRotation();
        double r, p, y;
        tf2::Matrix3x3(q).getRPY(r, p, y);

        ROS_INFO_STREAM(
            name << " xyz = ["
                << t.x() << ", "
                << t.y() << ", "
                << t.z() << "]  rpy = ["
                << r << ", "
                << p << ", "
                << y << "]");
      };

      printTf("WORLD -> FR1_BASE", T_world_fr1_base);
      printTf("WORLD -> FR2_BASE", T_world_fr2_base);

      valid_count_++;
      ROS_INFO_STREAM("Valid calibration count = " << valid_count_);

      if (valid_count_ >= MAX_COUNT)
      {
        ROS_WARN("Calibration finished. You can stop the node.");
      }
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("TF error: %s", ex.what());
    }
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calibration_node");
  ros::NodeHandle nh;

  CalibrationNode node(nh);

  ros::spin();
  return 0;
}