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
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <thread>

std::thread keyboard_thread_;

class CalibrationNode
{
public:
  CalibrationNode(ros::NodeHandle &nh)
      : it_(nh), tf_listener_(tf_buffer_)
  {
    tag_sub_ = nh.subscribe(
        "/tag_detections", 1,
        &CalibrationNode::tagCallback, this);

    // rgb_sub_ = it_.subscribe(
    //     "/camera/color/image_raw", 1,
    //     &CalibrationNode::rgbCallback, this);

    // depth_sub_ = it_.subscribe(
    //     "/camera/aligned_depth_to_color/image_raw", 1,
    //     &CalibrationNode::depthCallback, this);
    keyboard_thread_ = std::thread(&CalibrationNode::keyboardLoop, this);

    ROS_INFO("CalibrationNode initialized.");
    ROS_INFO("Press 'c' to trigger calibration, 'r' to reset.");
  }

  ~CalibrationNode()
  {
    cv::destroyAllWindows();
  }

private:
  ros::Subscriber tag_sub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber rgb_sub_;
  image_transport::Subscriber depth_sub_;

  bool trigger_calibration_ = false;
  bool calibrated_once_ = false;

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

  void keyboardLoop()
  {
    while (ros::ok())
    {
      char c;
      std::cin >> c;

      if (c == 'c')
      {
        trigger_calibration_ = true;
        calibrated_once_ = false;
        ROS_WARN("Calibration triggered. Waiting for stable detection...");
      }
      else if (c == 'r')
      {
        calibrated_once_ = false;
        trigger_calibration_ = false;
        ROS_WARN("Calibration reset.");
      }
    }
  }

  int valid_count_ = 0;
  /* ================= Tag Callback ================= */

  void printTf(const std::string &name, const tf2::Transform &T) const
  {
    tf2::Vector3 t = T.getOrigin();
    tf2::Quaternion q = T.getRotation();
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
  }

  void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
  {
    if (!trigger_calibration_){
      return;
    }
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
    printTf("CAMERA -> DIANA_EE", T_cam_diana_ee);
    printTf("CAMERA -> FR1_EE", T_cam_fr1_ee);
    printTf("CAMERA -> FR2_EE", T_cam_fr2_ee);

    try
    {
      // --------------------------------------------------
      // 3. 从 TF 中获取 T_base_ee
      // --------------------------------------------------
      ros::Time stamp = msg->header.stamp;
      auto T_diana_base_ee_msg =
          tf_buffer_.lookupTransform("diana7_pedestal_link", "diana7_tcp", stamp, ros::Duration(0.05));
      auto T_fr1_base_ee_msg =
          tf_buffer_.lookupTransform("arm1_pedestal_link", "arm1_tcp", stamp, ros::Duration(0.05));
      auto T_fr2_base_ee_msg =
          tf_buffer_.lookupTransform("arm2_pedestal_link", "arm2_tcp", stamp, ros::Duration(0.05));
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

      tf2::Transform T_world_fr2_ee = 
          T_diana_base_ee * T_cam_diana_ee.inverse() * T_cam_fr2_ee;
      
      tf2::Transform T_world_fr1_ee = 
          T_diana_base_ee * T_cam_diana_ee.inverse() * T_cam_fr1_ee;        

      // --------------------------------------------------
      // 6. 输出一次标定结果
      // --------------------------------------------------

      printTf("WORLD -> FR1_BASE", T_world_fr1_base);
      printTf("WORLD -> FR2_BASE", T_world_fr2_base);

      // Broadcast EE transforms
      auto broadcastTf = [&](const tf2::Transform &T, const std::string &parent, const std::string &child) {
        geometry_msgs::TransformStamped ts;
        ts.header.stamp = ros::Time::now();
        ts.header.frame_id = parent;
        ts.child_frame_id = child;
        ts.transform = tf2::toMsg(T);
        tf_broadcaster_.sendTransform(ts);
      };

      broadcastTf(T_world_fr1_ee, "diana7_pedestal_link", "calib_fr1_ee");
      broadcastTf(T_world_fr2_ee, "diana7_pedestal_link", "calib_fr2_ee");

      valid_count_++;
      ROS_INFO_STREAM("Valid calibration count = " << valid_count_);

    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("TF error: %s", ex.what());
      // 只打印相机坐标系中三个tag的位姿
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
