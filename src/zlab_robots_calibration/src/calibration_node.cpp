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
      : it_(nh)
  {
    tag_sub_ = nh.subscribe(
        "/tag_detections", 1,
        &CalibrationNode::tagCallback, this);

    // rgb_sub_ = it_.subscribe(
    //     "/camera/color/image_raw", 1,
    //     &CalibrationNode::rgbCallback, this);

    depth_sub_ = it_.subscribe(
        "/camera/aligned_depth_to_color/image_raw", 1,
        &CalibrationNode::depthCallback, this);

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
};


  /* ================= Tag Callback ================= */

  void tagCallback(
      const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
  {
    latest_detections_ = *msg;

    for (const auto &det : msg->detections)
    {
      int id = det.id[0];

      auto it = id_to_frame_.find(id);
      std::string frame_name =
          (it != id_to_frame_.end()) ? it->second : "unknown_tag";

      const auto &p = det.pose.pose.pose.position;
      const auto &q = det.pose.pose.pose.orientation;

      tf2::Quaternion quat(q.x, q.y, q.z, q.w);
      tf2::Matrix3x3 m(quat);

      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      ROS_INFO_STREAM(
          "[" << frame_name << "] "
          << "xyz = ["
          << p.x << ", "
          << p.y << ", "
          << p.z << "]  "
          << "rpy = ["
          << roll << ", "
          << pitch << ", "
          << yaw << "]");
    }
    // 标定流程，循环100次求平均：
    // 确保连续接受到的三次detections里都有diana，fr1，fr2三个tag
    // 订阅三对tf，分别对应diana，fr1，fr2
    // 然后根据diana的末端位姿（apriltag diana）反解diana的基座
    // 再根据fr1，fr2的apriltag位姿反解各自的基座
    // 以diana基座为世界坐标系，计算fr1，fr2基座在世界坐标系下的位姿
  }

  void depthCallback(const sensor_msgs::ImageConstPtr &msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvShare(
          msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("Depth cv_bridge error: %s", e.what());
      return;
    }

    latest_depth_ = cv_ptr->image.clone();
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
