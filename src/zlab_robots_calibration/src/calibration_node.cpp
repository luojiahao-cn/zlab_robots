#include <ros/ros.h>

#include <apriltag_ros/AprilTagDetectionArray.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <vector>
#include <iomanip>
#include <sstream>

struct PoseRPY
{
  ros::Time stamp;
  double x{0}, y{0}, z{0};
  double roll{0}, pitch{0}, yaw{0};
  bool valid{false};
};

class CalibrationNode
{
public:
  CalibrationNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh),
    pnh_(pnh),
    it_(nh_)
  {
    // -------- Parameters --------
    pnh_.param<std::string>("image_topic", image_topic_, std::string("/camera/color/image_raw"));
    pnh_.param<std::string>("detections_topic", detections_topic_, std::string("/apriltag_ros/tag_detections"));
    pnh_.param<std::string>("window_name", window_name_, std::string("Calibration View (RGB + AprilTag Pose)"));
    pnh_.param<bool>("show_window", show_window_, true);
    pnh_.param<bool>("overlay_pose", overlay_pose_, true);
    pnh_.param<bool>("print_to_console", print_to_console_, true);
    pnh_.param<double>("stale_timeout_sec", stale_timeout_sec_, 0.5); // 超过这个时间没更新，就认为pose过期

    // tag_ids: 你可以在 launch/yaml 里配置，比如 [0,1,2,3]
    std::vector<int> tag_ids;
    if (pnh_.getParam("tag_ids", tag_ids))
    {
      for (int id : tag_ids) watch_ids_.insert(id);
      watch_all_ids_ = false;
    }
    else
    {
      // 没配置就默认“不过滤”，检测到谁就显示谁
      watch_all_ids_ = true;
    }

    // -------- Subscribers --------
    det_sub_ = nh_.subscribe(detections_topic_, 5, &CalibrationNode::detectionsCallback, this);
    img_sub_ = it_.subscribe(image_topic_, 5, &CalibrationNode::imageCallback, this);

    if (show_window_)
    {
      cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
      cv::resizeWindow(window_name_, 1280, 720);
    }

    ROS_INFO_STREAM("[calibration_node] Started."
                    << " image_topic=" << image_topic_
                    << " detections_topic=" << detections_topic_
                    << " show_window=" << (show_window_ ? "true" : "false")
                    << " overlay_pose=" << (overlay_pose_ ? "true" : "false")
                    << " print_to_console=" << (print_to_console_ ? "true" : "false")
                    << " watch_all_ids=" << (watch_all_ids_ ? "true" : "false"));
    if (!watch_all_ids_)
    {
      std::ostringstream oss;
      oss << "[calibration_node] Watching tag IDs: ";
      for (auto id : watch_ids_) oss << id << " ";
      ROS_INFO_STREAM(oss.str());
    }
  }

  ~CalibrationNode()
  {
    if (show_window_) cv::destroyWindow(window_name_);
  }

private:
  static PoseRPY toPoseRPY(const geometry_msgs::Pose& pose, const ros::Time& stamp)
  {
    PoseRPY out;
    out.stamp = stamp;

    out.x = pose.position.x;
    out.y = pose.position.y;
    out.z = pose.position.z;

    tf2::Quaternion q(pose.orientation.x,
                      pose.orientation.y,
                      pose.orientation.z,
                      pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(out.roll, out.pitch, out.yaw);

    out.valid = true;
    return out;
  }

  static std::string formatPoseLine(int id, const PoseRPY& p)
  {
    auto deg = [](double rad) { return rad * 180.0 / M_PI; };

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4);
    oss << "Tag " << id
        << " | xyz(m)=[" << p.x << ", " << p.y << ", " << p.z << "]"
        << " rpy(deg)=[" << deg(p.roll) << ", " << deg(p.pitch) << ", " << deg(p.yaw) << "]";
    return oss.str();
  }

  bool shouldWatch(int id) const
  {
    return watch_all_ids_ || (watch_ids_.find(id) != watch_ids_.end());
  }

  bool isStale(const PoseRPY& p, const ros::Time& now) const
  {
    if (!p.valid) return true;
    return (now - p.stamp).toSec() > stale_timeout_sec_;
  }

  void detectionsCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);

    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;

    // 更新本帧出现的 tag
    for (const auto& det : msg->detections)
    {
      if (det.id.empty()) continue;
      const int id = det.id[0];
      if (!shouldWatch(id)) continue;

      // apriltag_ros: det.pose.pose.pose 是 geometry_msgs/Pose
      PoseRPY p = toPoseRPY(det.pose.pose.pose, stamp);
      latest_pose_by_id_[id] = p;
    }

    // 控制台输出（节流）
    if (print_to_console_)
    {
      // 这里用节流，避免刷屏太猛
      ROS_INFO_STREAM_THROTTLE(0.2, "[calibration_node] detections updated. (throttled)");
      // 如果你更希望每帧都打印，把上面一行删掉，改成下面这种逐个打印：
      // for (auto& kv : latest_pose_by_id_) ROS_INFO_STREAM(formatPoseLine(kv.first, kv.second));
    }
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    }
    catch (const cv_bridge::Exception& e)
    {
      ROS_ERROR_STREAM("[calibration_node] cv_bridge exception: " << e.what());
      return;
    }

    cv::Mat frame = cv_ptr->image.clone();
    const ros::Time now = ros::Time::now();

    // 把最新位姿叠加到图像上 + 同时输出更详细的每tag信息（节流）
    std::vector<std::pair<int, PoseRPY>> poses_snapshot;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      poses_snapshot.reserve(latest_pose_by_id_.size());
      for (const auto& kv : latest_pose_by_id_)
      {
        if (!shouldWatch(kv.first)) continue;
        poses_snapshot.push_back(kv);
      }
    }

    // 在控制台输出每个 tag 的 xyz+rpy（节流）
    if (print_to_console_ && !poses_snapshot.empty())
    {
      // 5Hz 打印一次比较舒服；你可以改成 0.1 更频繁
      static ros::Time last_print(0);
      if ((now - last_print).toSec() > 0.2)
      {
        last_print = now;
        for (const auto& kv : poses_snapshot)
        {
          const int id = kv.first;
          const PoseRPY& p = kv.second;
          if (isStale(p, now)) continue;
          ROS_INFO_STREAM(formatPoseLine(id, p));
        }
      }
    }

    if (overlay_pose_)
    {
      int y = 30;
      const int line_h = 26;

      cv::putText(frame, "Calibration Node: AprilTag Pose (xyz + rpy)", cv::Point(10, y),
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
      y += line_h;

      if (poses_snapshot.empty())
      {
        cv::putText(frame, "No tag detections yet.", cv::Point(10, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
      }
      else
      {
        for (const auto& kv : poses_snapshot)
        {
          const int id = kv.first;
          const PoseRPY& p = kv.second;

          std::string line;
          if (isStale(p, now))
          {
            std::ostringstream oss;
            oss << "Tag " << id << " | STALE (no update > " << stale_timeout_sec_ << "s)";
            line = oss.str();
            cv::putText(frame, line, cv::Point(10, y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
          }
          else
          {
            line = formatPoseLine(id, p);
            cv::putText(frame, line, cv::Point(10, y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 2);
          }
          y += line_h;
          if (y > frame.rows - 10) break;
        }
      }
    }

    if (show_window_)
    {
      cv::imshow(window_name_, frame);
      // 1ms waitKey 是必要的，否则窗口不会刷新
      const int key = cv::waitKey(1);

      // 按 q 或 ESC 退出节点（可选）
      if (key == 'q' || key == 27)
      {
        ROS_WARN("[calibration_node] Quit requested from window. Shutting down...");
        ros::shutdown();
      }
    }
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;

  ros::Subscriber det_sub_;
  image_transport::Subscriber img_sub_;

  std::mutex mtx_;
  std::unordered_map<int, PoseRPY> latest_pose_by_id_;

  std::unordered_set<int> watch_ids_;
  bool watch_all_ids_{true};

  std::string image_topic_;
  std::string detections_topic_;
  std::string window_name_;

  bool show_window_{true};
  bool overlay_pose_{true};
  bool print_to_console_{true};
  double stale_timeout_sec_{0.5};
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibration_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  CalibrationNode node(nh, pnh);
  ros::spin();
  return 0;
}
