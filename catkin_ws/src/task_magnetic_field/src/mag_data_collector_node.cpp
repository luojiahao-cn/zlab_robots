#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <magnetic_pose_estimation/MagneticField.h>
#include <unordered_set>
#include <vector>
#include <mutex>

class MagDataCollectorNode {
public:
  explicit MagDataCollectorNode(ros::NodeHandle &nh) : nh_(nh) {
    // 参数：数据源话题、每轮传感器数量、需要的轮数、采集超时
    nh_.param<std::string>("magnetic_field_topic", magnetic_field_topic_, std::string("/magnetic_field/processed"));
    nh_.param<int>("sensor_count", expected_sensor_count_, 25);
    nh_.param<int>("rounds_needed", rounds_needed_, 2);
    nh_.param<double>("collect_timeout", collect_timeout_sec_, 5.0);

    // 订阅采集触发
    trigger_sub_ = nh_.subscribe("/collect_mag_data", 1, &MagDataCollectorNode::onTrigger, this);
    // 订阅磁场数据流
    magnetic_sub_ = nh_.subscribe(magnetic_field_topic_, 200, &MagDataCollectorNode::onMagneticField, this);
    // 发布采集完成信号
    done_pub_ = nh_.advertise<std_msgs::Bool>("/mag_data_done", 1, false);
    // 定时器用于处理整体采集超时（单次触发周期），初始停止，触发时启动
    timeout_timer_ = nh_.createTimer(ros::Duration(collect_timeout_sec_), &MagDataCollectorNode::onTimeout, this, true /*oneshot*/, false /*autostart*/);

    ROS_INFO_STREAM("MagDataCollectorNode listening trigger on /collect_mag_data, stream="
                    << magnetic_field_topic_ << ", sensor_count=" << expected_sensor_count_
                    << ", rounds_needed=" << rounds_needed_ << ", timeout=" << collect_timeout_sec_ << "s");
  }

private:
  void onTrigger(const std_msgs::Empty::ConstPtr &) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (collecting_) {
      ROS_WARN("Collect already in progress; ignoring trigger");
      return;
    }
    collecting_ = true;
    rounds_completed_ = 0;
    current_round_seen_.clear();
    collected_.clear();
    collected_.resize(rounds_needed_);
    timeout_timer_.setPeriod(ros::Duration(collect_timeout_sec_), true);
    timeout_timer_.start();
    ROS_INFO("Started collection: need %d rounds of %d sensors", rounds_needed_, expected_sensor_count_);
  }

  void onMagneticField(const magnetic_pose_estimation::MagneticField::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!collecting_) return;

    // 记录本轮出现的传感器ID
    current_round_seen_.insert(msg->sensor_id);
    // 保存数据（可按需裁剪体积，这里保留全部样本）
    if (rounds_completed_ < rounds_needed_) {
      collected_[rounds_completed_].push_back(*msg);
    }

    // 判断是否完成一轮
    if ((int)current_round_seen_.size() >= expected_sensor_count_) {
      rounds_completed_++;
      ROS_INFO("Completed round %d/%d", rounds_completed_, rounds_needed_);
      current_round_seen_.clear();
    }

    // 判断是否完成全部轮次
    if (rounds_completed_ >= rounds_needed_) {
      finish(true);
    }
  }

  void onTimeout(const ros::TimerEvent &) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!collecting_) return;
    ROS_WARN("Collection timeout before completing required rounds (got %d/%d)", rounds_completed_, rounds_needed_);
    finish(false);
  }

  void finish(bool ok) {
    // 发布完成信号
    std_msgs::Bool done; done.data = ok;
    done_pub_.publish(done);
    // 停止本次采集
    collecting_ = false;
    timeout_timer_.stop();

    // 可选：在此将 collected_ 写入文件或发布到其它话题
    if (ok) {
      ROS_INFO("Collection finished successfully with %d rounds", rounds_completed_);
    }
  }

  ros::NodeHandle nh_;
  ros::Subscriber trigger_sub_;
  ros::Subscriber magnetic_sub_;
  ros::Publisher done_pub_;
  ros::Timer timeout_timer_;

  std::string magnetic_field_topic_;
  int expected_sensor_count_ = 25;
  int rounds_needed_ = 2;
  double collect_timeout_sec_ = 5.0;

  bool collecting_ = false;
  int rounds_completed_ = 0;
  std::unordered_set<int> current_round_seen_;
  std::vector<std::vector<magnetic_pose_estimation::MagneticField>> collected_;
  std::mutex mutex_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "mag_data_collector_node");
  ros::NodeHandle nh("~"); // 私有命名空间以便参数隔离
  MagDataCollectorNode node(nh);
  ros::spin();
  return 0;
}
