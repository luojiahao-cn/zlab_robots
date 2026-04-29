#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <vector>
#include <algorithm>
#include <cmath>
#include <deque>

// Helper to compute median of a vector
double getMedian(std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    size_t n = v.size();
    if (n % 2 == 0) {
        return (v[n / 2 - 1] + v[n / 2]) / 2.0;
    } else {
        return v[n / 2];
    }
}

class FilterNode {
private:
    ros::NodeHandle nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    std::string source_frame_;
    std::string target_frame_;
    std::string reference_frame_;

    // Sliding window buffer
    std::deque<geometry_msgs::TransformStamped> window_;
    int window_size_;

    // Thresholds to identify bad data (jitter)
    double max_translation_jump_;
    double max_rotation_jump_; // In radians
    geometry_msgs::TransformStamped last_valid_transform_;
    bool has_first_data_ = false;

    // Timeout to reinitialize filter when vision data is lost and recovered
    double reset_timeout_ms_;
    ros::Time last_update_time_;
    ros::Time last_tf_stamp_;
    bool has_last_tf_stamp_ = false;

    geometry_msgs::TransformStamped last_raw_transform_;
    bool has_last_raw_transform_ = false;

public:
    FilterNode(const std::string& source_frame, const std::string& target_frame)
        : nh_("~"),
          tf_listener_(tf_buffer_),
          source_frame_(source_frame),
          target_frame_(target_frame) 
    {
        // Load parameters with defaults
        nh_.param("window_size", window_size_, 5);
        nh_.param("max_translation_jump", max_translation_jump_, 0.5);
        nh_.param("max_rotation_jump", max_rotation_jump_, 0.5); // Default 0.5 rad (~28.6 degrees)
        nh_.param("reset_timeout_ms", reset_timeout_ms_, 500.0); // Default 500ms
        nh_.param<std::string>("reference_frame", reference_frame_, "lab_table");
    }

    void run() {
        ros::Rate rate(50.0); // 50 Hz loop
        while (ros::ok()) {
            // Check for timeout - if no update within reset_timeout_ms_, reinitialize filter
            if (has_first_data_) {
                double elapsed_ms = (ros::Time::now() - last_update_time_).toSec() * 1000.0;
                if (elapsed_ms > reset_timeout_ms_) {
                    // ROS_WARN("FilterNode: No update for %.1f ms, reinitializing filter for frame %s.", elapsed_ms, source_frame_.c_str());
                    resetFilter();
                }
            }

            try {
                // Query source frame against a fixed dynamic reference frame.
                geometry_msgs::TransformStamped transform =
                    tf_buffer_.lookupTransform(reference_frame_, source_frame_, ros::Time(0));

                if (isNewTransform(transform)) {
                    last_tf_stamp_ = transform.header.stamp;
                    has_last_tf_stamp_ = true;
                    last_raw_transform_ = transform;
                    has_last_raw_transform_ = true;
                    processTransform(transform);
                }
            } catch (tf2::TransformException &ex) {
                // If tf is not available yet, just wait
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    void resetFilter() {
        window_.clear();
        has_first_data_ = false;
        has_last_tf_stamp_ = false;
        has_last_raw_transform_ = false;
        // last_valid_transform_ intentionally kept for reference
    }

    void processTransform(const geometry_msgs::TransformStamped& current_transform) {
        // 1. Check for severe jitter (bad data)
        if (has_first_data_) {
            // Check translation jitter
            double dx = current_transform.transform.translation.x - last_valid_transform_.transform.translation.x;
            double dy = current_transform.transform.translation.y - last_valid_transform_.transform.translation.y;
            double dz = current_transform.transform.translation.z - last_valid_transform_.transform.translation.z;
            
            double trans_dist = std::sqrt(dx*dx + dy*dy + dz*dz);

            // Check rotation jitter
            tf2::Quaternion q_curr, q_last;
            tf2::fromMsg(current_transform.transform.rotation, q_curr);
            tf2::fromMsg(last_valid_transform_.transform.rotation, q_last);

            double angle_diff = q_curr.angleShortestPath(q_last);

            if (trans_dist > max_translation_jump_ || angle_diff > max_rotation_jump_) {
                // ROS_WARN_THROTTLE(1.0, "FilterNode: Severe jitter detected (trans: %f, rot: %f), dropping transform.", trans_dist, angle_diff);
                return;
            }
        } else {
            has_first_data_ = true;
        }

        last_valid_transform_ = current_transform;
        last_update_time_ = ros::Time::now();

        // 2. Add to sliding window
        window_.push_back(current_transform);
        if (window_.size() > window_size_) {
            window_.pop_front();
        }

        // 3. Median filter
        if (!window_.empty()) {
            publishFiltered(current_transform.header.frame_id);
        }
    }

    void publishFiltered(const std::string& parent_frame) {
        std::vector<double> tx, ty, tz;
        std::vector<double> qx, qy, qz, qw;

        for (const auto& t : window_) {
            tx.push_back(t.transform.translation.x);
            ty.push_back(t.transform.translation.y);
            tz.push_back(t.transform.translation.z);
            qx.push_back(t.transform.rotation.x);
            qy.push_back(t.transform.rotation.y);
            qz.push_back(t.transform.rotation.z);
            qw.push_back(t.transform.rotation.w);
        }

        geometry_msgs::TransformStamped filtered_transform;
        filtered_transform.header.stamp = ros::Time::now();
        filtered_transform.header.frame_id = parent_frame;
        filtered_transform.child_frame_id = target_frame_;

        filtered_transform.transform.translation.x = getMedian(tx);
        filtered_transform.transform.translation.y = getMedian(ty);
        filtered_transform.transform.translation.z = getMedian(tz);

        // For quaternions, independent median isn't strictly mathematically correct for rotations, 
        // but it is a simple approximation often used if variance is small.
        // We normalize it afterwards.
        double mx = getMedian(qx);
        double my = getMedian(qy);
        double mz = getMedian(qz);
        double mw = getMedian(qw);
        
        double norm = std::sqrt(mx*mx + my*my + mz*mz + mw*mw);
        if (norm > 1e-6) {
            filtered_transform.transform.rotation.x = mx / norm;
            filtered_transform.transform.rotation.y = my / norm;
            filtered_transform.transform.rotation.z = mz / norm;
            filtered_transform.transform.rotation.w = mw / norm;
        } else {
            filtered_transform.transform.rotation.x = 0;
            filtered_transform.transform.rotation.y = 0;
            filtered_transform.transform.rotation.z = 0;
            filtered_transform.transform.rotation.w = 1.0;
        }

        tf_broadcaster_.sendTransform(filtered_transform);
    }

    bool isNewTransform(const geometry_msgs::TransformStamped& t) {
        // 1) 时间戳判新（仅当上游给了有效非零时间）
        // if (has_last_tf_stamp_ && !t.header.stamp.isZero() && t.header.stamp != last_tf_stamp_) {
        //     return true;
        // }

        // 2) 时间戳不可用时，按位姿变化判新
        if (!has_last_raw_transform_) return true;

        double dx = t.transform.translation.x - last_raw_transform_.transform.translation.x;
        double dy = t.transform.translation.y - last_raw_transform_.transform.translation.y;
        double dz = t.transform.translation.z - last_raw_transform_.transform.translation.z;
        double trans = std::sqrt(dx*dx + dy*dy + dz*dz);

        tf2::Quaternion q1, q2;
        tf2::fromMsg(t.transform.rotation, q1);
        tf2::fromMsg(last_raw_transform_.transform.rotation, q2);
        double rot = q1.angleShortestPath(q2);

        const double eps_trans = 0; // 按噪声调
        const double eps_rot   = 0; // rad
        return trans > eps_trans || rot > eps_rot;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "filter_node");

    if (argc < 3) {
        ROS_ERROR("Usage: filter_node <source_frame> <target_frame>");
        return -1;
    }

    std::string source_frame = argv[1];
    std::string target_frame = argv[2];

    FilterNode node(source_frame, target_frame);
    node.run();

    return 0;
}
