#ifndef FY8300_NODE_H
#define FY8300_NODE_H

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include "signal_generator/fy8300_driver.h"
#include "signal_generator/ChannelControl.h"
#include "signal_generator/ChannelStatus.h"

namespace signal_generator {

class FY8300Node {
public:
    FY8300Node(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~FY8300Node();

    bool init();

private:
    void applyInitialConfig();
    void channelCallback(const signal_generator::ChannelControl::ConstPtr& msg);
    
    // Independent topic callbacks
    void waveformCb(const std_msgs::UInt8::ConstPtr& msg, int ch);
    void freqCb(const std_msgs::Float64::ConstPtr& msg, int ch);
    void ampCb(const std_msgs::Float32::ConstPtr& msg, int ch);
    void offsetCb(const std_msgs::Float32::ConstPtr& msg, int ch);
    void phaseCb(const std_msgs::Float32::ConstPtr& msg, int ch);
    void dutyCb(const std_msgs::Float32::ConstPtr& msg, int ch);
    void outputEnCb(const std_msgs::Bool::ConstPtr& msg, int ch);
    void outputSyncCb(const std_msgs::Bool::ConstPtr& msg);

    // Status polling
    void statusTimerCb(const ros::TimerEvent& event);
    void publishChannelStatus(int channel);

    bool autoConnect();

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    ros::Subscriber sub_channel_;
    std::vector<ros::Subscriber> sub_independents_;

    std::vector<ros::Publisher> pub_status_;
    ros::Timer status_timer_;
    double status_poll_rate_;

    FY8300Driver driver_;
    
    std::vector<std::string> search_ports_;
    std::string port_;
    int baudrate_;
    bool sync_output_;
};

} // namespace signal_generator

#endif // FY8300_NODE_H
