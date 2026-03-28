#include "signal_generator/fy8300_node.h"

namespace signal_generator {

FY8300Node::FY8300Node(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh), sync_output_(false), status_poll_rate_(1.0) {
    pnh_.param<int>("baudrate", baudrate_, 115200);
    pnh_.getParam("search_ports", search_ports_);
    pnh_.param<double>("status_poll_rate", status_poll_rate_, 1.0);

    // If search_ports is empty, add a default
    if (search_ports_.empty()) {
        search_ports_.push_back("/dev/ttyUSB0");
    }

    // sub_channel_ = nh_.subscribe("channel_control", 10, &FY8300Node::channelCallback, this);

    // Setup independent parameter topics for each channel
    for (int i = 1; i <= 3; ++i) {
        std::string ns = "fy8300/ch" + std::to_string(i) + "/";

        // Waveform: std_msgs/UInt8
        sub_independents_.push_back(nh_.subscribe<std_msgs::UInt8>(ns + "waveform", 1,
            boost::bind(&FY8300Node::waveformCb, this, _1, i)));

        // Frequency: std_msgs/Float64
        sub_independents_.push_back(nh_.subscribe<std_msgs::Float64>(ns + "frequency", 1,
            boost::bind(&FY8300Node::freqCb, this, _1, i)));

        // Amplitude: std_msgs/Float32
        sub_independents_.push_back(nh_.subscribe<std_msgs::Float32>(ns + "amplitude", 1,
            boost::bind(&FY8300Node::ampCb, this, _1, i)));

        // Offset: std_msgs/Float32
        sub_independents_.push_back(nh_.subscribe<std_msgs::Float32>(ns + "offset", 1,
            boost::bind(&FY8300Node::offsetCb, this, _1, i)));

        // Phase: std_msgs/Float32
        sub_independents_.push_back(nh_.subscribe<std_msgs::Float32>(ns + "phase", 1,
            boost::bind(&FY8300Node::phaseCb, this, _1, i)));

        // Duty: std_msgs/Float32
        sub_independents_.push_back(nh_.subscribe<std_msgs::Float32>(ns + "duty", 1,
            boost::bind(&FY8300Node::dutyCb, this, _1, i)));

        // Output Status: std_msgs/Bool
        sub_independents_.push_back(nh_.subscribe<std_msgs::Bool>(ns + "output_en", 1,
            boost::bind(&FY8300Node::outputEnCb, this, _1, i)));

        // Status publisher for this channel
        pub_status_.push_back(nh_.advertise<signal_generator::ChannelStatus>(ns + "status", 10));
    }

    // Global Sync Topic: std_msgs/Bool
    sub_independents_.push_back(nh_.subscribe("fy8300/sync_output", 1, &FY8300Node::outputSyncCb, this));
}

FY8300Node::~FY8300Node() {
    if (driver_.isConnected()) {
        ROS_INFO("System shutting down, disabling all outputs...");
        // 无论是否开启同步，分别关闭三个通道以确保安全
        driver_.setOutputEnable(1, false);
        driver_.setOutputEnable(2, false);
        driver_.setOutputEnable(3, false);
    }
    driver_.disconnect();
}

bool FY8300Node::init() {
    if (autoConnect()) {
        ROS_INFO("FY8300 connected on port %s", port_.c_str());
        applyInitialConfig();

        // Start status polling timer
        if (status_poll_rate_ > 0) {
            ros::Duration period(1.0 / status_poll_rate_);
            status_timer_ = nh_.createTimer(period, &FY8300Node::statusTimerCb, this);
            ROS_INFO("Status polling started at %.1f Hz", status_poll_rate_);
        }

        return true;
    }
    ROS_ERROR("Failed to connect to FY8300 on any specified ports.");
    return false;
}

void FY8300Node::applyInitialConfig() {
    XmlRpc::XmlRpcValue config_list;
    if (!pnh_.getParam("initial_configs", config_list)) {
        ROS_WARN("No 'initial_configs' found in parameter server. Skipping initial setup.");
        return;
    }

    if (config_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("'initial_configs' should be an array.");
        return;
    }

    // 第一步：发送除 output_en 以外的所有参数
    for (int i = 0; i < config_list.size(); ++i) {
        XmlRpc::XmlRpcValue& conf = config_list[i];
        
        signal_generator::ChannelControl msg;
        
        auto getDouble = [&](const std::string& key, auto& val) {
            if (conf.hasMember(key)) {
                if (conf[key].getType() == XmlRpc::XmlRpcValue::TypeDouble) val = static_cast<double>(conf[key]);
                else if (conf[key].getType() == XmlRpc::XmlRpcValue::TypeInt) val = (double)static_cast<int>(conf[key]);
                return true;
            }
            return false;
        };

        if (conf.hasMember("channel_index")) msg.channel_index = static_cast<int>(conf["channel_index"]);
        if (conf.hasMember("waveform")) msg.waveform = static_cast<int>(conf["waveform"]);

        getDouble("frequency", msg.frequency);
        getDouble("amplitude", msg.amplitude);
        getDouble("offset", msg.offset);
        getDouble("phase", msg.phase);
        
        if (conf.hasMember("duty_cycle")) {
            if (getDouble("duty_cycle", msg.duty_cycle)) {
                msg.update_mask |= 128;
            }
        }
        
        if (conf.hasMember("output_sync")) {
            msg.output_sync = static_cast<bool>(conf["output_sync"]);
            msg.update_mask |= 64; 
            // 只有通道1设置同步才生效
            if (msg.channel_index == 1) sync_output_ = msg.output_sync;
        }
        
        // 更新掩码：1(波形)|2(频率)|4(幅度)|8(偏置)|16(相位) = 31
        msg.update_mask |= 31; 

        signal_generator::ChannelControl::Ptr msg_ptr(new signal_generator::ChannelControl(msg));
        channelCallback(msg_ptr);
        
        ros::Duration(0.1).sleep(); 
    }
    ROS_INFO("Initial configurations applied from parameter file.");

    // 第二步：发送 output_en 命令
    for (int i = 0; i < config_list.size(); ++i) {
        XmlRpc::XmlRpcValue& conf = config_list[i];
        int ch = static_cast<int>(conf["channel_index"]);

        // 如果开启了同步，则只发送通道1的 output_en 命令
        if (sync_output_ && ch != 1) continue;

        if (conf.hasMember("output_en")) {
            signal_generator::ChannelControl msg;
            msg.channel_index = ch;
            msg.output_en = static_cast<bool>(conf["output_en"]);
            msg.update_mask = 32; // 仅更新 output_en

            signal_generator::ChannelControl::Ptr msg_ptr(new signal_generator::ChannelControl(msg));
            channelCallback(msg_ptr);
            ros::Duration(0.1).sleep();
        }
    }

    ROS_INFO("Output state applied from parameter file.");
}

bool FY8300Node::autoConnect() {
    for (const auto& p : search_ports_) {
        ROS_INFO("Attempting to connect to %s...", p.c_str());
        if (driver_.connect(p, baudrate_)) {
            if (driver_.probe()) {
                port_ = p;
                return true;
            }
            driver_.disconnect();
        }
    }
    return false;
}

void FY8300Node::channelCallback(const signal_generator::ChannelControl::ConstPtr& msg) {
    if (!driver_.isConnected()) {
        ROS_WARN("Attempted to control channel while driver is not connected.");
        return;
    }

    if (msg->channel_index < 1 || msg->channel_index > 3) {
        ROS_ERROR("Invalid channel index: %d. Must be 1, 2, or 3.", msg->channel_index);
        return;
    }

    bool success = true;
    uint8_t waveform = msg->waveform;
    // 通道2和3的波形编号需要调整，跳过5号波形
    if (msg->channel_index != 1 && waveform >= 5) {
        if(waveform == 5) {
            ROS_WARN("Channel %d: Adjustable Pulse waveform may not be supported. Using waveform 0 (sin wave) instead.", msg->channel_index);
            waveform = 1;
        }
        waveform -= 1;
    }
    if (msg->update_mask & 1) success &= driver_.setWaveform(msg->channel_index, waveform);
    if (!success) ROS_ERROR("Failed to set waveform for Channel %d", msg->channel_index);
    if (msg->update_mask & 2) success &= driver_.setFrequency(msg->channel_index, msg->frequency);
    if (!success) ROS_ERROR("Failed to set frequency for Channel %d", msg->channel_index);
    if (msg->update_mask & 4) success &= driver_.setAmplitude(msg->channel_index, msg->amplitude);
    if (!success) ROS_ERROR("Failed to set amplitude for Channel %d", msg->channel_index);
    if (msg->update_mask & 8) success &= driver_.setOffset(msg->channel_index, msg->offset);
    if (!success) ROS_ERROR("Failed to set offset for Channel %d", msg->channel_index);
    if (msg->update_mask & 16) success &= driver_.setPhase(msg->channel_index, msg->phase);
    if (!success) ROS_ERROR("Failed to set phase for Channel %d", msg->channel_index);
    if (msg->update_mask & 128) success &= driver_.setDutyCycle(msg->channel_index, msg->duty_cycle);
    if (!success) ROS_ERROR("Failed to set duty cycle for Channel %d", msg->channel_index);
    if (msg->update_mask & 32) {
        if(msg->channel_index !=1 && sync_output_) {
            ROS_WARN("Channel %d output enable change ignored due to sync mode.", msg->channel_index);
        } 
        else
            success &= driver_.setOutputEnable(msg->channel_index, msg->output_en);
    }
    if (!success) ROS_ERROR("Failed to set output enable for Channel %d", msg->channel_index);
    if (msg->update_mask & 64) {
        if (msg->channel_index == 1) {
            success &= driver_.setOutputSync(msg->output_sync);
            if (success) sync_output_ = msg->output_sync;
        } else {
            ROS_WARN("Output sync can only be set via Channel 1 message.");
        }
    }

    if (success) {
        ROS_DEBUG("Updated Channel %d with mask %d", msg->channel_index, msg->update_mask);
    } else {
        ROS_ERROR("Failed to update Channel %d", msg->channel_index);
    }
}

// Independent topic callbacks
void FY8300Node::waveformCb(const std_msgs::UInt8::ConstPtr& msg, int ch) {
    uint8_t waveform = msg->data;
    if (ch != 1 && waveform >= 5) {
        if(waveform == 5) waveform = 0;
        else waveform -= 1;
    }
    if (!driver_.setWaveform(ch, waveform)) ROS_ERROR("Failed to set waveform for Channel %d", ch);
}

void FY8300Node::freqCb(const std_msgs::Float64::ConstPtr& msg, int ch) {
    if (!driver_.setFrequency(ch, msg->data)) ROS_ERROR("Failed to set frequency for Channel %d", ch);
}

void FY8300Node::ampCb(const std_msgs::Float32::ConstPtr& msg, int ch) {
    if (!driver_.setAmplitude(ch, msg->data)) ROS_ERROR("Failed to set amplitude for Channel %d", ch);
}

void FY8300Node::offsetCb(const std_msgs::Float32::ConstPtr& msg, int ch) {
    if (!driver_.setOffset(ch, msg->data)) ROS_ERROR("Failed to set offset for Channel %d", ch);
}

void FY8300Node::phaseCb(const std_msgs::Float32::ConstPtr& msg, int ch) {
    if (!driver_.setPhase(ch, msg->data)) ROS_ERROR("Failed to set phase for Channel %d", ch);
}

void FY8300Node::dutyCb(const std_msgs::Float32::ConstPtr& msg, int ch) {
    if (!driver_.setDutyCycle(ch, msg->data)) ROS_ERROR("Failed to set duty for Channel %d", ch);
}

void FY8300Node::outputEnCb(const std_msgs::Bool::ConstPtr& msg, int ch) {
    if (ch != 1 && sync_output_) {
        ROS_WARN("Channel %d output toggle ignored due to sync mode.", ch);
        return;
    }
    if (!driver_.setOutputEnable(ch, msg->data)) ROS_ERROR("Failed to set output_en for Channel %d", ch);
}

void FY8300Node::outputSyncCb(const std_msgs::Bool::ConstPtr& msg) {
    if (driver_.setOutputSync(msg->data)) {
        sync_output_ = msg->data;
    } else {
        ROS_ERROR("Failed to set output synchronization.");
    }
}

void FY8300Node::statusTimerCb(const ros::TimerEvent& event) {
    if (!driver_.isConnected()) return;

    for (int ch = 1; ch <= 3; ++ch) {
        publishChannelStatus(ch);
    }
}

void FY8300Node::publishChannelStatus(int channel) {
    signal_generator::ChannelStatus status;
    status.channel_index = channel;

    uint8_t waveform;
    double frequency;
    float amplitude;
    float offset;
    float phase;
    float duty;
    bool output_enabled;

    bool success = driver_.readChannelStatus(
        channel,
        waveform,
        frequency,
        amplitude,
        offset,
        phase,
        duty,
        output_enabled
    );

    status.waveform = waveform;
    status.frequency = frequency;
    status.amplitude = amplitude;
    status.offset = offset;
    status.phase = phase;
    status.duty_cycle = duty;
    status.output_enabled = output_enabled;
    status.valid = success;

    if (!success) {
        ROS_WARN_THROTTLE(5, "Failed to read status for channel %d", channel);
    }

    pub_status_[channel - 1].publish(status);
}

} // namespace signal_generator
