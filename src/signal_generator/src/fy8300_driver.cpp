#include "signal_generator/fy8300_driver.h"
#include <ros/ros.h>
#include <iostream>
#include <iomanip>
#include <sstream>

namespace fy8300_ros {

FY8300Driver::FY8300Driver() : connected_(false) {}

FY8300Driver::~FY8300Driver() {
    disconnect();
}

bool FY8300Driver::connect(const std::string& port, uint32_t baudrate) {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
        if (serial_.isOpen()) {
            serial_.close();
        }
        serial_.setPort(port);
        serial_.setBaudrate(baudrate);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(500);
        serial_.setTimeout(timeout);
        serial_.open();
        
        if (serial_.isOpen()) {
            serial_.flushInput();
            serial_.flushOutput();
            return true;
        }
    } catch (serial::IOException& e) {
        return false;
    }
    return false;
}

bool FY8300Driver::probe() {
    // Try multiple times to probe the device in case of startup garbage
    for (int retry = 0; retry < 3; ++retry) {
        std::string response;
        try {
            serial_.flushInput();
            serial_.write("RMN\n");
            response = serial_.readline(64, "\n");
            
            if (!response.empty()) {
                // Strip whitespace/newlines/CR
                response.erase(response.find_last_not_of(" \n\r\t") + 1);
                response.erase(0, response.find_first_not_of(" \n\r\t"));
                
                // Convert to int to handle potential leading zeros like "00000001"
                try {
                    int val = std::stoi(response);
                    if (val == 0 || val == 1 || val == 255) {
                        connected_ = true;
                        ros::Duration(0.5).sleep();
                        return true;
                    }
                } catch (...) {
                    // Not a valid number, continue retrying
                }
            }
        } catch (serial::IOException& e) {
            // Keep trying if we have retries left
        }
        ros::Duration(0.5).sleep();
    }
    return false;
}

void FY8300Driver::disconnect() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (serial_.isOpen()) {
        serial_.close();
    }
    connected_ = false;
}

bool FY8300Driver::isConnected() const {
    return connected_;
}

bool FY8300Driver::setWaveform(int channel, int waveform) {
    std::string prefix;
    if (channel == 1) prefix = "WMW";
    else if (channel == 2) prefix = "WFW";
    else if (channel == 3) prefix = "TFW";
    else return false;

    std::stringstream ss;
    // 手册规定波形为2位数字，例如 WMW00, WMW13
    ss << prefix << std::setfill('0') << std::setw(2) << waveform;
    // std::cout << "Setting waveform command: " << ss.str() << std::endl;
    return sendCommand(ss.str());
}

bool FY8300Driver::setFrequency(int channel, double frequency) {
    std::string prefix;
    if (channel == 1) prefix = "WMF";
    else if (channel == 2) prefix = "WFF";
    else if (channel == 3) prefix = "TFF";
    else return false;

    // 单位为 uHz，1 Hz = 1,000,000 uHz。手册要求固定以14位数字发送。
    long long uhz = static_cast<long long>(frequency * 1e6);
    std::stringstream ss;
    ss << prefix << std::setfill('0') << std::setw(14) << uhz;
    return sendCommand(ss.str());
}

bool FY8300Driver::setAmplitude(int channel, float amplitude) {
    std::string prefix;
    if (channel == 1) prefix = "WMA";
    else if (channel == 2) prefix = "WFA";
    else if (channel == 3) prefix = "TFA";
    else return false;

    std::stringstream ss;
    ss << std::fixed << std::setprecision(3) << amplitude;
    return sendCommand(prefix + ss.str());
}

bool FY8300Driver::setOffset(int channel, float offset) {
    std::string prefix;
    if (channel == 1) prefix = "WMO";
    else if (channel == 2) prefix = "WFO";
    else if (channel == 3) prefix = "TFO";
    else return false;

    std::stringstream ss;
    ss << std::fixed << std::setprecision(3) << offset;
    return sendCommand(prefix + ss.str());
}

bool FY8300Driver::setPhase(int channel, float phase) {
    std::string prefix;
    if (channel == 1) prefix = "WMP";
    else if (channel == 2) prefix = "WFP";
    else if (channel == 3) prefix = "TFP";
    else return false;

    std::stringstream ss;
    ss << std::fixed << std::setprecision(3) << phase;
    return sendCommand(prefix + ss.str());
}

bool FY8300Driver::setDutyCycle(int channel, float duty) {
    std::string prefix;
    if (channel == 1) prefix = "WMD";
    else if (channel == 2) prefix = "WFD";
    else if (channel == 3) prefix = "TFD";
    else return false;

    std::stringstream ss;
    // Format is xx.x (percent)
    ss << prefix << std::fixed << std::setprecision(1) << duty;
    return sendCommand(ss.str());
}

bool FY8300Driver::setOutputEnable(int channel, bool enable) {
    std::string prefix;
    if (channel == 1) prefix = "WMN";
    else if (channel == 2) prefix = "WFN";
    else if (channel == 3) prefix = "TFN";
    else return false;

    return sendCommand(prefix + (enable ? "1" : "0"));
}

bool FY8300Driver::setOutputSync(bool enable) {
    return sendCommand(enable ? "USA7" : "USD7");
}

bool FY8300Driver::sendCommand(const std::string& cmd) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!connected_) return false;

    try {
        serial_.write(cmd + "\n");
        // Wait for 0x0a response
        std::string response = serial_.readline(64, "\n");
        // delay
        ros::Duration(0.5).sleep();
        return !response.empty();
    } catch (serial::IOException& e) {
        connected_ = false;
        return false;
    }
}

bool FY8300Driver::sendCommandWithResponse(const std::string& cmd, std::string& response) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!connected_) return false;

    try {
        serial_.write(cmd + "\n");
        response = serial_.readline(64, "\n");
        if (response.empty()) return false;
        // Response usually ends with \n, strip if needed
        if (!response.empty() && response.back() == '\n') {
            response.pop_back();
        }
        return true;
    } catch (serial::IOException& e) {
        connected_ = false;
        return false;
    }
}

bool FY8300Driver::readWaveform(int channel, uint8_t& waveform) {
    std::string prefix;
    if (channel == 1) prefix = "RMW";
    else if (channel == 2) prefix = "RFW";
    else if (channel == 3) prefix = "RTW";
    else return false;

    std::string response;
    if (!sendCommandWithResponse(prefix, response)) return false;

    try {
        waveform = static_cast<uint8_t>(std::stoi(response));
        return true;
    } catch (...) {
        return false;
    }
}

bool FY8300Driver::readFrequency(int channel, double& frequency) {
    std::string prefix;
    if (channel == 1) prefix = "RMF";
    else if (channel == 2) prefix = "RFF";
    else if (channel == 3) prefix = "RTF";
    else return false;

    std::string response;
    if (!sendCommandWithResponse(prefix, response)) return false;

    try {
        frequency = std::stod(response);
        return true;
    } catch (...) {
        return false;
    }
}

bool FY8300Driver::readAmplitude(int channel, float& amplitude) {
    std::string prefix;
    if (channel == 1) prefix = "RMA";
    else if (channel == 2) prefix = "RFA";
    else if (channel == 3) prefix = "RTA";
    else return false;

    std::string response;
    if (!sendCommandWithResponse(prefix, response)) return false;

    try {
        float millivolt = std::stof(response);
        amplitude = millivolt / 1000.0f;
        return true;
    } catch (...) {
        return false;
    }
}

bool FY8300Driver::readOffset(int channel, float& offset) {
    std::string prefix;
    if (channel == 1) prefix = "RMO";
    else if (channel == 2) prefix = "RFO";
    else if (channel == 3) prefix = "RTO";
    else return false;

    std::string response;
    if (!sendCommandWithResponse(prefix, response)) return false;

    try {
        int val = std::stoi(response);
        if (val == 10000) {
            offset = 0.0f;
        } else if (val < 10000) {
            offset = -(10000 - val) / 1000.0f;
        } else {
            offset = (val - 10000) / 1000.0f;
        }
        return true;
    } catch (...) {
        return false;
    }
}

bool FY8300Driver::readPhase(int channel, float& phase) {
    std::string prefix;
    if (channel == 1) prefix = "RMP";
    else if (channel == 2) prefix = "RFP";
    else if (channel == 3) prefix = "RTP";
    else return false;

    std::string response;
    if (!sendCommandWithResponse(prefix, response)) return false;

    try {
        phase = std::stof(response) / 10.0f;
        return true;
    } catch (...) {
        return false;
    }
}

bool FY8300Driver::readDutyCycle(int channel, float& duty) {
    std::string prefix;
    if (channel == 1) prefix = "RMD";
    else if (channel == 2) prefix = "RFD";
    else if (channel == 3) prefix = "RTD";
    else return false;

    std::string response;
    if (!sendCommandWithResponse(prefix, response)) return false;

    try {
        duty = std::stof(response) / 10.0f;
        return true;
    } catch (...) {
        return false;
    }
}

bool FY8300Driver::readOutputEnabled(int channel, bool& enabled) {
    std::string prefix;
    if (channel == 1) prefix = "RMN";
    else if (channel == 2) prefix = "RFN";
    else if (channel == 3) prefix = "RTN";
    else return false;

    std::string response;
    if (!sendCommandWithResponse(prefix, response)) return false;

    try {
        int val = std::stoi(response);
        enabled = (val > 0);
        return true;
    } catch (...) {
        return false;
    }
}

bool FY8300Driver::readChannelStatus(int channel, uint8_t& waveform, double& frequency,
                                     float& amplitude, float& offset, float& phase,
                                     float& duty, bool& output_enabled) {
    return readWaveform(channel, waveform) &&
           readFrequency(channel, frequency) &&
           readAmplitude(channel, amplitude) &&
           readOffset(channel, offset) &&
           readPhase(channel, phase) &&
           readDutyCycle(channel, duty) &&
           readOutputEnabled(channel, output_enabled);
}

} // namespace fy8300_ros
