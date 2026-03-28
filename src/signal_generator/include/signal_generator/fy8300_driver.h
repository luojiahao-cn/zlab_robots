#ifndef FY8300_DRIVER_H
#define FY8300_DRIVER_H

#include <serial/serial.h>
#include <string>
#include <vector>
#include <mutex>

namespace signal_generator {

class FY8300Driver {
public:
    FY8300Driver();
    ~FY8300Driver();

    bool connect(const std::string& port, uint32_t baudrate = 115200);
    bool probe();
    void disconnect();
    bool isConnected() const;

    // CH1: 1, CH2: 2, CH3: 3
    bool setWaveform(int channel, int waveform);
    bool setFrequency(int channel, double frequency);
    bool setAmplitude(int channel, float amplitude);
    bool setOffset(int channel, float offset);
    bool setPhase(int channel, float phase);
    bool setDutyCycle(int channel, float duty);
    bool setOutputEnable(int channel, bool enable);
    bool setOutputSync(bool enable);

    // Read methods - return true on success
    bool readWaveform(int channel, uint8_t& waveform);
    bool readFrequency(int channel, double& frequency);
    bool readAmplitude(int channel, float& amplitude);
    bool readOffset(int channel, float& offset);
    bool readPhase(int channel, float& phase);
    bool readDutyCycle(int channel, float& duty);
    bool readOutputEnabled(int channel, bool& enabled);
    bool readChannelStatus(int channel, uint8_t& waveform, double& frequency,
                           float& amplitude, float& offset, float& phase,
                           float& duty, bool& output_enabled);

private:
    bool sendCommand(const std::string& cmd);
    bool sendCommandWithResponse(const std::string& cmd, std::string& response);

    serial::Serial serial_;
    std::mutex mutex_;
    bool connected_;
};

} // namespace signal_generator

#endif // FY8300_DRIVER_H
