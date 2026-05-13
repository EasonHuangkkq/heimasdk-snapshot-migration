#pragma once

#include <chrono>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace RmdCanSdk {

constexpr float Pi = 3.14159265358979323846f;

struct MotorParameters {
    float polarity = 1.0f;
    float countBias = 0.0f;
    float encoderResolution = 1.0f;
    float gearRatioTor = 1.0f;
    float gearRatioPosVel = 1.0f;
    float ratedCurrent = 1.0f;
    float torqueConstant = 1.0f;
    float ratedTorque = 1.0f;
    float maximumTorque = 1.0f;
    float minimumPosition = -1.0f;
    float maximumPosition = 1.0f;
};

struct MotorTarget {
    float pos = 0.0f;
    float vel = 0.0f;
    float tor = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
    int mode = 8;
    int enabled = 0;
    int maxCurrent = 1000;
};

struct MotorActual {
    float pos = 0.0f;
    float vel = 0.0f;
    float tor = 0.0f;
    std::int32_t encoderCount = 0;
    short temp = 0;
    short driveTemp = 0;
    unsigned short voltage = 0;
    unsigned short statusWord = 65535;
    unsigned short errorCode = 0;
};

enum class MotorBus {
    Can,
    Ethercat,
};

struct MotorConfig {
    int alias = 0;
    int master = 0;
    int motorId = 0;
    int slaveId = 0;
    int ethercatSlave = -1;
    int domain = 0;
    MotorBus bus = MotorBus::Can;
    std::string type;
    MotorParameters parameters;
};

struct MasterConfig {
    int order = 0;
    std::string device;
    int baudrate = 1000000;
    bool canfd = false;
    int dbaudrate = 5000000;
    int division = 1;
};

struct ImuConfig {
    bool enabled = false;
    std::string device;
    int baudrate = 0;
    std::string type;
    std::string normalizedType;
};

struct Config {
    long periodNs = 1000000L;
    bool ethercatDc = false;
    int totalMotorCount = 0;
    std::vector<MasterConfig> masters;
    std::vector<int> ethercatDomainDivisions;
    std::vector<MotorConfig> motors;
    ImuConfig imu;
};

class MaskTracker {
public:
    void setExpectedFromSlaveIds(std::vector<int> const& slaveIds);
    bool markReceived(int slaveId);
    bool expects(int slaveId) const;
    bool isReceived(int slaveId) const;
    void resetCurrent();
    std::uint64_t expectedMask() const { return expectedMask_; }
    std::uint64_t currentMask() const { return currentMask_; }

private:
    static std::uint64_t bitForSlaveId(int slaveId);

    std::uint64_t expectedMask_ = 0;
    std::uint64_t currentMask_ = 0;
};

class FeedbackFrameTracker {
public:
    using Clock = std::chrono::steady_clock;

    void setExpectedFromSlaveIds(std::vector<int> const& slaveIds);
    bool markReceived(int slaveId, Clock::time_point now);
    bool shouldPublishTimeout(Clock::time_point now, Clock::duration timeout) const;
    bool isReceived(int slaveId) const;
    void reset();
    std::uint64_t expectedMask() const { return mask_.expectedMask(); }
    std::uint64_t currentMask() const { return mask_.currentMask(); }

private:
    MaskTracker mask_;
    bool active_ = false;
    Clock::time_point started_{};
};

float clamp(float value, float min, float max);

} // namespace RmdCanSdk
