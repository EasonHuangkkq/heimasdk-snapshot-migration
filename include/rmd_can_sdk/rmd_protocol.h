#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace RmdCanSdk {

struct MotorActual;

constexpr float RmdPMin = -12.566f;
constexpr float RmdPMax = 12.566f;
constexpr float RmdVMin = -45.0f;
constexpr float RmdVMax = 45.0f;
constexpr float RmdKpMin = 0.0f;
constexpr float RmdKpMax = 500.0f;
constexpr float RmdKdMin = 0.0f;
constexpr float RmdKdMax = 50.0f;
constexpr float RmdDefaultMaxTorqueNm = 24.0f;
constexpr float RmdTMin = -RmdDefaultMaxTorqueNm;
constexpr float RmdTMax = RmdDefaultMaxTorqueNm;

int standardTxId(int motorId);
int standardRxId(int motorId);
int mitTxId(int motorId);
int mitRxId(int motorId);

std::uint16_t floatToUint(float value, float min, float max, int bits);
float uintToFloat(std::uint16_t value, float min, float max, int bits);

std::array<unsigned char, 8> packMit(float p, float v, float kp, float kd, float torque,
                                     float maxTorqueNm = RmdDefaultMaxTorqueNm);

struct MitFeedback {
    bool valid = false;
    int motorId = 0;
    float positionRad = 0.0f;
    float velocityRadS = 0.0f;
    float feedbackTorqueNm = 0.0f;
};

MitFeedback parseMitReply(std::vector<unsigned char> const& data, int expectedMotorId = 0,
                          float maxTorqueNm = RmdDefaultMaxTorqueNm);
MitFeedback parseMitReply(std::array<unsigned char, 8> const& data,
                          int length,
                          int expectedMotorId,
                          float maxTorqueNm);

constexpr std::uint16_t Status1ErrorStall = 0x0002;
constexpr std::uint16_t Status1ErrorLowVoltage = 0x0004;
constexpr std::uint16_t Status1ErrorOverVoltage = 0x0008;
constexpr std::uint16_t Status1ErrorPhaseCurrentOverCurrent = 0x0010;
constexpr std::uint16_t Status1ErrorPowerOverLimit = 0x0040;
constexpr std::uint16_t Status1ErrorCalibrationParameterWrite = 0x0080;
constexpr std::uint16_t Status1ErrorOverspeed = 0x0100;
constexpr std::uint16_t Status1ErrorComponentOverTemperature = 0x0800;
constexpr std::uint16_t Status1ErrorMotorOverTemperature = 0x1000;
constexpr std::uint16_t Status1ErrorEncoderCalibration = 0x2000;
constexpr std::uint16_t Status1ErrorEncoderData = 0x4000;

struct Status1Feedback {
    bool valid = false;
    int temperatureC = 0;
    int mosTemperatureC = 0;
    unsigned char brakeReleaseCommandState = 0;
    bool brakeReleaseCommandActive = false;
    float voltageV = 0.0f;
    std::uint16_t errorState = 0;
};

Status1Feedback parseStatus1Reply(std::vector<unsigned char> const& data);

struct Status2Feedback {
    bool valid = false;
    int temperatureC = 0;
    float iqA = 0.0f;
    int speedDps = 0;
    int angleDeg = 0;
    float angleRad = 0.0f;
    float estimatedTorqueNm = 0.0f;
};

Status2Feedback parseStatus2Reply(std::vector<unsigned char> const& data, float torqueConstantNmPerAmp);
Status2Feedback parseStatus2Reply(std::array<unsigned char, 8> const& data,
                                  int length,
                                  float torqueConstantNmPerAmp);

struct MultiturnAngleFeedback {
    bool valid = false;
    std::int32_t rawAngleCentideg = 0;
    float angleDeg = 0.0f;
    float angleRad = 0.0f;
};

MultiturnAngleFeedback parseMultiturnAngleReply(std::vector<unsigned char> const& data);

void applyMitFeedback(MotorActual& actual, MitFeedback const& feedback);
void applyStatus2Feedback(MotorActual& actual, Status2Feedback const& feedback);

} // namespace RmdCanSdk
