#include "rmd_can_sdk/rmd_protocol.h"
#include "rmd_can_sdk/rmd_types.h"

#include <cmath>
#include <cstdint>

namespace RmdCanSdk {
namespace {

std::int16_t int16Le(unsigned char low, unsigned char high) {
    return static_cast<std::int16_t>(static_cast<std::uint16_t>(low) |
                                     (static_cast<std::uint16_t>(high) << 8));
}

std::uint16_t uint16Le(unsigned char low, unsigned char high) {
    return static_cast<std::uint16_t>(static_cast<std::uint16_t>(low) |
                                      (static_cast<std::uint16_t>(high) << 8));
}

std::int32_t int32Le(unsigned char b0, unsigned char b1, unsigned char b2, unsigned char b3) {
    std::uint32_t const raw = static_cast<std::uint32_t>(b0) |
                              (static_cast<std::uint32_t>(b1) << 8) |
                              (static_cast<std::uint32_t>(b2) << 16) |
                              (static_cast<std::uint32_t>(b3) << 24);
    return static_cast<std::int32_t>(raw);
}

float positiveTorqueLimit(float maxTorqueNm) {
    if (std::isfinite(maxTorqueNm) && maxTorqueNm > 0.0f) {
        return maxTorqueNm;
    }
    return RmdDefaultMaxTorqueNm;
}

MitFeedback parseMitReplyBytes(unsigned char const* data, int length, int expectedMotorId, float maxTorqueNm) {
    MitFeedback feedback;
    if (data == nullptr || length != 8) {
        return feedback;
    }
    feedback.motorId = data[0];
    if (expectedMotorId > 0 && feedback.motorId != expectedMotorId) {
        return feedback;
    }
    std::uint16_t p = static_cast<std::uint16_t>((data[1] << 8) | data[2]);
    std::uint16_t v = static_cast<std::uint16_t>((data[3] << 4) | (data[4] >> 4));
    std::uint16_t t = static_cast<std::uint16_t>(((data[4] & 0x0f) << 8) | data[5]);
    float const torqueLimit = positiveTorqueLimit(maxTorqueNm);
    feedback.positionRad = uintToFloat(p, RmdPMin, RmdPMax, 16);
    feedback.velocityRadS = uintToFloat(v, RmdVMin, RmdVMax, 12);
    feedback.feedbackTorqueNm = uintToFloat(t, -torqueLimit, torqueLimit, 12);
    feedback.valid = true;
    return feedback;
}

Status2Feedback parseStatus2ReplyBytes(unsigned char const* data, int length, float torqueConstantNmPerAmp) {
    Status2Feedback feedback;
    if (data == nullptr || length != 8 || data[0] != 0x9C) {
        return feedback;
    }
    feedback.temperatureC = static_cast<std::int8_t>(data[1]);
    feedback.iqA = static_cast<float>(int16Le(data[2], data[3])) * 0.01f;
    feedback.speedDps = int16Le(data[4], data[5]);
    feedback.angleDeg = int16Le(data[6], data[7]);
    feedback.angleRad = static_cast<float>(feedback.angleDeg) * Pi / 180.0f;
    feedback.estimatedTorqueNm = feedback.iqA * torqueConstantNmPerAmp;
    feedback.valid = true;
    return feedback;
}

} // namespace

int standardTxId(int motorId) {
    return 0x140 + motorId;
}

int standardRxId(int motorId) {
    return 0x240 + motorId;
}

int mitTxId(int motorId) {
    return 0x400 + motorId;
}

int mitRxId(int motorId) {
    return 0x500 + motorId;
}

std::uint16_t floatToUint(float value, float min, float max, int bits) {
    value = clamp(value, min, max);
    float const span = max - min;
    int const maxInt = (1 << bits) - 1;
    return static_cast<std::uint16_t>(std::lround((value - min) * maxInt / span));
}

float uintToFloat(std::uint16_t value, float min, float max, int bits) {
    int const maxInt = (1 << bits) - 1;
    if (value > maxInt) {
        value = static_cast<std::uint16_t>(maxInt);
    }
    return static_cast<float>(value) * (max - min) / static_cast<float>(maxInt) + min;
}

std::array<unsigned char, 8> packMit(float pDes, float vDes, float kpDes, float kdDes, float torqueDes,
                                     float maxTorqueNm) {
    float const torqueLimit = positiveTorqueLimit(maxTorqueNm);
    std::uint16_t p = floatToUint(pDes, RmdPMin, RmdPMax, 16);
    std::uint16_t v = floatToUint(vDes, RmdVMin, RmdVMax, 12);
    std::uint16_t kp = floatToUint(kpDes, RmdKpMin, RmdKpMax, 12);
    std::uint16_t kd = floatToUint(kdDes, RmdKdMin, RmdKdMax, 12);
    std::uint16_t torque = floatToUint(torqueDes, -torqueLimit, torqueLimit, 12);
    return {
        static_cast<unsigned char>((p >> 8) & 0xff),
        static_cast<unsigned char>(p & 0xff),
        static_cast<unsigned char>((v >> 4) & 0xff),
        static_cast<unsigned char>(((v & 0x000f) << 4) | ((kp >> 8) & 0x000f)),
        static_cast<unsigned char>(kp & 0xff),
        static_cast<unsigned char>((kd >> 4) & 0xff),
        static_cast<unsigned char>(((kd & 0x000f) << 4) | ((torque >> 8) & 0x000f)),
        static_cast<unsigned char>(torque & 0xff),
    };
}

MitFeedback parseMitReply(std::vector<unsigned char> const& data, int expectedMotorId, float maxTorqueNm) {
    return parseMitReplyBytes(data.data(), static_cast<int>(data.size()), expectedMotorId, maxTorqueNm);
}

MitFeedback parseMitReply(std::array<unsigned char, 8> const& data,
                          int length,
                          int expectedMotorId,
                          float maxTorqueNm) {
    return parseMitReplyBytes(data.data(), length, expectedMotorId, maxTorqueNm);
}

Status1Feedback parseStatus1Reply(std::vector<unsigned char> const& data) {
    Status1Feedback feedback;
    if (data.size() != 8 || data[0] != 0x9A) {
        return feedback;
    }
    feedback.temperatureC = static_cast<std::int8_t>(data[1]);
    feedback.mosTemperatureC = static_cast<std::int8_t>(data[2]);
    feedback.brakeReleaseCommandState = data[3];
    feedback.brakeReleaseCommandActive = data[3] != 0;
    feedback.voltageV = static_cast<float>(uint16Le(data[4], data[5])) * 0.1f;
    feedback.errorState = uint16Le(data[6], data[7]);
    feedback.valid = true;
    return feedback;
}

Status2Feedback parseStatus2Reply(std::vector<unsigned char> const& data, float torqueConstantNmPerAmp) {
    return parseStatus2ReplyBytes(data.data(), static_cast<int>(data.size()), torqueConstantNmPerAmp);
}

Status2Feedback parseStatus2Reply(std::array<unsigned char, 8> const& data,
                                  int length,
                                  float torqueConstantNmPerAmp) {
    return parseStatus2ReplyBytes(data.data(), length, torqueConstantNmPerAmp);
}

MultiturnAngleFeedback parseMultiturnAngleReply(std::vector<unsigned char> const& data) {
    MultiturnAngleFeedback feedback;
    if (data.size() != 8 || data[0] != 0x92) {
        return feedback;
    }
    feedback.rawAngleCentideg = int32Le(data[4], data[5], data[6], data[7]);
    feedback.angleDeg = static_cast<float>(feedback.rawAngleCentideg) * 0.01f;
    feedback.angleRad = feedback.angleDeg * Pi / 180.0f;
    feedback.valid = true;
    return feedback;
}

void applyMitFeedback(MotorActual& actual, MitFeedback const& feedback) {
    if (!feedback.valid) {
        return;
    }
    actual.pos = feedback.positionRad;
    actual.vel = feedback.velocityRadS;
    actual.statusWord = 0x0237;
    actual.errorCode = 0;
}

void applyStatus2Feedback(MotorActual& actual, Status2Feedback const& feedback) {
    if (!feedback.valid) {
        return;
    }
    actual.tor = feedback.estimatedTorqueNm;
    actual.temp = static_cast<short>(feedback.temperatureC);
}

} // namespace RmdCanSdk
