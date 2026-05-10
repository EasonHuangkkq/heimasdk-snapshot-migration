#include "rmd_can_sdk/rmd_ethercat_pdo.h"

#include <cmath>
#include <cstring>

namespace RmdCanSdk {
namespace {

// MT_Device EtherCAT follows the validated heimaSDK path: encoderResolution is
// output-side counts per revolution, and countBias is already in counts.
std::int32_t radToCounts(float rad, MotorParameters const& params) {
    float const revolutions = rad / (2.0f * Pi);
    return static_cast<std::int32_t>(std::lround(params.polarity * revolutions * params.encoderResolution +
                                                params.countBias));
}

std::int32_t radDeltaToCounts(float rad, MotorParameters const& params) {
    float const revolutions = rad / (2.0f * Pi);
    return static_cast<std::int32_t>(std::lround(params.polarity * revolutions * params.encoderResolution));
}

float countsToRad(std::int32_t counts, MotorParameters const& params) {
    if (params.encoderResolution == 0.0f) {
        return 0.0f;
    }
    return params.polarity * (static_cast<float>(counts) - params.countBias) / params.encoderResolution * 2.0f * Pi;
}

float countDeltaToRad(std::int32_t counts, MotorParameters const& params) {
    if (params.encoderResolution == 0.0f) {
        return 0.0f;
    }
    return params.polarity * static_cast<float>(counts) / params.encoderResolution * 2.0f * Pi;
}

std::int16_t torqueToRaw(float torque, MotorParameters const& params) {
    float const scale = params.torqueConstant * params.ratedCurrent;
    if (scale == 0.0f) {
        return 0;
    }
    return static_cast<std::int16_t>(params.polarity * 1000.0f * torque / scale);
}

std::int16_t torqueToRawLimited(float torque, MotorParameters const& params, int maxCurrent) {
    float const scale = params.torqueConstant * params.ratedCurrent;
    if (scale == 0.0f) {
        return 0;
    }
    int raw = static_cast<int>(std::lround(params.polarity * 1000.0f * torque / scale));
    int const limit = maxCurrent < 0 ? 0 : maxCurrent;
    if (raw > limit) {
        raw = limit;
    } else if (raw < -limit) {
        raw = -limit;
    }
    return static_cast<std::int16_t>(raw);
}

float rawToTorque(std::int16_t raw, MotorParameters const& params) {
    return params.polarity * static_cast<float>(raw) / 1000.0f * params.ratedCurrent * params.torqueConstant;
}

float clampPosition(float value, MotorParameters const& params) {
    return clamp(value, params.minimumPosition, params.maximumPosition);
}

float clampTorque(float value, MotorParameters const& params) {
    return clamp(value, -params.maximumTorque, params.maximumTorque);
}

} // namespace

std::uint16_t ethercatMtDeviceControlWord(int enabled, std::uint16_t statusWord) {
    if (enabled == -1) {
        return 0x0086;
    }
    if (enabled != 1) {
        return 0x0006;
    }

    switch (statusWord & 0x007f) {
    case 0x0031:
        return 0x0007;
    case 0x0033:
    case 0x0037:
        return 0x000f;
    default:
        return 0x0006;
    }
}

HeimaStandardRxData packEthercatStandardRxPdo(MotorTarget const& target, MotorParameters const& params) {
    HeimaStandardRxData rx;
    rx.ControlWord = target.enabled == 1 ? 0x000f : 0x0006;
    rx.TargetPosition = radToCounts(clampPosition(target.pos, params), params);
    rx.TargetVelocity = radDeltaToCounts(target.vel, params);
    rx.Mode = static_cast<char>(target.mode);
    rx.MaxTorque = static_cast<std::uint16_t>(target.maxCurrent < 0 ? 0 : target.maxCurrent);
    if (target.mode == 8) {
        rx.TargetTorque = 0;
    } else if (target.mode == 10) {
        rx.TargetTorque = torqueToRawLimited(clampTorque(target.tor, params), params, target.maxCurrent);
    } else {
        rx.TargetTorque = torqueToRaw(clampTorque(target.tor, params), params);
    }
    rx.Undefined = 0;
    return rx;
}

HeimaPvtRxData packEthercatPvtRxPdo(MotorTarget const& target, MotorParameters const& params) {
    HeimaPvtRxData rx;
    rx.ControlWord = target.enabled == 1 ? 0x000f : 0x0006;
    rx.TargetPosition = radToCounts(clampPosition(target.pos, params), params);
    rx.TargetVelocity = radDeltaToCounts(target.vel, params);
    rx.Mode = static_cast<char>(target.mode);
    if (target.mode == 8) {
        rx.TargetTorque = 0;
        rx.PvtKp = 0;
        rx.PvtKd = 0;
    } else if (target.mode == 10) {
        rx.TargetTorque = torqueToRawLimited(clampTorque(target.tor, params), params, target.maxCurrent);
        rx.PvtKp = 0;
        rx.PvtKd = 0;
    } else {
        rx.TargetTorque = torqueToRaw(clampTorque(target.tor, params), params);
        rx.PvtKp = target.mode == 5 ? static_cast<std::int32_t>(std::lround(target.kp * 1000.0f)) : 0;
        rx.PvtKd = target.mode == 5 ? static_cast<std::int32_t>(std::lround(target.kd * 1000.0f)) : 0;
    }
    rx.Undefined = 0;
    return rx;
}

EthercatRxPdoBytes packEthercatRxPdo(MotorTarget const& target, MotorParameters const& params) {
    return packEthercatPvtRxPdo(target, params);
}

EthercatPackedTarget packEthercatTargetForRealtime(MotorTarget const& target,
                                                   MotorParameters const& params,
                                                   EthercatMtDeviceRxProfile rxProfile) {
    EthercatPackedTarget packed;
    packed.enabled = target.enabled;
    packed.rxProfile = rxProfile;
    if (rxProfile == EthercatMtDeviceRxProfile::Standard) {
        HeimaStandardRxData const rx = packEthercatStandardRxPdo(target, params);
        std::memcpy(packed.rxBytes.data(), &rx, sizeof(rx));
    } else {
        HeimaPvtRxData const rx = packEthercatPvtRxPdo(target, params);
        std::memcpy(packed.rxBytes.data(), &rx, sizeof(rx));
    }
    return packed;
}

MotorActual parseEthercatStandardTxPdo(HeimaStandardTxData const& tx, MotorParameters const& params) {
    MotorActual actual;
    actual.encoderCount = tx.ActualPosition;
    actual.pos = countsToRad(tx.ActualPosition, params);
    actual.vel = countDeltaToRad(tx.ActualVelocity, params);
    actual.tor = rawToTorque(tx.ActualTorque, params);
    actual.statusWord = tx.StatusWord;
    actual.errorCode = tx.ErrorCode;
    return actual;
}

MotorActual parseEthercatTxPdo(EthercatTxPdoBytes const& tx, MotorParameters const& params) {
    MotorActual actual = parseEthercatStandardTxPdo(
        HeimaStandardTxData{tx.StatusWord,
                            tx.ActualPosition,
                            tx.ActualVelocity,
                            tx.ActualTorque,
                            tx.ErrorCode,
                            tx.ModeDisplay,
                            tx.Undefined},
        params);
    actual.temp = tx.MotorTemperature;
    actual.driveTemp = tx.DriveTemperature;
    actual.voltage = tx.Voltage;
    return actual;
}

} // namespace RmdCanSdk
