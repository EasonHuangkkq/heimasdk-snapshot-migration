#include "rmd_can_sdk/rmd_safety.h"

namespace RmdCanSdk {
namespace {

float clampAndTrack(float value, float min, float max, bool& clamped) {
    float const out = clamp(value, min, max);
    if (out != value) {
        clamped = true;
    }
    return out;
}

} // namespace

LimitedTarget limitTarget(MotorTarget const& target, MotorParameters const& params) {
    LimitedTarget result;
    result.target = target;
    result.target.pos = clampAndTrack(target.pos, params.minimumPosition, params.maximumPosition, result.clamped);
    result.target.tor = clampAndTrack(target.tor, -params.maximumTorque, params.maximumTorque, result.clamped);
    result.target.kp = clampAndTrack(target.kp, RmdKpMin, RmdKpMax, result.clamped);
    result.target.kd = clampAndTrack(target.kd, RmdKdMin, RmdKdMax, result.clamped);
    return result;
}

void markActualStale(MotorActual& actual) {
    actual.statusWord = 0xffff;
    actual.errorCode = ErrorCodeFeedbackTimeout;
}

void markActualBackendFault(MotorActual& actual) {
    actual.statusWord = 0xffff;
    actual.errorCode = ErrorCodeBackendFault;
}

} // namespace RmdCanSdk
