#pragma once

#include "rmd_can_sdk/rmd_types.h"

#include <string>

namespace RmdCanSdk {

struct SineTargetSpec {
    float centerRad = 0.0f;
    float amplitudeRad = 0.0f;
    float phaseRad = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
    float feedforwardTorqueNm = 0.0f;
};

struct SineTargetLimitCheck {
    bool valid = false;
    float minTargetRad = 0.0f;
    float maxTargetRad = 0.0f;
    float minAllowedRad = 0.0f;
    float maxAllowedRad = 0.0f;
    std::string reason;
};

SineTargetLimitCheck checkSineTargetWithinLimits(SineTargetSpec const& spec,
                                                MotorParameters const& parameters,
                                                float marginRad);

SineTargetLimitCheck checkAngleWithinLimits(float angleRad,
                                            MotorParameters const& parameters,
                                            float marginRad);

float sineTargetAt(SineTargetSpec const& spec, float elapsedSec, float frequencyHz);

} // namespace RmdCanSdk
