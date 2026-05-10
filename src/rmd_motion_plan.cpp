#include "rmd_can_sdk/rmd_motion_plan.h"

#include <cmath>
#include <sstream>

namespace RmdCanSdk {

SineTargetLimitCheck checkSineTargetWithinLimits(SineTargetSpec const& spec,
                                                MotorParameters const& parameters,
                                                float marginRad) {
    SineTargetLimitCheck check;
    marginRad = std::max(0.0f, marginRad);
    float const amplitude = std::fabs(spec.amplitudeRad);
    check.minTargetRad = spec.centerRad - amplitude;
    check.maxTargetRad = spec.centerRad + amplitude;
    check.minAllowedRad = parameters.minimumPosition + marginRad;
    check.maxAllowedRad = parameters.maximumPosition - marginRad;

    if (!std::isfinite(spec.centerRad) || !std::isfinite(spec.amplitudeRad) ||
        !std::isfinite(spec.phaseRad) ||
        !std::isfinite(spec.kp) || !std::isfinite(spec.kd) ||
        !std::isfinite(spec.feedforwardTorqueNm)) {
        check.reason = "sine target contains a non-finite value";
        return check;
    }
    if (spec.amplitudeRad < 0.0f) {
        check.reason = "sine amplitude must be non-negative";
        return check;
    }
    if (check.minAllowedRad > check.maxAllowedRad) {
        check.reason = "position limit margin leaves no valid range";
        return check;
    }
    if (check.minTargetRad < check.minAllowedRad || check.maxTargetRad > check.maxAllowedRad) {
        std::ostringstream reason;
        reason << "target range [" << check.minTargetRad << ", " << check.maxTargetRad
               << "] exceeds allowed range [" << check.minAllowedRad << ", " << check.maxAllowedRad << "]";
        check.reason = reason.str();
        return check;
    }

    check.valid = true;
    return check;
}

SineTargetLimitCheck checkAngleWithinLimits(float angleRad,
                                            MotorParameters const& parameters,
                                            float marginRad) {
    SineTargetSpec spec;
    spec.centerRad = angleRad;
    spec.amplitudeRad = 0.0f;
    return checkSineTargetWithinLimits(spec, parameters, marginRad);
}

float sineTargetAt(SineTargetSpec const& spec, float elapsedSec, float frequencyHz) {
    return spec.centerRad + spec.amplitudeRad * std::sin(2.0f * Pi * frequencyHz * elapsedSec + spec.phaseRad);
}

} // namespace RmdCanSdk
