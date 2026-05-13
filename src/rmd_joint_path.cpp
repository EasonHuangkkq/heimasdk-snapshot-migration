#include "rmd_can_sdk/rmd_joint_path.h"

#include <stdexcept>

namespace RmdCanSdk {
namespace {

void setError(std::string* error, char const* text) {
    if (error != nullptr) {
        *error = text;
    }
}

double lerp(double start, double end, double t) {
    return start + (end - start) * t;
}

LegJointTargets interpolate(LegJointTargets const& start, LegJointTargets const& end, double t) {
    LegJointTargets out;
    out.hipPitchRad = lerp(start.hipPitchRad, end.hipPitchRad, t);
    out.kneePitchRad = lerp(start.kneePitchRad, end.kneePitchRad, t);
    out.anklePitchRad = lerp(start.anklePitchRad, end.anklePitchRad, t);
    out.ankleRollRad = lerp(start.ankleRollRad, end.ankleRollRad, t);
    return out;
}

} // namespace

bool validateLegJointPath(std::vector<LegJointWaypoint> const& path, std::string* error) {
    if (path.empty()) {
        setError(error, "joint path must contain at least one waypoint");
        return false;
    }
    if (path.front().timeMs < 0) {
        setError(error, "joint path first waypoint time must be non-negative");
        return false;
    }
    for (std::size_t i = 1; i < path.size(); ++i) {
        if (path[i].timeMs <= path[i - 1].timeMs) {
            setError(error, "joint path waypoint times must be strictly increasing");
            return false;
        }
    }
    if (error != nullptr) {
        error->clear();
    }
    return true;
}

LegJointTargets sampleLegJointPath(std::vector<LegJointWaypoint> const& path, int elapsedMs) {
    std::string error;
    if (!validateLegJointPath(path, &error)) {
        throw std::invalid_argument(error);
    }

    if (elapsedMs <= path.front().timeMs) {
        return path.front().joints;
    }
    if (elapsedMs >= path.back().timeMs) {
        return path.back().joints;
    }

    for (std::size_t i = 1; i < path.size(); ++i) {
        if (elapsedMs <= path[i].timeMs) {
            int const startTime = path[i - 1].timeMs;
            int const endTime = path[i].timeMs;
            double const t = static_cast<double>(elapsedMs - startTime) /
                             static_cast<double>(endTime - startTime);
            return interpolate(path[i - 1].joints, path[i].joints, t);
        }
    }

    return path.back().joints;
}

} // namespace RmdCanSdk
