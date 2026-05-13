#pragma once

#include "rmd_can_sdk/rmd_leg_kinematics.h"

#include <string>
#include <vector>

namespace RmdCanSdk {

struct LegJointWaypoint {
    int timeMs = 0;
    LegJointTargets joints;
};

bool validateLegJointPath(std::vector<LegJointWaypoint> const& path, std::string* error = nullptr);
LegJointTargets sampleLegJointPath(std::vector<LegJointWaypoint> const& path, int elapsedMs);

} // namespace RmdCanSdk
