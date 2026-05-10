#pragma once

#include "rmd_can_sdk/rmd_protocol.h"
#include "rmd_can_sdk/rmd_types.h"

namespace RmdCanSdk {

constexpr unsigned short ErrorCodeFeedbackTimeout = 1;
constexpr unsigned short ErrorCodeBackendFault = 2;
constexpr unsigned short ErrorCodeTargetLimited = 3;

struct LimitedTarget {
    MotorTarget target;
    bool clamped = false;
};

LimitedTarget limitTarget(MotorTarget const& target, MotorParameters const& params);
void markActualStale(MotorActual& actual);
void markActualBackendFault(MotorActual& actual);

} // namespace RmdCanSdk
