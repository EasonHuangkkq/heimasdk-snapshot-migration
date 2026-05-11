#pragma once

#include "rmd_can_sdk/heima_driver_sdk.h"
#include "rmd_can_sdk/rmd_types.h"

#include <vector>

namespace RmdCanSdk {

MotorParameters const* findParamsForAlias(Config const& config, int alias);
bool isActiveMotor(DriverSDK::DriverSDK& sdk, int motorIndex);
bool operationEnabled(unsigned short statusWord);
bool feedbackReady(DriverSDK::DriverSDK& sdk,
                   std::vector<DriverSDK::motorActualStruct> const& actuals,
                   int status);
void disableMotors(DriverSDK::DriverSDK& sdk,
                   std::vector<DriverSDK::motorTargetStruct>& targets,
                   int repeatCount = 50,
                   int periodMs = 5);
void applyRealtimeSettings(int rtPriority);

} // namespace RmdCanSdk
