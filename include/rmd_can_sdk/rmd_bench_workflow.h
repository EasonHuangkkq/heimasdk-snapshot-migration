#pragma once

#include "rmd_can_sdk/heima_driver_sdk.h"
#include "rmd_can_sdk/rmd_types.h"

#include <vector>

namespace RmdCanSdk {

struct RealtimeThreadSettings {
    int priority = 0;
    int cpu = -1;
    bool valid = true;
};

MotorParameters const* findParamsForAlias(Config const& config, int alias);
bool isActiveMotor(DriverSDK::DriverSDK& sdk, int motorIndex);
bool operationEnabled(unsigned short statusWord);
bool operationFeedbackReady(std::vector<int> const& activeMotors,
                            std::vector<DriverSDK::motorActualStruct> const& actuals,
                            int status);
bool operationFeedbackReady(DriverSDK::DriverSDK& sdk,
                            std::vector<DriverSDK::motorActualStruct> const& actuals,
                            int status);
int parseNonNegativeInt(char const* text, int fallback);
bool recordConsecutiveReady(bool ready, int& consecutiveReady, int requiredReadySamples);
bool feedbackReady(DriverSDK::DriverSDK& sdk,
                   std::vector<DriverSDK::motorActualStruct> const& actuals,
                   int status);
void disableMotors(DriverSDK::DriverSDK& sdk,
                   std::vector<DriverSDK::motorTargetStruct>& targets,
                   int repeatCount = 50,
                   int periodMs = 5);
RealtimeThreadSettings parseRealtimeSettings(char const* priorityText,
                                             char const* cpuText,
                                             int defaultPriority,
                                             int defaultCpu);
void applyRealtimeSettings(int rtPriority);
void applyRealtimeSettings(RealtimeThreadSettings const& settings, char const* label);

} // namespace RmdCanSdk
