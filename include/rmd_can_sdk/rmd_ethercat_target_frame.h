#pragma once

#include "rmd_can_sdk/heima_ecat_types.h"
#include "rmd_can_sdk/rmd_ethercat_mt_device.h"
#include "rmd_can_sdk/rmd_realtime_core.h"

#include <array>
#include <bitset>
#include <cstddef>
#include <cstdint>

namespace RmdCanSdk {

struct EthercatPackedTarget {
    int enabled = 0;
    EthercatMtDeviceRxProfile rxProfile = EthercatMtDeviceRxProfile::Standard;
    std::array<std::uint8_t, sizeof(HeimaPvtRxData)> rxBytes{};
};

struct EthercatPackedTargetFrame : FrameMetadata {
    std::array<EthercatPackedTarget, MaxRealtimeMotors> targets{};
    std::bitset<MaxRealtimeMotors> valid{};
};

EthercatPackedTarget packEthercatTargetForRealtime(MotorTarget const& target,
                                                   MotorParameters const& params,
                                                   EthercatMtDeviceRxProfile rxProfile);

int applyEthercatCommandWatchdog(EthercatPackedTargetFrame& frame,
                                 RealtimeClock::time_point now,
                                 RealtimeClock::duration timeout);

} // namespace RmdCanSdk
