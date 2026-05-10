#pragma once

#include "rmd_can_sdk/rmd_motor_registry.h"
#include "rmd_can_sdk/rmd_ethercat_mt_device.h"
#include "rmd_can_sdk/rmd_types.h"

#include <cstddef>
#include <string>
#include <vector>

namespace RmdCanSdk {

struct EthercatPdoBinding {
    std::size_t globalIndex = 0;
    std::size_t backendLocalIndex = 0;
    int alias = 0;
    int master = 0;
    int slave = 0;
    int domain = 0;
    int rxOffset = -1;
    int txOffset = -1;
    EthercatMtDeviceRxProfile rxProfile = EthercatMtDeviceRxProfile::Standard;
    EthercatMtDeviceTxProfile txProfile = EthercatMtDeviceTxProfile::Extended;
    std::size_t rxSize = 0;
    std::size_t txSize = 0;
    std::string type;
    MotorParameters parameters;
};

std::vector<EthercatPdoBinding> buildEthercatPdoBindings(MotorRegistry const& registry, std::size_t backendIndex);

void assignEthercatPdoProfiles(std::vector<EthercatPdoBinding>& bindings,
                               std::vector<char> const& operatingModes);

void assignEthercatPdoOffsets(std::vector<EthercatPdoBinding>& bindings,
                              std::size_t backendLocalIndex,
                              int rxOffset,
                              int txOffset);

int resolveEthercatSlavePosition(EthercatPdoBinding const& binding);

} // namespace RmdCanSdk
