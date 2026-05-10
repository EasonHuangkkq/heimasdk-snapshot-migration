#pragma once

#include "rmd_can_sdk/rmd_ethercat_bindings.h"
#include "rmd_can_sdk/rmd_ethercat_target_frame.h"
#include "rmd_can_sdk/rmd_realtime_core.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace RmdCanSdk {

void readEthercatDomainSnapshot(std::uint8_t const* domainData,
                                std::size_t domainSize,
                                std::vector<EthercatPdoBinding> const& bindings,
                                MotorActualFrame& actuals);

void writeEthercatDomainSnapshot(std::uint8_t* domainData,
                                 std::size_t domainSize,
                                 std::vector<EthercatPdoBinding> const& bindings,
                                 MotorTargetFrame const& targets,
                                 MotorActualFrame const& actuals);

void writeEthercatDomainSnapshot(std::uint8_t* domainData,
                                 std::size_t domainSize,
                                 std::vector<EthercatPdoBinding> const& bindings,
                                 EthercatPackedTargetFrame const& targets,
                                 MotorActualFrame const& actuals);

} // namespace RmdCanSdk
