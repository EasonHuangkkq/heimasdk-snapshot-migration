#pragma once

#include "rmd_can_sdk/heima_ecat_types.h"
#include "rmd_can_sdk/rmd_ethercat_target_frame.h"
#include "rmd_can_sdk/rmd_types.h"

namespace RmdCanSdk {

using EthercatRxPdoBytes = HeimaDriverRxData;
using EthercatTxPdoBytes = HeimaDriverTxData;

std::uint16_t ethercatMtDeviceControlWord(int enabled, std::uint16_t statusWord);
HeimaStandardRxData packEthercatStandardRxPdo(MotorTarget const& target, MotorParameters const& params);
HeimaPvtRxData packEthercatPvtRxPdo(MotorTarget const& target, MotorParameters const& params);
EthercatRxPdoBytes packEthercatRxPdo(MotorTarget const& target, MotorParameters const& params);
EthercatPackedTarget packEthercatTargetForRealtime(MotorTarget const& target,
                                                   MotorParameters const& params,
                                                   EthercatMtDeviceRxProfile rxProfile);
MotorActual parseEthercatStandardTxPdo(HeimaStandardTxData const& tx, MotorParameters const& params);
MotorActual parseEthercatTxPdo(EthercatTxPdoBytes const& tx, MotorParameters const& params);

} // namespace RmdCanSdk
