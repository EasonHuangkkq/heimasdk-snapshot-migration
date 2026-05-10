#include "rmd_can_sdk/rmd_ethercat_snapshot.h"

#include "rmd_can_sdk/rmd_ethercat_pdo.h"
#include "rmd_can_sdk/rmd_safety.h"

#include <array>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>

namespace RmdCanSdk {
namespace {

bool hasByteRange(int offset, std::size_t extent, std::size_t domainSize) {
    if (offset < 0) {
        return false;
    }
    std::size_t const begin = static_cast<std::size_t>(offset);
    return begin <= domainSize && extent <= domainSize - begin;
}

int rawDebugLimit() {
    static int const limit = [] {
        char const* value = std::getenv("RMD_ECAT_DEBUG_RAW");
        if (value == nullptr) {
            return 0;
        }
        int const parsed = std::atoi(value);
        return parsed > 0 ? parsed : 20;
    }();
    return limit;
}

} // namespace

void readEthercatDomainSnapshot(std::uint8_t const* domainData,
                                std::size_t domainSize,
                                std::vector<EthercatPdoBinding> const& bindings,
                                MotorActualFrame& actuals) {
    int const debugLimit = rawDebugLimit();
    for (EthercatPdoBinding const& binding : bindings) {
        if (binding.globalIndex >= MaxRealtimeMotors) {
            continue;
        }

        std::size_t const txSize =
            binding.txSize == 0 ? ethercatMtDeviceTxPdoSize(binding.txProfile) : binding.txSize;
        if (domainData == nullptr || !hasByteRange(binding.txOffset, txSize, domainSize)) {
            markActualStale(actuals.actuals[binding.globalIndex]);
            actuals.valid.reset(binding.globalIndex);
            actuals.stale.set(binding.globalIndex);
            continue;
        }

        HeimaStandardTxData standardTx{};
        EthercatTxPdoBytes extendedTx{};
        MotorActual actual;
        if (binding.txProfile == EthercatMtDeviceTxProfile::Standard) {
            std::memcpy(&standardTx, domainData + binding.txOffset, sizeof(standardTx));
            actual = parseEthercatStandardTxPdo(standardTx, binding.parameters);
        } else {
            std::memcpy(&extendedTx, domainData + binding.txOffset, sizeof(extendedTx));
            actual = parseEthercatTxPdo(extendedTx, binding.parameters);
            standardTx = HeimaStandardTxData{extendedTx.StatusWord,
                                             extendedTx.ActualPosition,
                                             extendedTx.ActualVelocity,
                                             extendedTx.ActualTorque,
                                             extendedTx.ErrorCode,
                                             extendedTx.ModeDisplay,
                                             extendedTx.Undefined};
        }
        if (debugLimit > 0) {
            static std::array<int, MaxRealtimeMotors> debugPrintsByMotor{};
            int& debugPrints = debugPrintsByMotor[binding.globalIndex];
            if (debugPrints < debugLimit) {
                std::cerr << "raw ecat alias=" << binding.alias
                          << " slave=" << binding.slave
                          << " txOffset=" << binding.txOffset
                          << " status=0x" << std::hex << std::setw(4) << std::setfill('0') << standardTx.StatusWord
                          << " error=0x" << std::setw(4) << standardTx.ErrorCode
                          << std::dec << std::setfill(' ')
                          << " pos_counts=" << standardTx.ActualPosition
                          << " vel_counts=" << standardTx.ActualVelocity
                          << " torque_raw=" << standardTx.ActualTorque
                          << " mode=" << static_cast<int>(standardTx.ModeDisplay)
                          << " temp=" << actual.temp
                          << " drive_temp=" << actual.driveTemp
                          << " voltage=" << actual.voltage << "\n";
                debugPrints++;
            }
        }
        actuals.actuals[binding.globalIndex] = actual;
        actuals.valid.set(binding.globalIndex);
        actuals.stale.reset(binding.globalIndex);
    }
}

void writeEthercatDomainSnapshot(std::uint8_t* domainData,
                                 std::size_t domainSize,
                                 std::vector<EthercatPdoBinding> const& bindings,
                                 MotorTargetFrame const& targets,
                                 MotorActualFrame const& actuals) {
    for (EthercatPdoBinding const& binding : bindings) {
        if (binding.globalIndex >= MaxRealtimeMotors) {
            continue;
        }
        std::size_t const rxSize =
            binding.rxSize == 0 ? ethercatMtDeviceRxPdoSize(binding.rxProfile) : binding.rxSize;
        if (domainData == nullptr || !hasByteRange(binding.rxOffset, rxSize, domainSize)) {
            continue;
        }

        MotorTarget target;
        if (targets.valid.test(binding.globalIndex)) {
            target = targets.targets[binding.globalIndex];
        }

        std::uint16_t statusWord = 0xffff;
        if (actuals.valid.test(binding.globalIndex)) {
            statusWord = actuals.actuals[binding.globalIndex].statusWord;
        }
        if (binding.rxProfile == EthercatMtDeviceRxProfile::Standard) {
            HeimaStandardRxData rx = packEthercatStandardRxPdo(target, binding.parameters);
            rx.ControlWord = ethercatMtDeviceControlWord(target.enabled, statusWord);
            std::memcpy(domainData + binding.rxOffset, &rx, sizeof(rx));
        } else {
            HeimaPvtRxData rx = packEthercatPvtRxPdo(target, binding.parameters);
            rx.ControlWord = ethercatMtDeviceControlWord(target.enabled, statusWord);
            std::memcpy(domainData + binding.rxOffset, &rx, sizeof(rx));
        }
    }
}

void writeEthercatDomainSnapshot(std::uint8_t* domainData,
                                 std::size_t domainSize,
                                 std::vector<EthercatPdoBinding> const& bindings,
                                 EthercatPackedTargetFrame const& targets,
                                 MotorActualFrame const& actuals) {
    for (EthercatPdoBinding const& binding : bindings) {
        if (binding.globalIndex >= MaxRealtimeMotors) {
            continue;
        }
        std::size_t const rxSize =
            binding.rxSize == 0 ? ethercatMtDeviceRxPdoSize(binding.rxProfile) : binding.rxSize;
        if (domainData == nullptr || !hasByteRange(binding.rxOffset, rxSize, domainSize)) {
            continue;
        }

        EthercatPackedTarget target;
        if (targets.valid.test(binding.globalIndex)) {
            target = targets.targets[binding.globalIndex];
        }

        std::uint16_t statusWord = 0xffff;
        if (actuals.valid.test(binding.globalIndex)) {
            statusWord = actuals.actuals[binding.globalIndex].statusWord;
        }
        if (binding.rxProfile == EthercatMtDeviceRxProfile::Standard) {
            HeimaStandardRxData rx{};
            std::memcpy(&rx, target.rxBytes.data(), sizeof(rx));
            rx.ControlWord = ethercatMtDeviceControlWord(target.enabled, statusWord);
            std::memcpy(domainData + binding.rxOffset, &rx, sizeof(rx));
        } else {
            HeimaPvtRxData rx{};
            std::memcpy(&rx, target.rxBytes.data(), sizeof(rx));
            rx.ControlWord = ethercatMtDeviceControlWord(target.enabled, statusWord);
            std::memcpy(domainData + binding.rxOffset, &rx, sizeof(rx));
        }
    }
}

} // namespace RmdCanSdk
