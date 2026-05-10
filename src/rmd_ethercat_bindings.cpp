#include "rmd_can_sdk/rmd_ethercat_bindings.h"

#include <stdexcept>

namespace RmdCanSdk {

std::vector<EthercatPdoBinding> buildEthercatPdoBindings(MotorRegistry const& registry, std::size_t backendIndex) {
    if (backendIndex >= registry.backends().size()) {
        throw std::out_of_range("EtherCAT backend index outside registry");
    }

    BackendGroup const& backend = registry.backends()[backendIndex];
    if (backend.bus != MotorBus::Ethercat) {
        return {};
    }

    std::vector<EthercatPdoBinding> bindings;
    bindings.reserve(backend.motorIndexes.size());
    for (std::size_t const motorIndex : backend.motorIndexes) {
        RuntimeMotor const& motor = registry.motors().at(motorIndex);
        EthercatPdoBinding binding;
        binding.globalIndex = motor.globalIndex;
        binding.backendLocalIndex = motor.backendLocalIndex;
        binding.alias = motor.alias;
        binding.master = motor.master;
        binding.slave = motor.ethercatSlave;
        binding.domain = motor.domain;
        binding.type = motor.type;
        binding.parameters = motor.parameters;
        if (isEthercatMtDeviceType(binding.type)) {
            binding.rxProfile = EthercatMtDeviceRxProfile::Standard;
            binding.txProfile = EthercatMtDeviceTxProfile::Extended;
            binding.rxSize = ethercatMtDeviceRxPdoSize(binding.rxProfile);
            binding.txSize = ethercatMtDeviceTxPdoSize(binding.txProfile);
        }
        bindings.push_back(binding);
    }
    return bindings;
}

void assignEthercatPdoProfiles(std::vector<EthercatPdoBinding>& bindings, std::vector<char> const& operatingModes) {
    for (EthercatPdoBinding& binding : bindings) {
        if (!isEthercatMtDeviceType(binding.type)) {
            continue;
        }
        if (binding.globalIndex >= operatingModes.size()) {
            throw std::out_of_range("EtherCAT operating mode missing for binding global index");
        }
        binding.rxProfile = ethercatMtDeviceRxProfileForMode(static_cast<int>(operatingModes[binding.globalIndex]));
        binding.txProfile = EthercatMtDeviceTxProfile::Extended;
        binding.rxSize = ethercatMtDeviceRxPdoSize(binding.rxProfile);
        binding.txSize = ethercatMtDeviceTxPdoSize(binding.txProfile);
    }
}

void assignEthercatPdoOffsets(std::vector<EthercatPdoBinding>& bindings,
                              std::size_t backendLocalIndex,
                              int rxOffset,
                              int txOffset) {
    for (EthercatPdoBinding& binding : bindings) {
        if (binding.backendLocalIndex == backendLocalIndex) {
            binding.rxOffset = rxOffset;
            binding.txOffset = txOffset;
            return;
        }
    }
    throw std::out_of_range("EtherCAT PDO binding local index not found");
}

int resolveEthercatSlavePosition(EthercatPdoBinding const& binding) {
    if (binding.slave >= 0) {
        return binding.slave;
    }
    if (binding.alias <= 0) {
        throw std::invalid_argument("EtherCAT binding has no alias for slave-position fallback");
    }
    return binding.alias - 1;
}

} // namespace RmdCanSdk
