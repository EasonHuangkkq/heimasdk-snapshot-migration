#pragma once

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>

namespace RmdCanSdk {

struct EthercatPdoEntrySpec {
    std::uint16_t index = 0;
    std::uint8_t subindex = 0;
    std::uint8_t bitLength = 0;
};

enum class EthercatMtDeviceRxProfile {
    Standard,
    Pvt,
};

enum class EthercatMtDeviceTxProfile {
    Standard,
    Extended,
};

struct EthercatMtDevicePdoProfileSpec {
    std::uint16_t pdoIndex = 0;
    std::vector<EthercatPdoEntrySpec> entries;
};

struct EthercatMtDevicePdoSpec {
    std::uint32_t vendorId = 0;
    std::uint32_t productCode = 0;
    std::uint16_t rxAssignmentIndex = 0;
    std::uint16_t txAssignmentIndex = 0;
};

EthercatMtDevicePdoSpec const& ethercatMtDevicePdoSpec();
EthercatMtDeviceRxProfile ethercatMtDeviceRxProfileForMode(int mode);
EthercatMtDevicePdoProfileSpec const& ethercatMtDeviceRxPdoSpec(EthercatMtDeviceRxProfile profile);
EthercatMtDevicePdoProfileSpec const& ethercatMtDeviceTxPdoSpec(EthercatMtDeviceTxProfile profile);
std::size_t ethercatMtDeviceRxPdoSize(EthercatMtDeviceRxProfile profile);
std::size_t ethercatMtDeviceTxPdoSize(EthercatMtDeviceTxProfile profile);
bool isEthercatMtDeviceType(std::string const& type);

} // namespace RmdCanSdk
