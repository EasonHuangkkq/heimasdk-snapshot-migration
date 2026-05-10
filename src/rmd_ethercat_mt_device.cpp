#include "rmd_can_sdk/rmd_ethercat_mt_device.h"

#include "rmd_can_sdk/heima_ecat_types.h"

#include <cctype>
#include <stdexcept>

namespace RmdCanSdk {

EthercatMtDevicePdoSpec const& ethercatMtDevicePdoSpec() {
    static EthercatMtDevicePdoSpec const spec{
        0x00202008u,
        0x00000000u,
        0x1c12,
        0x1c13,
    };
    return spec;
}

EthercatMtDeviceRxProfile ethercatMtDeviceRxProfileForMode(int mode) {
    return mode == 5 ? EthercatMtDeviceRxProfile::Pvt : EthercatMtDeviceRxProfile::Standard;
}

EthercatMtDevicePdoProfileSpec const& ethercatMtDeviceRxPdoSpec(EthercatMtDeviceRxProfile profile) {
    static EthercatMtDevicePdoProfileSpec const standard{
        0x1600,
        {
            {0x6040, 0x00, 16},
            {0x607a, 0x00, 32},
            {0x60ff, 0x00, 32},
            {0x6071, 0x00, 16},
            {0x6072, 0x00, 16},
            {0x6060, 0x00, 8},
            {0x2ffd, 0x00, 8},
        },
    };
    static EthercatMtDevicePdoProfileSpec const pvt{
        0x1601,
        {
            {0x6040, 0x00, 16},
            {0x607a, 0x00, 32},
            {0x60ff, 0x00, 32},
            {0x6071, 0x00, 16},
            {0x2000, 0x00, 32},
            {0x2001, 0x00, 32},
            {0x6060, 0x00, 8},
            {0x2ffd, 0x00, 8},
        },
    };
    switch (profile) {
    case EthercatMtDeviceRxProfile::Standard:
        return standard;
    case EthercatMtDeviceRxProfile::Pvt:
        return pvt;
    }
    throw std::logic_error("unknown MT_Device RxPDO profile");
}

EthercatMtDevicePdoProfileSpec const& ethercatMtDeviceTxPdoSpec(EthercatMtDeviceTxProfile profile) {
    static EthercatMtDevicePdoProfileSpec const standard{
        0x1a00,
        {
            {0x6041, 0x00, 16},
            {0x6064, 0x00, 32},
            {0x606c, 0x00, 32},
            {0x6077, 0x00, 16},
            {0x603f, 0x00, 16},
            {0x6061, 0x00, 8},
            {0x2ffe, 0x00, 8},
        },
    };
    static EthercatMtDevicePdoProfileSpec const extended{
        0x1a02,
        {
            {0x6041, 0x00, 16},
            {0x6064, 0x00, 32},
            {0x606c, 0x00, 32},
            {0x6077, 0x00, 16},
            {0x603f, 0x00, 16},
            {0x2009, 0x00, 16},
            {0x200c, 0x00, 16},
            {0x200a, 0x00, 16},
            {0x6061, 0x00, 8},
            {0x2ffe, 0x00, 8},
        },
    };
    switch (profile) {
    case EthercatMtDeviceTxProfile::Standard:
        return standard;
    case EthercatMtDeviceTxProfile::Extended:
        return extended;
    }
    throw std::logic_error("unknown MT_Device TxPDO profile");
}

std::size_t ethercatMtDeviceRxPdoSize(EthercatMtDeviceRxProfile profile) {
    switch (profile) {
    case EthercatMtDeviceRxProfile::Standard:
        return sizeof(HeimaStandardRxData);
    case EthercatMtDeviceRxProfile::Pvt:
        return sizeof(HeimaPvtRxData);
    }
    throw std::logic_error("unknown MT_Device RxPDO profile");
}

std::size_t ethercatMtDeviceTxPdoSize(EthercatMtDeviceTxProfile profile) {
    switch (profile) {
    case EthercatMtDeviceTxProfile::Standard:
        return sizeof(HeimaStandardTxData);
    case EthercatMtDeviceTxProfile::Extended:
        return sizeof(HeimaDriverTxData);
    }
    throw std::logic_error("unknown MT_Device TxPDO profile");
}

bool isEthercatMtDeviceType(std::string const& type) {
    std::string normalized;
    normalized.reserve(type.size());
    for (unsigned char ch : type) {
        if (ch == '_' || ch == '-') {
            continue;
        }
        normalized.push_back(static_cast<char>(std::toupper(ch)));
    }
    return normalized == "MTDEVICE";
}

} // namespace RmdCanSdk
