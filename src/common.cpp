#include "rmd_can_sdk/common.h"

#include <bit>
#include <cstdint>

namespace RmdCanSdk {

unsigned short singleToHalf(float value) {
    auto bits = std::bit_cast<std::uint32_t>(value);
    std::uint32_t sign = bits & 0x80000000u;
    int exp = static_cast<int>(bits & 0x7f800000u);
    std::uint32_t mantissa = bits & 0x007fffffu;
    std::uint32_t halfSign = sign >> 16;
    if (exp == 0x7f800000) {
        std::uint32_t nanBit = mantissa == 0 ? 0 : 0x0200;
        return static_cast<unsigned short>(halfSign | 0x7c00 | nanBit | (mantissa >> 13));
    }
    int halfExp = (exp >> 23) - 127 + 15;
    if (halfExp >= 0x1f) {
        return static_cast<unsigned short>(halfSign | 0x7c00);
    }
    if (halfExp <= 0) {
        if (14 - halfExp > 24) {
            return static_cast<unsigned short>(halfSign);
        }
        mantissa |= 0x00800000u;
        std::uint32_t halfMantissa = mantissa >> (14 - halfExp);
        std::uint32_t roundBit = 1u << (13 - halfExp);
        if ((mantissa & roundBit) != 0 && (mantissa & (3 * roundBit - 1)) != 0) {
            halfMantissa++;
        }
        return static_cast<unsigned short>(halfSign | halfMantissa);
    }
    std::uint32_t halfExpBits = static_cast<std::uint32_t>(halfExp) << 10;
    std::uint32_t halfMantissa = mantissa >> 13;
    if ((mantissa & 0x00001000u) != 0 && (mantissa & 0x00002fffu) != 0) {
        return static_cast<unsigned short>((halfSign | halfExpBits | halfMantissa) + 1);
    }
    return static_cast<unsigned short>(halfSign | halfExpBits | halfMantissa);
}

float halfToSingle(unsigned short value) {
    if ((value & 0x7fff) == 0) {
        return std::bit_cast<float>(static_cast<std::uint32_t>(value) << 16);
    }
    std::uint32_t halfSign = value & 0x8000u;
    int halfExp = value & 0x7c00;
    std::uint32_t halfMantissa = value & 0x03ffu;
    std::uint32_t sign = halfSign << 16;
    if (halfExp == 0x7c00) {
        return std::bit_cast<float>(sign | 0x7f800000u | (halfMantissa << 13));
    }
    int exp = (halfExp >> 10) - 15 + 127;
    if (halfExp == 0) {
        int leading = std::countl_zero(halfMantissa) - 22;
        exp = (exp - leading) << 23;
        std::uint32_t mantissa = (halfMantissa << (14 + leading)) & 0x7fffffu;
        return std::bit_cast<float>(sign | static_cast<std::uint32_t>(exp) | mantissa);
    }
    return std::bit_cast<float>(sign | (static_cast<std::uint32_t>(exp) << 23) | (halfMantissa << 13));
}

} // namespace RmdCanSdk

namespace DriverSDK {

unsigned short single2half(float value) {
    return RmdCanSdk::singleToHalf(value);
}

float half2single(unsigned short value) {
    return RmdCanSdk::halfToSingle(value);
}

} // namespace DriverSDK

