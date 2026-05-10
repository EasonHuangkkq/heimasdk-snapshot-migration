#include "rmd_can_sdk/heima_ecat_types.h"

#include <bit>
#include <cstring>
#include <unistd.h>

namespace RmdCanSdk {

namespace {

template <typename To, typename From>
To bitCopy(From const& value) {
    static_assert(sizeof(To) == sizeof(From));
    To out;
    std::memcpy(&out, &value, sizeof(out));
    return out;
}

} // namespace

long heimaTimespecToNs(timespec const& value) {
    return value.tv_sec * HeimaNsecPerSec + value.tv_nsec;
}

std::uint16_t heimaSingle2Half(float value) {
    std::uint32_t bits = bitCopy<std::uint32_t>(value);
    std::uint32_t sign = bits & 0x80000000U;
    std::uint32_t exp = bits & 0x7f800000U;
    std::uint32_t man = bits & 0x007fffffU;
    std::uint32_t halfSign = sign >> 16;
    if (exp == 0x7f800000U) {
        std::uint32_t nanBit = man == 0 ? 0 : 0x0200U;
        return static_cast<std::uint16_t>(halfSign | 0x7c00U | nanBit | (man >> 13));
    }
    int halfExp = static_cast<int>(exp >> 23) - 127 + 15;
    if (halfExp >= 0x1f) {
        return static_cast<std::uint16_t>(halfSign | 0x7c00U);
    }
    if (halfExp <= 0) {
        if (14 - halfExp > 24) {
            return static_cast<std::uint16_t>(halfSign);
        }
        man = man | 0x00800000U;
        std::uint32_t halfMan = man >> (14 - halfExp);
        std::uint32_t roundBit = 1U << (13 - halfExp);
        if ((man & roundBit) != 0 && (man & (3 * roundBit - 1)) != 0) {
            halfMan++;
        }
        return static_cast<std::uint16_t>(halfSign | halfMan);
    }
    std::uint32_t halfExpBits = static_cast<std::uint32_t>(halfExp) << 10;
    std::uint32_t halfMan = man >> 13;
    std::uint32_t roundBit = 0x00001000U;
    if ((man & roundBit) != 0 && (man & (3 * roundBit - 1)) != 0) {
        return static_cast<std::uint16_t>((halfSign | halfExpBits | halfMan) + 1);
    }
    return static_cast<std::uint16_t>(halfSign | halfExpBits | halfMan);
}

float heimaHalf2Single(std::uint16_t value) {
    if ((value & 0x7fffU) == 0) {
        std::uint32_t result = static_cast<std::uint32_t>(value) << 16;
        return bitCopy<float>(result);
    }
    std::uint32_t halfSign = value & 0x8000U;
    int halfExp = value & 0x7c00U;
    std::uint32_t halfMan = value & 0x03ffU;
    std::uint32_t sign = halfSign << 16;
    if (halfExp == 0x7c00) {
        std::uint32_t result = sign | 0x7f800000U | (halfMan << 13);
        return bitCopy<float>(result);
    }
    int exp = (halfExp >> 10) - 15 + 127;
    if (halfExp == 0) {
        int e = std::countl_zero(halfMan) - 6;
        exp = (exp - e) << 23;
        std::uint32_t man = (halfMan << (14 + e)) & 0x7fffffU;
        std::uint32_t result = sign | static_cast<std::uint32_t>(exp) | man;
        return bitCopy<float>(result);
    }
    std::uint32_t expBits = static_cast<std::uint32_t>(exp) << 23;
    std::uint32_t result = sign | expBits | (halfMan << 13);
    return bitCopy<float>(result);
}

void heimaAdjustCpu(int& cpu, int processor) {
    long const processorCount = sysconf(_SC_NPROCESSORS_ONLN);
    if (processorCount <= 0) {
        return;
    }
    if (cpu <= 0 || cpu >= processorCount) {
        cpu = static_cast<int>(processorCount) - 1;
        if (cpu > processor && processor > 0) {
            cpu = processor;
        }
    }
}

} // namespace RmdCanSdk
