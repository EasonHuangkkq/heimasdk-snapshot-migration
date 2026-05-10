#pragma once

#include <cstdint>
#include <ctime>

namespace RmdCanSdk {

constexpr long HeimaNsecPerSec = 1000000000L;

long heimaTimespecToNs(timespec const& value);

std::uint16_t heimaSingle2Half(float value);
float heimaHalf2Single(std::uint16_t value);
void heimaAdjustCpu(int& cpu, int processor);

struct HeimaSdoMsg {
    void* sdoHandler = nullptr;
    long value = 0;
    int alias = 0;
    short state = 0;
    std::uint16_t index = 0;
    std::uint8_t subindex = 0;
    std::uint8_t signed_ = 0;
    std::uint8_t bitLength = 0;
    std::uint8_t operation = 0;
    int recycled = 0;
};

struct HeimaRegMsg {
    void* regHandler = nullptr;
    long value = 0;
    int alias = 0;
    int recycled = 0;
};

#pragma pack(push, 1)
struct HeimaStandardRxData {
    std::uint16_t ControlWord = 0;
    std::int32_t TargetPosition = 0;
    std::int32_t TargetVelocity = 0;
    std::int16_t TargetTorque = 0;
    std::uint16_t MaxTorque = 0;
    char Mode = 0;
    std::int8_t Undefined = 0;
};

struct HeimaPvtRxData {
    std::uint16_t ControlWord = 0;
    std::int32_t TargetPosition = 0;
    std::int32_t TargetVelocity = 0;
    std::int16_t TargetTorque = 0;
    std::int32_t PvtKp = 0;
    std::int32_t PvtKd = 0;
    char Mode = 0;
    std::int8_t Undefined = 0;
};

struct HeimaStandardTxData {
    std::uint16_t StatusWord = 0;
    std::int32_t ActualPosition = 0;
    std::int32_t ActualVelocity = 0;
    std::int16_t ActualTorque = 0;
    std::uint16_t ErrorCode = 0;
    char ModeDisplay = 0;
    std::int8_t Undefined = 0;
};

struct HeimaDriverTxData {
    std::uint16_t StatusWord = 0;
    std::int32_t ActualPosition = 0;
    std::int32_t ActualVelocity = 0;
    std::int16_t ActualTorque = 0;
    std::uint16_t ErrorCode = 0;
    std::int16_t MotorTemperature = 0;
    std::int16_t DriveTemperature = 0;
    std::uint16_t Voltage = 0;
    char ModeDisplay = 0;
    std::int8_t Undefined = 0;
};
#pragma pack(pop)

using HeimaDriverRxData = HeimaPvtRxData;

static_assert(sizeof(HeimaStandardRxData) == 16);
static_assert(sizeof(HeimaPvtRxData) == 22);
static_assert(sizeof(HeimaStandardTxData) == 16);
static_assert(sizeof(HeimaDriverTxData) == 22);

} // namespace RmdCanSdk
