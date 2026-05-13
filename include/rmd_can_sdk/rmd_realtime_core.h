#pragma once

#include "rmd_can_sdk/rmd_snapshot_buffer.h"
#include "rmd_can_sdk/rmd_types.h"

#include <array>
#include <bitset>
#include <chrono>
#include <cstddef>
#include <cstdint>

namespace RmdCanSdk {

constexpr std::size_t MaxRealtimeMotors = 64;

using RealtimeClock = std::chrono::steady_clock;

struct FrameMetadata {
    std::uint64_t sequence = 0;
    RealtimeClock::time_point timestamp{};
    std::size_t motorCount = 0;
};

struct MotorTargetFrame : FrameMetadata {
    std::array<MotorTarget, MaxRealtimeMotors> targets{};
    std::bitset<MaxRealtimeMotors> valid{};
};

struct MotorActualFrame : FrameMetadata {
    std::array<MotorActual, MaxRealtimeMotors> actuals{};
    std::bitset<MaxRealtimeMotors> valid{};
    std::bitset<MaxRealtimeMotors> stale{};
};

template <typename Frame>
using FrameBuffer = SnapshotBuffer<Frame>;

} // namespace RmdCanSdk
