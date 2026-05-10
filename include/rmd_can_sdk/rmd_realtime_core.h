#pragma once

#include "rmd_can_sdk/rmd_types.h"

#include <array>
#include <atomic>
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
class FrameBuffer {
public:
    explicit FrameBuffer(Frame initial = Frame{}) {
        buffers_[0] = initial;
        buffers_[1] = initial;
        buffers_[2] = initial;
    }

    void publish(Frame const& frame) {
        buffers_[writing_] = frame;
        std::uint64_t const nextVersion = writeVersion_ + 1;
        std::uint64_t const previous = latest_.exchange(packState(nextVersion, writing_), std::memory_order_acq_rel);
        writeVersion_ = nextVersion;
        writing_ = stateIndex(previous);
    }

    Frame read() const {
        Frame out;
        readInto(out);
        return out;
    }

    void readInto(Frame& out) const {
        for (int attempt = 0; attempt < 3; ++attempt) {
            std::uint64_t observed = latest_.load(std::memory_order_acquire);
            std::uint64_t const version = stateVersion(observed);
            if (version == readVersion_) {
                break;
            }
            std::uint64_t const replacement = packState(version, reading_);
            if (latest_.compare_exchange_weak(
                    observed, replacement, std::memory_order_acq_rel, std::memory_order_acquire)) {
                reading_ = stateIndex(observed);
                readVersion_ = version;
                break;
            }
        }
        out = buffers_[reading_];
    }

private:
    static constexpr std::uint64_t packState(std::uint64_t version, int index) {
        return (version << 2) | static_cast<std::uint64_t>(index & 0x3);
    }

    static constexpr int stateIndex(std::uint64_t state) {
        return static_cast<int>(state & 0x3);
    }

    static constexpr std::uint64_t stateVersion(std::uint64_t state) {
        return state >> 2;
    }

    std::array<Frame, 3> buffers_{};
    mutable std::atomic<std::uint64_t> latest_{packState(0, 0)};
    int writing_ = 1;
    std::uint64_t writeVersion_ = 0;
    mutable int reading_ = 2;
    mutable std::uint64_t readVersion_ = 0;
};

} // namespace RmdCanSdk
