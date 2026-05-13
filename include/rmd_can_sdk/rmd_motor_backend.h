#pragma once

#include <atomic>
#include <cstdint>

namespace RmdCanSdk {

struct BackendStatus {
    bool running = false;
    bool degraded = false;
    int errorCode = 0;
    std::uint64_t cycleCount = 0;
    std::uint64_t deadlineMissCount = 0;
    std::uint64_t lastCycleNs = 0;
    std::uint64_t maxCycleNs = 0;
    std::uint64_t lateWakeupCount = 0;
    std::uint64_t lastWakeupLatencyNs = 0;
    std::uint64_t maxWakeupLatencyNs = 0;
    std::uint64_t staleFrameCount = 0;
    std::uint64_t rxTimeoutCount = 0;
    std::uint64_t wcIncompleteCount = 0;
};

class AtomicBackendStatus {
public:
    void setRunning(bool running) {
        running_.store(running, std::memory_order_release);
    }

    void setFault(int errorCode) {
        degraded_.store(true, std::memory_order_release);
        errorCode_.store(errorCode, std::memory_order_release);
    }

    void recordCycle(std::uint64_t cycleNs, std::uint64_t periodNs, std::uint64_t wakeupLatencyNs = 0) {
        cycleCount_.fetch_add(1, std::memory_order_relaxed);
        lastCycleNs_.store(cycleNs, std::memory_order_relaxed);
        recordMaxCycle(cycleNs);
        lastWakeupLatencyNs_.store(wakeupLatencyNs, std::memory_order_relaxed);
        recordMaxWakeupLatency(wakeupLatencyNs);
        if (wakeupLatencyNs > 0) {
            lateWakeupCount_.fetch_add(1, std::memory_order_relaxed);
        }
        if (periodNs > 0 && cycleNs > periodNs) {
            deadlineMissCount_.fetch_add(1, std::memory_order_relaxed);
        }
    }

    void recordStaleFrame() {
        staleFrameCount_.fetch_add(1, std::memory_order_relaxed);
    }

    void recordRxTimeout() {
        rxTimeoutCount_.fetch_add(1, std::memory_order_relaxed);
    }

    void recordWcIncomplete() {
        wcIncompleteCount_.fetch_add(1, std::memory_order_relaxed);
    }

    BackendStatus snapshot() const {
        BackendStatus out;
        out.running = running_.load(std::memory_order_acquire);
        out.degraded = degraded_.load(std::memory_order_acquire);
        out.errorCode = errorCode_.load(std::memory_order_acquire);
        out.cycleCount = cycleCount_.load(std::memory_order_relaxed);
        out.deadlineMissCount = deadlineMissCount_.load(std::memory_order_relaxed);
        out.lastCycleNs = lastCycleNs_.load(std::memory_order_relaxed);
        out.maxCycleNs = maxCycleNs_.load(std::memory_order_relaxed);
        out.lateWakeupCount = lateWakeupCount_.load(std::memory_order_relaxed);
        out.lastWakeupLatencyNs = lastWakeupLatencyNs_.load(std::memory_order_relaxed);
        out.maxWakeupLatencyNs = maxWakeupLatencyNs_.load(std::memory_order_relaxed);
        out.staleFrameCount = staleFrameCount_.load(std::memory_order_relaxed);
        out.rxTimeoutCount = rxTimeoutCount_.load(std::memory_order_relaxed);
        out.wcIncompleteCount = wcIncompleteCount_.load(std::memory_order_relaxed);
        return out;
    }

private:
    void recordMaxCycle(std::uint64_t cycleNs) {
        std::uint64_t observed = maxCycleNs_.load(std::memory_order_relaxed);
        while (observed < cycleNs &&
               !maxCycleNs_.compare_exchange_weak(observed, cycleNs, std::memory_order_relaxed)) {
        }
    }

    void recordMaxWakeupLatency(std::uint64_t wakeupLatencyNs) {
        std::uint64_t observed = maxWakeupLatencyNs_.load(std::memory_order_relaxed);
        while (observed < wakeupLatencyNs &&
               !maxWakeupLatencyNs_.compare_exchange_weak(observed, wakeupLatencyNs, std::memory_order_relaxed)) {
        }
    }

    std::atomic<bool> running_{false};
    std::atomic<bool> degraded_{false};
    std::atomic<int> errorCode_{0};
    std::atomic<std::uint64_t> cycleCount_{0};
    std::atomic<std::uint64_t> deadlineMissCount_{0};
    std::atomic<std::uint64_t> lastCycleNs_{0};
    std::atomic<std::uint64_t> maxCycleNs_{0};
    std::atomic<std::uint64_t> lateWakeupCount_{0};
    std::atomic<std::uint64_t> lastWakeupLatencyNs_{0};
    std::atomic<std::uint64_t> maxWakeupLatencyNs_{0};
    std::atomic<std::uint64_t> staleFrameCount_{0};
    std::atomic<std::uint64_t> rxTimeoutCount_{0};
    std::atomic<std::uint64_t> wcIncompleteCount_{0};
};

class MotorBackend {
public:
    virtual ~MotorBackend() = default;
    virtual int start() = 0;
    virtual void stop() = 0;
    virtual BackendStatus status() const = 0;
};

} // namespace RmdCanSdk
