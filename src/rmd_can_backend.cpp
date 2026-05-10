#include "rmd_can_sdk/rmd_can_backend.h"

#include "rmd_can_sdk/rmd_protocol.h"
#include "rmd_can_sdk/rmd_safety.h"

#include <algorithm>
#include <chrono>

namespace RmdCanSdk {

RmdCanBackend::RmdCanBackend(Config const& config,
                             MotorRegistry const& registry,
                             std::size_t backendIndex,
                             FrameBuffer<MotorTargetFrame>& targetBuffer,
                             FrameBuffer<MotorActualFrame>& actualBuffer)
    : config_(config),
      registry_(registry),
      backendIndex_(backendIndex),
      targetBuffer_(targetBuffer),
      actualBuffer_(actualBuffer) {
    auto const& group = registry_.backends().at(backendIndex_);
    motorIndexes_ = group.motorIndexes;
    workingActuals_.motorCount = static_cast<std::size_t>(config_.totalMotorCount);

    std::vector<int> slaveIds;
    for (std::size_t registryIndex : motorIndexes_) {
        auto const& motor = registry_.motors().at(registryIndex);
        slaveIds.push_back(motor.slaveId);
        workingActuals_.valid.set(motor.globalIndex);
    }
    mitFrame_.setExpectedFromSlaveIds(slaveIds);
}

RmdCanBackend::~RmdCanBackend() {
    stop();
}

int RmdCanBackend::start() {
    if (status_.snapshot().running) {
        return 0;
    }
    auto const& group = registry_.backends().at(backendIndex_);
    auto masterIt = std::find_if(config_.masters.begin(), config_.masters.end(), [&](MasterConfig const& master) {
        return master.order == group.master;
    });
    if (masterIt == config_.masters.end()) {
        status_.setFault(ErrorCodeBackendFault);
        return -1;
    }
    if (transport_.open(masterIt->device) != 0) {
        status_.setFault(ErrorCodeBackendFault);
        return -1;
    }
    stop_.store(false, std::memory_order_release);
    status_.setRunning(true);
    txThread_ = std::thread(&RmdCanBackend::txLoop, this);
    rxThread_ = std::thread(&RmdCanBackend::rxLoop, this);
    return 0;
}

void RmdCanBackend::stop() {
    stop_.store(true, std::memory_order_release);
    if (txThread_.joinable()) {
        txThread_.join();
    }
    if (rxThread_.joinable()) {
        rxThread_.join();
    }
    transport_.close();
    status_.setRunning(false);
}

BackendStatus RmdCanBackend::status() const {
    return status_.snapshot();
}

void RmdCanBackend::publishActuals() {
    workingActuals_.sequence++;
    workingActuals_.timestamp = RealtimeClock::now();
    actualBuffer_.publish(workingActuals_);
}

void RmdCanBackend::markMissingStale() {
    bool marked = false;
    for (std::size_t registryIndex : motorIndexes_) {
        auto const& motor = registry_.motors().at(registryIndex);
        if (mitFrame_.isReceived(motor.slaveId)) {
            continue;
        }
        markActualStale(workingActuals_.actuals[motor.globalIndex]);
        workingActuals_.stale.set(motor.globalIndex);
        marked = true;
    }
    if (marked) {
        status_.recordStaleFrame();
    }
}

void RmdCanBackend::txLoop() {
    auto const& group = registry_.backends().at(backendIndex_);
    auto masterIt = std::find_if(config_.masters.begin(), config_.masters.end(), [&](MasterConfig const& master) {
        return master.order == group.master;
    });
    std::chrono::nanoseconds const period(config_.periodNs * std::max(1, masterIt->division));
    MotorTargetFrame targets;
    std::uint64_t tick = 0;
    auto nextWakeup = std::chrono::steady_clock::now();
    while (!stop_.load(std::memory_order_acquire)) {
        auto const started = std::chrono::steady_clock::now();
        targetBuffer_.readInto(targets);
        for (std::size_t registryIndex : motorIndexes_) {
            auto const& motor = registry_.motors().at(registryIndex);
            MotorTarget target = targets.targets[motor.globalIndex];
            if (target.enabled != 1) {
                target = MotorTarget{};
            }
            auto limited = limitTarget(target, motor.parameters);
            auto data = packMit(limited.target.pos, limited.target.vel, limited.target.kp, limited.target.kd,
                                limited.target.tor, motor.parameters.maximumTorque);
            if (transport_.send(mitTxId(motor.motorId), data.data(), static_cast<int>(data.size())) < 0) {
                status_.setFault(ErrorCodeBackendFault);
            }
        }
        if (tick % 20 == 0) {
            unsigned char status2[8] = {0x9C, 0, 0, 0, 0, 0, 0, 0};
            for (std::size_t registryIndex : motorIndexes_) {
                auto const& motor = registry_.motors().at(registryIndex);
                transport_.send(standardTxId(motor.motorId), status2, 8);
            }
        }
        tick++;
        auto const finished = std::chrono::steady_clock::now();
        auto const elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(finished - started);
        status_.recordCycle(static_cast<std::uint64_t>(elapsed.count()), static_cast<std::uint64_t>(period.count()));
        nextWakeup += period;
        if (finished > nextWakeup) {
            nextWakeup = finished;
        }
        std::this_thread::sleep_until(nextWakeup);
    }
}

void RmdCanBackend::rxLoop() {
    while (!stop_.load(std::memory_order_acquire)) {
        CanFrame frame;
        int const received = transport_.receive(frame, 50);
        if (received <= 0) {
            status_.recordRxTimeout();
            status_.setFault(ErrorCodeFeedbackTimeout);
            markMissingStale();
            publishActuals();
            mitFrame_.reset();
            continue;
        }

        for (std::size_t registryIndex : motorIndexes_) {
            auto const& motor = registry_.motors().at(registryIndex);
            if (frame.id == mitRxId(motor.motorId)) {
                auto feedback = parseMitReply(frame.data, frame.length, motor.motorId, motor.parameters.maximumTorque);
                if (feedback.valid) {
                    applyMitFeedback(workingActuals_.actuals[motor.globalIndex], feedback);
                    workingActuals_.valid.set(motor.globalIndex);
                    workingActuals_.stale.reset(motor.globalIndex);
                    if (mitFrame_.markReceived(motor.slaveId, FeedbackFrameTracker::Clock::now())) {
                        publishActuals();
                    }
                }
            } else if (frame.id == standardRxId(motor.motorId)) {
                auto feedback = parseStatus2Reply(frame.data, frame.length, motor.parameters.torqueConstant);
                if (feedback.valid) {
                    applyStatus2Feedback(workingActuals_.actuals[motor.globalIndex], feedback);
                }
            }
        }
    }
}

} // namespace RmdCanSdk
