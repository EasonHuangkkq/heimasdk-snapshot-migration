#pragma once

#include "rmd_can_sdk/rmd_can_config.h"
#include "rmd_can_sdk/rmd_can_transport.h"
#include "rmd_can_sdk/rmd_motor_backend.h"
#include "rmd_can_sdk/rmd_motor_registry.h"
#include "rmd_can_sdk/rmd_realtime_core.h"

#include <atomic>
#include <cstddef>
#include <thread>
#include <vector>

namespace RmdCanSdk {

class RmdCanBackend final : public MotorBackend {
public:
    RmdCanBackend(Config const& config,
                  MotorRegistry const& registry,
                  std::size_t backendIndex,
                  FrameBuffer<MotorTargetFrame>& targetBuffer,
                  FrameBuffer<MotorActualFrame>& actualBuffer);
    ~RmdCanBackend() override;

    int start() override;
    void stop() override;
    BackendStatus status() const override;

private:
    void txLoop();
    void rxLoop();
    void publishActuals();
    void markMissingStale();

    Config const& config_;
    MotorRegistry const& registry_;
    std::size_t backendIndex_ = 0;
    FrameBuffer<MotorTargetFrame>& targetBuffer_;
    FrameBuffer<MotorActualFrame>& actualBuffer_;
    AtomicBackendStatus status_;
    SocketCanTransport transport_;
    FeedbackFrameTracker mitFrame_;
    std::vector<std::size_t> motorIndexes_;
    MotorActualFrame workingActuals_;
    std::atomic<bool> stop_{true};
    std::thread txThread_;
    std::thread rxThread_;
};

} // namespace RmdCanSdk
