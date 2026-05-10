#pragma once

#include "rmd_can_sdk/rmd_motor_backend.h"
#include "rmd_can_sdk/rmd_motor_registry.h"
#include "rmd_can_sdk/rmd_ethercat_target_frame.h"
#include "rmd_can_sdk/rmd_realtime_core.h"

#include <vector>

namespace RmdCanSdk {

class RmdEthercatRuntime;

class RmdEthercatBackend final : public MotorBackend {
public:
    RmdEthercatBackend(Config const& config,
                       MotorRegistry const& registry,
                       std::size_t backendIndex,
                       FrameBuffer<EthercatPackedTargetFrame>& targetBuffer,
                       FrameBuffer<MotorActualFrame>& actualBuffer,
                       std::vector<char> operatingModes);
    ~RmdEthercatBackend() override;

    int start() override;
    void stop() override;
    BackendStatus status() const override;

private:
    RmdEthercatRuntime* runtime_ = nullptr;
    AtomicBackendStatus status_;
};

} // namespace RmdCanSdk
