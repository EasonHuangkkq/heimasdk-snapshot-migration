#include "rmd_can_sdk/rmd_ethercat_backend.h"

#include "rmd_can_sdk/rmd_safety.h"

namespace RmdCanSdk {

RmdEthercatBackend::RmdEthercatBackend(Config const&,
                                       MotorRegistry const&,
                                       std::size_t,
                                       FrameBuffer<EthercatPackedTargetFrame>&,
                                       FrameBuffer<MotorActualFrame>&,
                                       std::vector<char>) {}

RmdEthercatBackend::~RmdEthercatBackend() = default;

int RmdEthercatBackend::start() {
    status_.setRunning(false);
    status_.setFault(ErrorCodeBackendFault);
    return -1;
}

void RmdEthercatBackend::stop() {
    status_.setRunning(false);
}

BackendStatus RmdEthercatBackend::status() const {
    return status_.snapshot();
}

} // namespace RmdCanSdk
