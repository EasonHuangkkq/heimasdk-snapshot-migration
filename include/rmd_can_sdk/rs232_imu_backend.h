#pragma once

#include "rmd_can_sdk/heima_driver_sdk.h"
#include "rmd_can_sdk/rmd_realtime_core.h"

#include <atomic>
#include <string>
#include <thread>

namespace RmdCanSdk {

using ImuSnapshotBuffer = FrameBuffer<DriverSDK::imuStruct>;

bool isSupportedImuType(char const* type);

class Rs232ImuBackend {
public:
    explicit Rs232ImuBackend(ImuSnapshotBuffer& buffer);
    ~Rs232ImuBackend();

    int start(char const* device, int baudrate);
    int start(char const* device, int baudrate, char const* type);
    void stop();
    bool running() const { return running_.load(std::memory_order_acquire); }

private:
    void readLoop();

    ImuSnapshotBuffer& buffer_;
    std::string device_;
    std::string type_;
    int baudrate_ = 0;
    int serialFd_ = -1;
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_{true};
    std::thread thread_;
};

} // namespace RmdCanSdk
