#include "rmd_can_sdk/rmd_bench_workflow.h"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <sched.h>
#include <sys/mman.h>
#include <thread>

namespace RmdCanSdk {

MotorParameters const* findParamsForAlias(Config const& config, int alias) {
    for (auto const& motor : config.motors) {
        if (motor.alias == alias) {
            return &motor.parameters;
        }
    }
    return nullptr;
}

bool isActiveMotor(DriverSDK::DriverSDK& sdk, int motorIndex) {
    for (int active : sdk.getActiveMotors()) {
        if (active == motorIndex) {
            return true;
        }
    }
    return false;
}

bool operationEnabled(unsigned short statusWord) {
    return (statusWord & 0x007fU) == 0x0037U;
}

bool feedbackReady(DriverSDK::DriverSDK& sdk,
                   std::vector<DriverSDK::motorActualStruct> const& actuals,
                   int status) {
    if (status != 0) {
        return false;
    }
    for (int active : sdk.getActiveMotors()) {
        auto const index = static_cast<std::size_t>(active);
        if (index >= actuals.size()) {
            return false;
        }
        auto const& actual = actuals[index];
        if (actual.statusWord == 0xffff || actual.errorCode != 0) {
            return false;
        }
    }
    return true;
}

void disableMotors(DriverSDK::DriverSDK& sdk,
                   std::vector<DriverSDK::motorTargetStruct>& targets,
                   int repeatCount,
                   int periodMs) {
    for (auto& target : targets) {
        target.enabled = 0;
        target.vel = 0.0f;
        target.tor = 0.0f;
    }
    int const repeats = repeatCount > 0 ? repeatCount : 1;
    int const sleepMs = periodMs > 0 ? periodMs : 1;
    for (int i = 0; i < repeats; ++i) {
        sdk.setMotorTarget(targets);
        std::this_thread::sleep_for(std::chrono::milliseconds(sleepMs));
    }
}

void applyRealtimeSettings(int rtPriority) {
    if (rtPriority <= 0) {
        std::cout << "rt_priority=0 realtime_scheduler=disabled memory_lock=disabled\n";
        return;
    }

    bool const memoryLocked = (mlockall(MCL_CURRENT | MCL_FUTURE) == 0);
    if (!memoryLocked) {
        std::cerr << "warning: mlockall failed: " << std::strerror(errno) << "\n";
    }

    sched_param param{};
    param.sched_priority = rtPriority;
    bool const schedulerSet = (sched_setscheduler(0, SCHED_FIFO, &param) == 0);
    if (!schedulerSet) {
        std::cerr << "warning: sched_setscheduler SCHED_FIFO failed: " << std::strerror(errno) << "\n";
    }

    std::cout << "rt_priority=" << rtPriority
              << " realtime_scheduler=" << (schedulerSet ? "SCHED_FIFO" : "failed")
              << " memory_lock=" << (memoryLocked ? "locked" : "failed") << "\n";
}

} // namespace RmdCanSdk
