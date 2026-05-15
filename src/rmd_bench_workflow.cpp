#include "rmd_can_sdk/rmd_bench_workflow.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <sched.h>
#include <string>
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

bool operationFeedbackReady(std::vector<int> const& activeMotors,
                            std::vector<DriverSDK::motorActualStruct> const& actuals,
                            int status) {
    if (status != 0) {
        return false;
    }
    for (int active : activeMotors) {
        auto const index = static_cast<std::size_t>(active);
        if (index >= actuals.size()) {
            return false;
        }
        auto const& actual = actuals[index];
        if (actual.statusWord == 0xffff || actual.errorCode != 0 || !operationEnabled(actual.statusWord)) {
            return false;
        }
    }
    return true;
}

bool operationFeedbackReady(DriverSDK::DriverSDK& sdk,
                            std::vector<DriverSDK::motorActualStruct> const& actuals,
                            int status) {
    return operationFeedbackReady(sdk.getActiveMotors(), actuals, status);
}

int parseNonNegativeInt(char const* text, int fallback) {
    if (text == nullptr) {
        return fallback;
    }
    char* end = nullptr;
    long const value = std::strtol(text, &end, 10);
    if (end == text || *end != '\0' || value < 0 || value > 100000000L) {
        return -1;
    }
    return static_cast<int>(value);
}

float smoothRampProgress(float linearProgress) {
    double const t = std::clamp(static_cast<double>(linearProgress), 0.0, 1.0);
    return static_cast<float>(t * t * t * (10.0 + t * (-15.0 + 6.0 * t)));
}

float smoothRampProgressForElapsed(double elapsedMs, double rampMs) {
    if (rampMs <= 0.0) {
        return 1.0f;
    }
    return smoothRampProgress(static_cast<float>(elapsedMs / rampMs));
}

bool recordConsecutiveReady(bool ready, int& consecutiveReady, int requiredReadySamples) {
    if (!ready) {
        consecutiveReady = 0;
        return false;
    }
    ++consecutiveReady;
    return consecutiveReady >= requiredReadySamples;
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

RealtimeThreadSettings parseRealtimeSettings(char const* priorityText,
                                             char const* cpuText,
                                             int defaultPriority,
                                             int defaultCpu) {
    RealtimeThreadSettings settings;
    settings.priority = parseNonNegativeInt(priorityText, defaultPriority);
    settings.cpu = parseNonNegativeInt(cpuText, defaultCpu);
    bool const priorityValid = settings.priority >= 0;
    bool const cpuValid = cpuText == nullptr ? settings.cpu >= -1 : settings.cpu >= 0;
    settings.valid = priorityValid && cpuValid;
    return settings;
}

void applyRealtimeSettings(RealtimeThreadSettings const& settings, char const* label) {
    std::string const prefix = (label != nullptr && *label != '\0') ? std::string(label) + "_" : "";
    if (!settings.valid) {
        std::cerr << prefix << "realtime_settings=invalid\n";
        return;
    }

    bool affinitySet = false;
    if (settings.cpu >= 0) {
        cpu_set_t set;
        CPU_ZERO(&set);
        CPU_SET(settings.cpu, &set);
        affinitySet = (sched_setaffinity(0, sizeof(set), &set) == 0);
        if (!affinitySet) {
            std::cerr << "warning: setting " << prefix << "CPU affinity to CPU " << settings.cpu
                      << " failed: " << std::strerror(errno) << "\n";
        }
    }

    bool memoryLocked = false;
    if (settings.priority > 0) {
        memoryLocked = (mlockall(MCL_CURRENT | MCL_FUTURE) == 0);
        if (!memoryLocked) {
            std::cerr << "warning: mlockall failed: " << std::strerror(errno) << "\n";
        }
    }

    bool schedulerSet = false;
    if (settings.priority > 0) {
        sched_param param{};
        param.sched_priority = settings.priority;
        schedulerSet = (sched_setscheduler(0, SCHED_FIFO, &param) == 0);
        if (!schedulerSet) {
            std::cerr << "warning: sched_setscheduler SCHED_FIFO failed: " << std::strerror(errno) << "\n";
        }
    }

    std::cout << prefix << "rt_priority=" << settings.priority
              << " realtime_scheduler="
              << (settings.priority > 0 ? (schedulerSet ? "SCHED_FIFO" : "failed") : "disabled")
              << " cpu=" << settings.cpu
              << " affinity=" << (settings.cpu >= 0 ? (affinitySet ? "set" : "failed") : "disabled")
              << " memory_lock="
              << (settings.priority > 0 ? (memoryLocked ? "locked" : "failed") : "disabled") << "\n";
}

void applyRealtimeSettings(int rtPriority) {
    applyRealtimeSettings(RealtimeThreadSettings{rtPriority, -1, rtPriority >= 0}, nullptr);
}

} // namespace RmdCanSdk
