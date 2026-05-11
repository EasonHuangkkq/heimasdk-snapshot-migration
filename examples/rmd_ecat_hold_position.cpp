#include "rmd_can_sdk/heima_driver_sdk.h"
#include "rmd_can_sdk/rmd_bench_workflow.h"
#include "rmd_can_sdk/rmd_can_config.h"

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

namespace {

volatile std::sig_atomic_t gStopRequested = 0;
using RmdCanSdk::disableMotors;
using RmdCanSdk::feedbackReady;

void requestStop(int) {
    gStopRequested = 1;
}

int parsePositive(char const* text, int fallback) {
    if (text == nullptr) {
        return fallback;
    }
    char* end = nullptr;
    long const value = std::strtol(text, &end, 10);
    if (end == text || value <= 0 || value > 100000000L) {
        return -1;
    }
    return static_cast<int>(value);
}

void printActuals(char const* prefix,
                  int sample,
                  DriverSDK::DriverSDK& sdk,
                  std::vector<DriverSDK::motorActualStruct> const& actuals) {
    std::cout << prefix << " " << sample;
    for (int active : sdk.getActiveMotors()) {
        auto const& actual = actuals[static_cast<std::size_t>(active)];
        std::cout << " | motor " << active + 1
                  << " pos=" << actual.pos
                  << " vel=" << actual.vel
                  << " tor=" << actual.tor
                  << " status=0x" << std::hex << std::setw(4) << std::setfill('0') << actual.statusWord
                  << " err=0x" << std::setw(4) << actual.errorCode
                  << std::dec << std::setfill(' ');
    }
    std::cout << "\n";
}

} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);

    if (argc < 2 || argc > 8) {
        std::cerr << "usage: " << argv[0]
                  << " <config.xml> [hold_ms=10000] [max_current=500] [settle_samples=450] [period_ms=5]"
                  << " [target=current|zero] [ramp_ms=0]\n";
        return 2;
    }

    char const* config = argv[1];
    int const holdMs = argc > 2 ? parsePositive(argv[2], 10000) : 10000;
    int const maxCurrent = argc > 3 ? parsePositive(argv[3], 500) : 500;
    int const settleSamples = argc > 4 ? parsePositive(argv[4], 450) : 450;
    int const periodMs = argc > 5 ? parsePositive(argv[5], 5) : 5;
    std::string const targetMode = argc > 6 ? argv[6] : "current";
    int const rampMs = argc > 7 ? parsePositive(argv[7], 0) : 0;
    if (holdMs <= 0 || maxCurrent <= 0 || maxCurrent > 65535 || settleSamples <= 0 || periodMs <= 0) {
        std::cerr << "invalid numeric argument\n";
        return 2;
    }
    if (targetMode != "current" && targetMode != "zero") {
        std::cerr << "target must be current or zero\n";
        return 2;
    }
    if (rampMs < 0) {
        std::cerr << "ramp_ms must be non-negative\n";
        return 2;
    }

    try {
        int const countFromConfig = RmdCanSdk::loadConfig(config).totalMotorCount;
        auto& sdk = DriverSDK::DriverSDK::instance();
        std::vector<char> modes(static_cast<std::size_t>(countFromConfig), static_cast<char>(8));
        std::vector<unsigned short> maxCurrents(static_cast<std::size_t>(countFromConfig),
                                                static_cast<unsigned short>(maxCurrent));
        sdk.setMode(modes);
        sdk.setMaxCurr(maxCurrents);
        sdk.init(config);

        int const count = sdk.getTotalMotorNr();
        std::vector<DriverSDK::motorActualStruct> actuals(static_cast<std::size_t>(count));
        std::vector<DriverSDK::motorTargetStruct> targets(static_cast<std::size_t>(count));
        std::vector<float> startPositions(static_cast<std::size_t>(count), 0.0f);
        std::vector<float> finalPositions(static_cast<std::size_t>(count), 0.0f);

        int consecutiveReady = 0;
        for (int sample = 0; sample < settleSamples && !gStopRequested; ++sample) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            int const actualStatus = sdk.getMotorActual(actuals);
            if (feedbackReady(sdk, actuals, actualStatus)) {
                ++consecutiveReady;
            } else {
                consecutiveReady = 0;
            }
            if ((sample % 25) == 0 || sample + 1 == settleSamples) {
                printActuals("settle", sample, sdk, actuals);
            }
        }
        if (gStopRequested) {
            std::cerr << "stop requested before enabling motors\n";
            return 130;
        }
        if (consecutiveReady < 10) {
            std::cerr << "EtherCAT feedback was not stable enough to capture hold position; consecutive ready samples="
                      << consecutiveReady << "\n";
            return 1;
        }

        for (int active : sdk.getActiveMotors()) {
            std::size_t const index = static_cast<std::size_t>(active);
            startPositions[index] = actuals[index].pos;
            finalPositions[index] = targetMode == "zero" ? 0.0f : actuals[index].pos;
            targets[index].pos = startPositions[index];
            targets[index].vel = 0.0f;
            targets[index].tor = 0.0f;
            targets[index].kp = 0.0f;
            targets[index].kd = 0.0f;
            targets[index].enabled = 1;
            std::cout << "hold target motor " << active + 1
                      << " start=" << startPositions[index]
                      << " final=" << finalPositions[index]
                      << " max_current=" << maxCurrent
                      << " ramp_ms=" << rampMs << "\n";
        }

        int const iterations = std::max(1, holdMs / periodMs);
        for (int i = 0; i < iterations && !gStopRequested; ++i) {
            float const progress = rampMs <= 0 ? 1.0f : std::min(1.0f, static_cast<float>(i * periodMs) / rampMs);
            for (int active : sdk.getActiveMotors()) {
                std::size_t const index = static_cast<std::size_t>(active);
                targets[index].pos =
                    startPositions[index] + (finalPositions[index] - startPositions[index]) * progress;
            }
            if (sdk.setMotorTarget(targets) != 0) {
                std::cerr << "setMotorTarget failed while holding position\n";
                disableMotors(sdk, targets);
                return 1;
            }
            if ((i % std::max(1, 100 / periodMs)) == 0 || i + 1 == iterations) {
                int const actualStatus = sdk.getMotorActual(actuals);
                if (!feedbackReady(sdk, actuals, actualStatus)) {
                    std::cerr << "feedback lost while holding position\n";
                    printActuals("hold", i, sdk, actuals);
                    disableMotors(sdk, targets);
                    return 1;
                }
                printActuals("hold", i, sdk, actuals);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(periodMs));
        }

        if (gStopRequested) {
            std::cerr << "stop requested; disabling motors\n";
        }
        disableMotors(sdk, targets);
    } catch (std::exception const& ex) {
        std::cerr << "hold position failed: " << ex.what() << "\n";
        return 1;
    }

    return 0;
}
