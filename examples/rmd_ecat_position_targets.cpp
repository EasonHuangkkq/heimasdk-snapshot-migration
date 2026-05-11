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
#include <limits>
#include <string>
#include <thread>
#include <vector>

namespace {

volatile std::sig_atomic_t gStopRequested = 0;
constexpr int MaxConsecutiveMissedFeedback = 10;
using RmdCanSdk::disableMotors;
using RmdCanSdk::feedbackReady;
using RmdCanSdk::findParamsForAlias;
using RmdCanSdk::isActiveMotor;

struct LoopTimingStats {
    int samples = 0;
    int overruns = 0;
    double intervalMinMs = std::numeric_limits<double>::infinity();
    double intervalMaxMs = 0.0;
    double intervalSumMs = 0.0;
    double overrunMaxMs = 0.0;
};

void requestStop(int) {
    gStopRequested = 1;
}

double elapsedMs(std::chrono::steady_clock::time_point begin,
                 std::chrono::steady_clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

void recordLoopInterval(LoopTimingStats& stats, double intervalMs) {
    ++stats.samples;
    stats.intervalMinMs = std::min(stats.intervalMinMs, intervalMs);
    stats.intervalMaxMs = std::max(stats.intervalMaxMs, intervalMs);
    stats.intervalSumMs += intervalMs;
}

void recordLoopOverrun(LoopTimingStats& stats, double overrunMs) {
    if (overrunMs <= 0.0) {
        return;
    }
    ++stats.overruns;
    stats.overrunMaxMs = std::max(stats.overrunMaxMs, overrunMs);
}

int parsePositive(char const* text, int fallback) {
    if (text == nullptr) {
        return fallback;
    }
    char* end = nullptr;
    long const value = std::strtol(text, &end, 10);
    if (end == text || *end != '\0' || value <= 0 || value > 100000000L) {
        return -1;
    }
    return static_cast<int>(value);
}

int parseNonNegative(char const* text, int fallback) {
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

bool parseFloat(char const* text, float& out) {
    if (text == nullptr || *text == '\0') {
        return false;
    }
    char* end = nullptr;
    float const value = std::strtof(text, &end);
    if (end == text || *end != '\0') {
        return false;
    }
    out = value;
    return true;
}

bool parseTargetAssignment(char const* text, int& motorIndex, float& position) {
    std::string const spec = text == nullptr ? "" : text;
    std::size_t const equals = spec.find('=');
    if (equals == std::string::npos || equals == 0 || equals + 1 >= spec.size()) {
        std::cerr << "target assignment must be motor=position\n";
        return false;
    }

    std::string const motorText = spec.substr(0, equals);
    std::string const positionText = spec.substr(equals + 1);
    char* end = nullptr;
    long const motor = std::strtol(motorText.c_str(), &end, 10);
    if (end == motorText.c_str() || *end != '\0') {
        std::cerr << "motor index must be an integer\n";
        return false;
    }
    if (motor <= 0) {
        std::cerr << "motor index must be positive\n";
        return false;
    }

    float parsedPosition = 0.0f;
    if (!parseFloat(positionText.c_str(), parsedPosition)) {
        std::cerr << "target position must be a number\n";
        return false;
    }
    motorIndex = static_cast<int>(motor) - 1;
    position = parsedPosition;
    return true;
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
                  << " temp=" << actual.temp
                  << " drive_temp=" << actual.driveTemp
                  << " voltage=" << actual.voltage
                  << " status=0x" << std::hex << std::setw(4) << std::setfill('0') << actual.statusWord
                  << " err=0x" << std::setw(4) << actual.errorCode
                  << std::dec << std::setfill(' ');
    }
    std::cout << "\n";
}

void printTimingSummary(LoopTimingStats const& stats, int periodMs) {
    double const averageMs = stats.samples > 0 ? stats.intervalSumMs / stats.samples : 0.0;
    double const minMs = stats.samples > 0 ? stats.intervalMinMs : 0.0;
    std::cout << "timing_summary"
              << " period_ms=" << periodMs
              << " interval_samples=" << stats.samples
              << " interval_min_ms=" << minMs
              << " interval_avg_ms=" << averageMs
              << " interval_max_ms=" << stats.intervalMaxMs
              << " overruns=" << stats.overruns
              << " overrun_max_ms=" << stats.overrunMaxMs
              << "\n";
}

} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);

    if (argc < 8) {
        std::cerr << "usage: " << argv[0]
                  << " <config.xml> <hold_ms> <max_current> <settle_samples> <period_ms> <ramp_ms>"
                  << " <motor=position> [motor=position ...]\n";
        std::cerr << "example: " << argv[0]
                  << " config/configOriginal_heima.xml 30000 500 450 5 8000 3=0.25 4=-0.5\n";
        return 2;
    }

    char const* configPath = argv[1];
    int const holdMs = parsePositive(argv[2], 30000);
    int const maxCurrent = parsePositive(argv[3], 500);
    int const settleSamples = parsePositive(argv[4], 450);
    int const periodMs = parsePositive(argv[5], 5);
    int const rampMs = parseNonNegative(argv[6], 8000);
    if (holdMs <= 0 || maxCurrent <= 0 || maxCurrent > 65535 || settleSamples <= 0 || periodMs <= 0 || rampMs < 0) {
        std::cerr << "invalid numeric argument\n";
        return 2;
    }

    try {
        RmdCanSdk::Config const parsedConfig = RmdCanSdk::loadConfig(configPath);
        int const countFromConfig = parsedConfig.totalMotorCount;
        if (countFromConfig <= 0) {
            std::cerr << "config has no motors\n";
            return 2;
        }

        std::vector<float> requestedTargets(static_cast<std::size_t>(countFromConfig), 0.0f);
        std::vector<bool> hasRequestedTarget(static_cast<std::size_t>(countFromConfig), false);
        for (int arg = 7; arg < argc; ++arg) {
            int motorIndex = -1;
            float position = 0.0f;
            if (!parseTargetAssignment(argv[arg], motorIndex, position)) {
                return 2;
            }
            if (motorIndex >= countFromConfig) {
                std::cerr << "motor index outside configured motor count\n";
                return 2;
            }
            if (hasRequestedTarget[static_cast<std::size_t>(motorIndex)]) {
                std::cerr << "duplicate target for motor " << motorIndex + 1 << "\n";
                return 2;
            }
            RmdCanSdk::MotorParameters const* params = findParamsForAlias(parsedConfig, motorIndex + 1);
            if (params == nullptr) {
                std::cerr << "missing motor parameters for alias " << motorIndex + 1 << "\n";
                return 2;
            }
            if (position < params->minimumPosition || position > params->maximumPosition) {
                std::cerr << "target for motor " << motorIndex + 1
                          << " outside configured position limits ["
                          << params->minimumPosition << ", " << params->maximumPosition << "]\n";
                return 2;
            }
            requestedTargets[static_cast<std::size_t>(motorIndex)] = position;
            hasRequestedTarget[static_cast<std::size_t>(motorIndex)] = true;
        }

        auto& sdk = DriverSDK::DriverSDK::instance();
        std::vector<char> modes(static_cast<std::size_t>(countFromConfig), static_cast<char>(8));
        std::vector<unsigned short> maxCurrents(static_cast<std::size_t>(countFromConfig),
                                                static_cast<unsigned short>(maxCurrent));
        sdk.setMode(modes);
        sdk.setMaxCurr(maxCurrents);
        sdk.init(configPath);

        for (std::size_t i = 0; i < hasRequestedTarget.size(); ++i) {
            if (hasRequestedTarget[i] && !isActiveMotor(sdk, static_cast<int>(i))) {
                std::cerr << "requested motor " << i + 1 << " is not active in this backend\n";
                return 2;
            }
        }

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
            std::cerr << "EtherCAT feedback was not stable enough; consecutive ready samples="
                      << consecutiveReady << "\n";
            return 1;
        }

        for (int active : sdk.getActiveMotors()) {
            std::size_t const index = static_cast<std::size_t>(active);
            startPositions[index] = actuals[index].pos;
            finalPositions[index] = hasRequestedTarget[index] ? requestedTargets[index] : actuals[index].pos;
            targets[index].pos = startPositions[index];
            targets[index].vel = 0.0f;
            targets[index].tor = 0.0f;
            targets[index].kp = 0.0f;
            targets[index].kd = 0.0f;
            targets[index].enabled = 1;
            std::cout << "position target motor " << active + 1
                      << " start=" << startPositions[index]
                      << " final=" << finalPositions[index]
                      << " max_current=" << maxCurrent
                      << " ramp_ms=" << rampMs
                      << (hasRequestedTarget[index] ? " requested" : " hold-current") << "\n";
        }

        int const iterations = std::max(1, holdMs / periodMs);
        int consecutiveMissedFeedback = 0;
        LoopTimingStats timingStats;
        auto nextWake = std::chrono::steady_clock::now();
        auto previousLoopStart = nextWake;
        for (int i = 0; i < iterations && !gStopRequested; ++i) {
            auto const loopStart = std::chrono::steady_clock::now();
            if (i > 0) {
                recordLoopInterval(timingStats, elapsedMs(previousLoopStart, loopStart));
            }
            previousLoopStart = loopStart;

            float const progress = rampMs <= 0 ? 1.0f : std::min(1.0f, static_cast<float>(i * periodMs) / rampMs);
            for (int active : sdk.getActiveMotors()) {
                std::size_t const index = static_cast<std::size_t>(active);
                targets[index].pos =
                    startPositions[index] + (finalPositions[index] - startPositions[index]) * progress;
            }
            if (sdk.setMotorTarget(targets) != 0) {
                std::cerr << "setMotorTarget failed while holding position targets\n";
                disableMotors(sdk, targets);
                return 1;
            }
            if ((i % std::max(1, 100 / periodMs)) == 0 || i + 1 == iterations) {
                int const actualStatus = sdk.getMotorActual(actuals);
                if (!feedbackReady(sdk, actuals, actualStatus)) {
                    printActuals("hold", i, sdk, actuals);
                    ++consecutiveMissedFeedback;
                    if (consecutiveMissedFeedback >= MaxConsecutiveMissedFeedback) {
                        std::cerr << "feedback lost while holding position targets for "
                                  << consecutiveMissedFeedback << " consecutive monitor samples\n";
                        disableMotors(sdk, targets);
                        return 1;
                    }
                } else {
                    consecutiveMissedFeedback = 0;
                    printActuals("hold", i, sdk, actuals);
                }
            }
            nextWake += std::chrono::milliseconds(periodMs);
            auto const beforeSleep = std::chrono::steady_clock::now();
            recordLoopOverrun(timingStats, elapsedMs(nextWake, beforeSleep));
            if (beforeSleep < nextWake) {
                std::this_thread::sleep_until(nextWake);
            }
        }

        if (gStopRequested) {
            std::cerr << "stop requested; disabling motors\n";
        }
        printTimingSummary(timingStats, periodMs);
        disableMotors(sdk, targets);
    } catch (std::exception const& ex) {
        std::cerr << "position targets failed: " << ex.what() << "\n";
        return 1;
    }

    return 0;
}
