#include "rmd_can_sdk/heima_driver_sdk.h"
#include "rmd_can_sdk/rmd_bench_workflow.h"
#include "rmd_can_sdk/rmd_can_config.h"

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <cmath>
#include <fstream>
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
using RmdCanSdk::operationFeedbackReady;
using RmdCanSdk::recordConsecutiveReady;
using RmdCanSdk::smoothRampProgressForElapsed;

struct LoopTimingStats {
    int samples = 0;
    int overruns = 0;
    double intervalMinMs = std::numeric_limits<double>::infinity();
    double intervalMaxMs = 0.0;
    double intervalSumMs = 0.0;
    double overrunMaxMs = 0.0;
};

struct AppLoopTimingRow {
    int sample = 0;
    double intervalMs = 0.0;
    double targetPrepMs = 0.0;
    double setTargetMs = 0.0;
    double getActualMs = 0.0;
    double printActualsMs = 0.0;
    double backendStatusMs = 0.0;
    double activeMs = 0.0;
    double overrunMs = 0.0;
    int monitored = 0;
    int backendStatusSampled = 0;
    int actualStatus = -1;
    int feedbackReady = -1;
    int consecutiveMissedFeedback = 0;
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
                  << " tor_abs=" << std::fabs(actual.tor)
                  << " temp=" << actual.temp
                  << " drive_temp=" << actual.driveTemp
                  << " voltage=" << actual.voltage
                  << " status=0x" << std::hex << std::setw(4) << std::setfill('0') << actual.statusWord
                  << " err=0x" << std::setw(4) << actual.errorCode
                  << std::dec << std::setfill(' ');
    }
    std::cout << "\n";
}

void writeBackendStatusHeader(std::ofstream& out) {
    out << "sample,time_ms,backend,running,degraded,error_code,cycle_count,deadline_miss_count,"
           "last_cycle_ns,max_cycle_ns,late_wakeup_count,last_wakeup_latency_ns,max_wakeup_latency_ns,"
           "stale_frame_count,rx_timeout_count,wc_incomplete_count\n";
}

void writeBackendStatusRows(std::ofstream& out,
                            DriverSDK::DriverSDK& sdk,
                            int sample,
                            int periodMs,
                            std::vector<DriverSDK::backendStatusStruct>& statuses) {
    if (!out.is_open() || sdk.getBackendStatus(statuses) != 0) {
        return;
    }
    for (std::size_t backend = 0; backend < statuses.size(); ++backend) {
        auto const& status = statuses[backend];
        out << sample << ',' << static_cast<long long>(sample) * periodMs << ',' << backend << ','
            << status.running << ',' << status.degraded << ',' << status.errorCode << ','
            << status.cycleCount << ',' << status.deadlineMissCount << ','
            << status.lastCycleNs << ',' << status.maxCycleNs << ','
            << status.lateWakeupCount << ',' << status.lastWakeupLatencyNs << ','
            << status.maxWakeupLatencyNs << ',' << status.staleFrameCount << ','
            << status.rxTimeoutCount << ',' << status.wcIncompleteCount << '\n';
    }
}

void printBackendStatusSummary(DriverSDK::DriverSDK& sdk,
                               std::vector<DriverSDK::backendStatusStruct>& statuses) {
    if (sdk.getBackendStatus(statuses) != 0) {
        return;
    }
    for (std::size_t backend = 0; backend < statuses.size(); ++backend) {
        auto const& status = statuses[backend];
        std::cout << "backend_status_summary"
                  << " backend=" << backend
                  << " running=" << status.running
                  << " degraded=" << status.degraded
                  << " error_code=" << status.errorCode
                  << " cycle_count=" << status.cycleCount
                  << " deadline_miss_count=" << status.deadlineMissCount
                  << " last_cycle_ns=" << status.lastCycleNs
                  << " max_cycle_ns=" << status.maxCycleNs
                  << " late_wakeup_count=" << status.lateWakeupCount
                  << " last_wakeup_latency_ns=" << status.lastWakeupLatencyNs
                  << " max_wakeup_latency_ns=" << status.maxWakeupLatencyNs
                  << " stale_frame_count=" << status.staleFrameCount
                  << " rx_timeout_count=" << status.rxTimeoutCount
                  << " wc_incomplete_count=" << status.wcIncompleteCount
                  << "\n";
    }
}

bool writeAppTimingCsv(std::string const& path, std::vector<AppLoopTimingRow> const& rows) {
    if (path.empty()) {
        return true;
    }
    std::ofstream out(path);
    if (!out) {
        std::cerr << "opening app timing CSV failed: " << path << "\n";
        return false;
    }
    out << "sample,interval_ms,target_prep_ms,set_target_ms,get_actual_ms,print_actuals_ms,"
           "backend_status_ms,active_ms,overrun_ms,monitored,backend_status_sampled,"
           "actual_status,feedback_ready,consecutive_missed_feedback\n";
    for (auto const& row : rows) {
        out << row.sample << ',' << row.intervalMs << ',' << row.targetPrepMs << ','
            << row.setTargetMs << ',' << row.getActualMs << ',' << row.printActualsMs << ','
            << row.backendStatusMs << ',' << row.activeMs << ',' << row.overrunMs << ','
            << row.monitored << ',' << row.backendStatusSampled << ',' << row.actualStatus << ','
            << row.feedbackReady << ',' << row.consecutiveMissedFeedback << '\n';
    }
    return true;
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
        std::cerr << "env: RMD_ECAT_RT_CPU/RMD_ECAT_RT_PRIORITY configure the EtherCAT backend; "
                     "RMD_ECAT_APP_CPU/RMD_ECAT_APP_RT_PRIORITY configure this app loop\n";
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
    bool const appRealtimeRequested =
        std::getenv("RMD_ECAT_APP_RT_PRIORITY") != nullptr || std::getenv("RMD_ECAT_APP_CPU") != nullptr;
    RmdCanSdk::RealtimeThreadSettings const appRealtimeSettings =
        RmdCanSdk::parseRealtimeSettings(std::getenv("RMD_ECAT_APP_RT_PRIORITY"),
                                         std::getenv("RMD_ECAT_APP_CPU"),
                                         0,
                                         -1);
    if (!appRealtimeSettings.valid) {
        std::cerr << "invalid app realtime env; RMD_ECAT_APP_RT_PRIORITY and RMD_ECAT_APP_CPU "
                     "must be non-negative integers\n";
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
        if (appRealtimeRequested) {
            RmdCanSdk::applyRealtimeSettings(appRealtimeSettings, "app");
        }

        std::ofstream backendStatusCsv;
        std::vector<DriverSDK::backendStatusStruct> backendStatuses;
        if (char const* statusPath = std::getenv("RMD_ECAT_BACKEND_STATUS_CSV")) {
            backendStatusCsv.open(statusPath);
            if (!backendStatusCsv) {
                std::cerr << "opening backend status CSV failed: " << statusPath << "\n";
                return 1;
            }
            writeBackendStatusHeader(backendStatusCsv);
        }
        std::string appTimingCsvPath;
        if (char const* timingPath = std::getenv("RMD_ECAT_APP_TIMING_CSV")) {
            appTimingCsvPath = timingPath;
        }

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
            bool const readyWindowFull =
                recordConsecutiveReady(feedbackReady(sdk, actuals, actualStatus), consecutiveReady, 10);
            if ((sample % 25) == 0 || readyWindowFull || sample + 1 == settleSamples) {
                printActuals("settle", sample, sdk, actuals);
            }
            if (readyWindowFull) {
                break;
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

        int consecutiveOperationEnabled = 0;
        for (int sample = 0; sample < settleSamples && !gStopRequested; ++sample) {
            if (sdk.setMotorTarget(targets) != 0) {
                std::cerr << "setMotorTarget failed while enabling motors\n";
                disableMotors(sdk, targets);
                return 1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(periodMs));
            int const actualStatus = sdk.getMotorActual(actuals);
            if (operationFeedbackReady(sdk, actuals, actualStatus)) {
                ++consecutiveOperationEnabled;
            } else {
                consecutiveOperationEnabled = 0;
            }
            if ((sample % std::max(1, 100 / periodMs)) == 0 || consecutiveOperationEnabled >= 10 ||
                sample + 1 == settleSamples) {
                printActuals("enable", sample, sdk, actuals);
            }
            if (consecutiveOperationEnabled >= 10) {
                break;
            }
        }
        if (gStopRequested) {
            std::cerr << "stop requested while enabling motors\n";
            disableMotors(sdk, targets);
            return 130;
        }
        if (consecutiveOperationEnabled < 10) {
            std::cerr << "motors did not reach operation enabled; consecutive enabled samples="
                      << consecutiveOperationEnabled << "\n";
            disableMotors(sdk, targets);
            return 1;
        }

        int const iterations = std::max(1, holdMs / periodMs);
        std::vector<AppLoopTimingRow> appTimingRows;
        if (!appTimingCsvPath.empty()) {
            appTimingRows.reserve(static_cast<std::size_t>(iterations));
        }
        int consecutiveMissedFeedback = 0;
        LoopTimingStats timingStats;
        auto nextWake = std::chrono::steady_clock::now();
        auto previousLoopStart = nextWake;
        for (int i = 0; i < iterations && !gStopRequested; ++i) {
            auto const loopStart = std::chrono::steady_clock::now();
            double intervalMs = 0.0;
            if (i > 0) {
                intervalMs = elapsedMs(previousLoopStart, loopStart);
                recordLoopInterval(timingStats, intervalMs);
            }
            previousLoopStart = loopStart;

            auto const targetPrepStart = std::chrono::steady_clock::now();
            float const progress = smoothRampProgressForElapsed(static_cast<double>(i) * periodMs, rampMs);
            for (int active : sdk.getActiveMotors()) {
                std::size_t const index = static_cast<std::size_t>(active);
                targets[index].pos =
                    startPositions[index] + (finalPositions[index] - startPositions[index]) * progress;
            }
            auto const targetPrepEnd = std::chrono::steady_clock::now();
            double const targetPrepMs = elapsedMs(targetPrepStart, targetPrepEnd);

            auto const setTargetStart = std::chrono::steady_clock::now();
            if (sdk.setMotorTarget(targets) != 0) {
                std::cerr << "setMotorTarget failed while holding position targets\n";
                disableMotors(sdk, targets);
                return 1;
            }
            double const setTargetMs = elapsedMs(setTargetStart, std::chrono::steady_clock::now());

            bool monitored = false;
            int actualStatus = -1;
            int feedbackReadyValue = -1;
            double getActualMs = 0.0;
            double printActualsMs = 0.0;
            if ((i % std::max(1, 100 / periodMs)) == 0 || i + 1 == iterations) {
                monitored = true;
                auto const getActualStart = std::chrono::steady_clock::now();
                actualStatus = sdk.getMotorActual(actuals);
                getActualMs = elapsedMs(getActualStart, std::chrono::steady_clock::now());
                bool const ready = operationFeedbackReady(sdk, actuals, actualStatus);
                feedbackReadyValue = ready ? 1 : 0;
                if (!ready) {
                    ++consecutiveMissedFeedback;
                    auto const printStart = std::chrono::steady_clock::now();
                    printActuals("hold", i, sdk, actuals);
                    printActualsMs = elapsedMs(printStart, std::chrono::steady_clock::now());
                    if (consecutiveMissedFeedback >= MaxConsecutiveMissedFeedback) {
                        std::cerr << "operation-enabled feedback lost while holding position targets for "
                                  << consecutiveMissedFeedback << " consecutive monitor samples\n";
                        disableMotors(sdk, targets);
                        return 1;
                    }
                } else {
                    consecutiveMissedFeedback = 0;
                    auto const printStart = std::chrono::steady_clock::now();
                    printActuals("hold", i, sdk, actuals);
                    printActualsMs = elapsedMs(printStart, std::chrono::steady_clock::now());
                }
            }

            bool backendStatusSampled = false;
            double backendStatusMs = 0.0;
            if ((i % std::max(1, 1000 / periodMs)) == 0 || i + 1 == iterations) {
                backendStatusSampled = true;
                auto const backendStatusStart = std::chrono::steady_clock::now();
                writeBackendStatusRows(backendStatusCsv, sdk, i, periodMs, backendStatuses);
                backendStatusMs = elapsedMs(backendStatusStart, std::chrono::steady_clock::now());
            }
            nextWake += std::chrono::milliseconds(periodMs);
            auto const beforeSleep = std::chrono::steady_clock::now();
            double const activeMs = elapsedMs(loopStart, beforeSleep);
            double const overrunMs = std::max(0.0, elapsedMs(nextWake, beforeSleep));
            recordLoopOverrun(timingStats, overrunMs);
            if (!appTimingCsvPath.empty()) {
                appTimingRows.push_back({i,
                                         intervalMs,
                                         targetPrepMs,
                                         setTargetMs,
                                         getActualMs,
                                         printActualsMs,
                                         backendStatusMs,
                                         activeMs,
                                         overrunMs,
                                         monitored ? 1 : 0,
                                         backendStatusSampled ? 1 : 0,
                                         actualStatus,
                                         feedbackReadyValue,
                                         consecutiveMissedFeedback});
            }
            if (beforeSleep < nextWake) {
                std::this_thread::sleep_until(nextWake);
            }
        }

        if (gStopRequested) {
            std::cerr << "stop requested; disabling motors\n";
        }
        printBackendStatusSummary(sdk, backendStatuses);
        printTimingSummary(timingStats, periodMs);
        disableMotors(sdk, targets);
        if (!writeAppTimingCsv(appTimingCsvPath, appTimingRows)) {
            return 1;
        }
        if (!appTimingCsvPath.empty()) {
            std::cout << "app_timing_csv path=" << appTimingCsvPath
                      << " rows=" << appTimingRows.size() << "\n";
        }
    } catch (std::exception const& ex) {
        std::cerr << "position targets failed: " << ex.what() << "\n";
        return 1;
    }

    return 0;
}
