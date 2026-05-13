#include "rmd_can_sdk/heima_driver_sdk.h"
#include "rmd_can_sdk/rmd_bench_workflow.h"
#include "rmd_can_sdk/rmd_can_config.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <vector>

namespace {

volatile std::sig_atomic_t gStopRequested = 0;
using RmdCanSdk::applyRealtimeSettings;
using RmdCanSdk::disableMotors;
using RmdCanSdk::feedbackReady;
using RmdCanSdk::findParamsForAlias;
using RmdCanSdk::isActiveMotor;
using RmdCanSdk::operationEnabled;
using RmdCanSdk::operationFeedbackReady;

struct Sample {
    int index = 0;
    double elapsedMs = 0.0;
    double intervalMs = 0.0;
    double overrunMs = 0.0;
    float target = 0.0f;
    float pos = 0.0f;
    float vel = 0.0f;
    float tor = 0.0f;
    short temp = 0;
    short driveTemp = 0;
    unsigned short voltage = 0;
    unsigned short status = 0xffff;
    unsigned short error = 0;
    int actualStatus = 0;
};

struct TimingStats {
    int intervals = 0;
    int overruns = 0;
    double intervalMinMs = std::numeric_limits<double>::infinity();
    double intervalMaxMs = 0.0;
    double intervalSumMs = 0.0;
    double overrunMaxMs = 0.0;
};

struct ResponseMetrics {
    bool moved = false;
    double responseStartMs = -1.0;
    double t10Ms = -1.0;
    double t50Ms = -1.0;
    double t90Ms = -1.0;
    double settlingMs = -1.0;
    float overshoot = 0.0f;
    float maxAbsError = 0.0f;
};

void requestStop(int) {
    gStopRequested = 1;
}

double elapsedMs(std::chrono::steady_clock::time_point begin,
                 std::chrono::steady_clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

int parseInt(char const* text, char const* name) {
    if (text == nullptr || *text == '\0') {
        std::cerr << name << " must be an integer\n";
        return -1;
    }
    char* end = nullptr;
    long const value = std::strtol(text, &end, 10);
    if (end == text || *end != '\0' || value < 0 || value > 100000000L) {
        std::cerr << name << " must be an integer\n";
        return -1;
    }
    return static_cast<int>(value);
}

bool parseFloatArg(char const* text, char const* name, float& out) {
    if (text == nullptr || *text == '\0') {
        std::cerr << name << " must be a number\n";
        return false;
    }
    char* end = nullptr;
    float const value = std::strtof(text, &end);
    if (end == text || *end != '\0' || !std::isfinite(value)) {
        std::cerr << name << " must be a number\n";
        return false;
    }
    out = value;
    return true;
}

void updateTiming(TimingStats& stats, double intervalMs, double overrunMs) {
    ++stats.intervals;
    stats.intervalMinMs = std::min(stats.intervalMinMs, intervalMs);
    stats.intervalMaxMs = std::max(stats.intervalMaxMs, intervalMs);
    stats.intervalSumMs += intervalMs;
    if (overrunMs > 0.0) {
        ++stats.overruns;
        stats.overrunMaxMs = std::max(stats.overrunMaxMs, overrunMs);
    }
}

ResponseMetrics computeMetrics(std::vector<Sample> const& samples,
                               double stepCommandMs,
                               float startPosition,
                               float finalPosition) {
    ResponseMetrics metrics;
    float const step = finalPosition - startPosition;
    float const absStep = std::fabs(step);
    if (samples.empty() || absStep <= 0.0f) {
        return metrics;
    }

    float const sign = step >= 0.0f ? 1.0f : -1.0f;
    float const responseThreshold = std::max(0.001f, absStep * 0.02f);
    float const settleBand = std::max(0.001f, absStep * 0.02f);
    float maxTravel = 0.0f;
    metrics.maxAbsError = 0.0f;

    for (auto const& sample : samples) {
        if (sample.elapsedMs < stepCommandMs) {
            continue;
        }
        float const travel = sign * (sample.pos - startPosition);
        maxTravel = std::max(maxTravel, travel);
        metrics.maxAbsError = std::max(metrics.maxAbsError, std::fabs(finalPosition - sample.pos));
        if (!metrics.moved && travel >= responseThreshold) {
            metrics.moved = true;
            metrics.responseStartMs = sample.elapsedMs - stepCommandMs;
        }
        if (metrics.t10Ms < 0.0 && travel >= absStep * 0.10f) {
            metrics.t10Ms = sample.elapsedMs - stepCommandMs;
        }
        if (metrics.t50Ms < 0.0 && travel >= absStep * 0.50f) {
            metrics.t50Ms = sample.elapsedMs - stepCommandMs;
        }
        if (metrics.t90Ms < 0.0 && travel >= absStep * 0.90f) {
            metrics.t90Ms = sample.elapsedMs - stepCommandMs;
        }
    }

    metrics.overshoot = std::max(0.0f, maxTravel - absStep);
    for (std::size_t i = 0; i < samples.size(); ++i) {
        if (samples[i].elapsedMs < stepCommandMs) {
            continue;
        }
        bool settled = true;
        for (std::size_t j = i; j < samples.size(); ++j) {
            if (std::fabs(samples[j].pos - finalPosition) > settleBand) {
                settled = false;
                break;
            }
        }
        if (settled) {
            metrics.settlingMs = samples[i].elapsedMs - stepCommandMs;
            break;
        }
    }
    return metrics;
}

bool writeCsv(std::string const& path, std::vector<Sample> const& samples) {
    std::ofstream out(path);
    if (!out) {
        std::cerr << "failed to open CSV output: " << path << "\n";
        return false;
    }

    out << "index,elapsed_ms,interval_ms,overrun_ms,target,pos,vel,tor,temp,drive_temp,voltage,status,error,actual_status\n";
    out << std::fixed << std::setprecision(6);
    for (auto const& sample : samples) {
        out << sample.index << ','
            << sample.elapsedMs << ','
            << sample.intervalMs << ','
            << sample.overrunMs << ','
            << sample.target << ','
            << sample.pos << ','
            << sample.vel << ','
            << sample.tor << ','
            << sample.temp << ','
            << sample.driveTemp << ','
            << sample.voltage << ','
            << sample.status << ','
            << sample.error << ','
            << sample.actualStatus << '\n';
    }
    return true;
}

} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);

    if (argc != 11 && argc != 15) {
        std::cerr << "usage: " << argv[0]
                  << " <config.xml> <motor> <step_rad> <baseline_ms> <response_ms>"
                  << " <max_current> <settle_samples> <period_us> <csv_path> <rt_priority>"
                  << " [mode kp kd ff_torque]\n";
        return 2;
    }

    char const* configPath = argv[1];
    int const motorOneBased = parseInt(argv[2], "motor");
    float stepRad = 0.0f;
    if (!parseFloatArg(argv[3], "step_rad", stepRad)) {
        return 2;
    }
    int const baselineMs = parseInt(argv[4], "baseline_ms");
    int const responseMs = parseInt(argv[5], "response_ms");
    int const maxCurrent = parseInt(argv[6], "max_current");
    int const settleSamples = parseInt(argv[7], "settle_samples");
    int const periodUs = parseInt(argv[8], "period_us");
    std::string const csvPath = argv[9];
    int const rtPriority = parseInt(argv[10], "rt_priority");
    int const mode = argc == 15 ? parseInt(argv[11], "mode") : 8;
    float pvtKp = 0.0f;
    float pvtKd = 0.0f;
    float feedforwardTorque = 0.0f;
    if (argc == 15) {
        if (!parseFloatArg(argv[12], "kp", pvtKp) ||
            !parseFloatArg(argv[13], "kd", pvtKd) ||
            !parseFloatArg(argv[14], "ff_torque", feedforwardTorque)) {
            return 2;
        }
    }

    if (motorOneBased <= 0) {
        std::cerr << "motor must be positive\n";
        return 2;
    }
    if (mode < -128 || mode > 127) {
        std::cerr << "mode must fit int8\n";
        return 2;
    }
    if ((pvtKp < 0.0f || pvtKd < 0.0f) || !std::isfinite(pvtKp) || !std::isfinite(pvtKd)) {
        std::cerr << "kp and kd must be finite non-negative numbers\n";
        return 2;
    }
    if (!std::isfinite(feedforwardTorque) || std::fabs(feedforwardTorque) > 50.0f) {
        std::cerr << "ff_torque must be finite and within +/-50 Nm\n";
        return 2;
    }
    if (std::fabs(stepRad) < 1.0e-6f || std::fabs(stepRad) > 0.2f) {
        std::cerr << "step_rad must be nonzero and no larger than 0.2 rad\n";
        return 2;
    }
    if (baselineMs <= 0 || responseMs <= 0 || maxCurrent <= 0 || maxCurrent > 65535 || settleSamples <= 0) {
        std::cerr << "baseline_ms, response_ms, max_current, and settle_samples must be positive\n";
        return 2;
    }
    if (periodUs <= 0) {
        std::cerr << "period_us must be positive\n";
        return 2;
    }
    if (periodUs > 10000) {
        std::cerr << "period_us must be <= 10000 for response testing\n";
        return 2;
    }
    if (rtPriority < 0 || rtPriority > 90) {
        std::cerr << "rt_priority must be in [0, 90]\n";
        return 2;
    }

    try {
        RmdCanSdk::Config const parsedConfig = RmdCanSdk::loadConfig(configPath);
        int const countFromConfig = parsedConfig.totalMotorCount;
        if (motorOneBased > countFromConfig) {
            std::cerr << "motor outside configured motor count\n";
            return 2;
        }
        RmdCanSdk::MotorParameters const* params = findParamsForAlias(parsedConfig, motorOneBased);
        if (params == nullptr) {
            std::cerr << "missing motor parameters for alias " << motorOneBased << "\n";
            return 2;
        }

        auto& sdk = DriverSDK::DriverSDK::instance();
        std::vector<char> modes(static_cast<std::size_t>(countFromConfig), static_cast<char>(mode));
        std::vector<unsigned short> maxCurrents(static_cast<std::size_t>(countFromConfig),
                                                static_cast<unsigned short>(maxCurrent));
        sdk.setMode(modes);
        sdk.setMaxCurr(maxCurrents);
        sdk.init(configPath);

        int const motorIndex = motorOneBased - 1;
        if (!isActiveMotor(sdk, motorIndex)) {
            std::cerr << "requested motor " << motorOneBased << " is not active in this backend\n";
            return 2;
        }

        int const count = sdk.getTotalMotorNr();
        std::vector<DriverSDK::motorActualStruct> actuals(static_cast<std::size_t>(count));
        std::vector<DriverSDK::motorTargetStruct> targets(static_cast<std::size_t>(count));

        int consecutiveReady = 0;
        for (int sample = 0; sample < settleSamples && !gStopRequested; ++sample) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            int const actualStatus = sdk.getMotorActual(actuals);
            if (feedbackReady(sdk, actuals, actualStatus)) {
                ++consecutiveReady;
            } else {
                consecutiveReady = 0;
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

        float const startPosition = actuals[static_cast<std::size_t>(motorIndex)].pos;
        float const finalPosition = startPosition + stepRad;
        if (finalPosition < params->minimumPosition || finalPosition > params->maximumPosition) {
            std::cerr << "step target outside configured position limits: start=" << startPosition
                      << " final=" << finalPosition
                      << " limits=[" << params->minimumPosition << ", " << params->maximumPosition << "]\n";
            return 2;
        }

        for (int active : sdk.getActiveMotors()) {
            std::size_t const index = static_cast<std::size_t>(active);
            targets[index].pos = actuals[index].pos;
            targets[index].vel = 0.0f;
            targets[index].tor = 0.0f;
            targets[index].kp = mode == 5 ? pvtKp : 0.0f;
            targets[index].kd = mode == 5 ? pvtKd : 0.0f;
            targets[index].enabled = 1;
        }

        applyRealtimeSettings(rtPriority);

        int consecutiveOperationEnabled = 0;
        for (int sample = 0; sample < settleSamples && !gStopRequested; ++sample) {
            if (sdk.setMotorTarget(targets) != 0) {
                std::cerr << "setMotorTarget failed while enabling motors\n";
                disableMotors(sdk, targets);
                return 1;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(periodUs));
            int const actualStatus = sdk.getMotorActual(actuals);
            if (operationFeedbackReady(sdk, actuals, actualStatus)) {
                ++consecutiveOperationEnabled;
            } else {
                consecutiveOperationEnabled = 0;
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

        int const baselineIterations = std::max(1, baselineMs * 1000 / periodUs);
        int const responseIterations = std::max(1, responseMs * 1000 / periodUs);
        int const totalIterations = baselineIterations + responseIterations;
        std::vector<Sample> samples;
        samples.reserve(static_cast<std::size_t>(totalIterations));
        TimingStats timing;

        auto const testStart = std::chrono::steady_clock::now();
        auto nextWake = testStart;
        auto previousLoopStart = testStart;
        double const stepCommandMs = static_cast<double>(baselineIterations * periodUs) / 1000.0;

        for (int i = 0; i < totalIterations && !gStopRequested; ++i) {
            auto const loopStart = std::chrono::steady_clock::now();
            double intervalMs = 0.0;
            if (i > 0) {
                intervalMs = elapsedMs(previousLoopStart, loopStart);
            }
            previousLoopStart = loopStart;

            bool const stepActive = i >= baselineIterations;
            targets[static_cast<std::size_t>(motorIndex)].pos = stepActive ? finalPosition : startPosition;
            targets[static_cast<std::size_t>(motorIndex)].tor = stepActive ? feedforwardTorque : 0.0f;
            targets[static_cast<std::size_t>(motorIndex)].kp = mode == 5 ? pvtKp : 0.0f;
            targets[static_cast<std::size_t>(motorIndex)].kd = mode == 5 ? pvtKd : 0.0f;
            int const setStatus = sdk.setMotorTarget(targets);
            int const actualStatus = sdk.getMotorActual(actuals);
            auto const afterIo = std::chrono::steady_clock::now();

            auto const& actual = actuals[static_cast<std::size_t>(motorIndex)];
            Sample sample;
            sample.index = i;
            sample.elapsedMs = elapsedMs(testStart, afterIo);
            sample.intervalMs = intervalMs;
            sample.target = targets[static_cast<std::size_t>(motorIndex)].pos;
            sample.pos = actual.pos;
            sample.vel = actual.vel;
            sample.tor = actual.tor;
            sample.temp = actual.temp;
            sample.driveTemp = actual.driveTemp;
            sample.voltage = actual.voltage;
            sample.status = actual.statusWord;
            sample.error = actual.errorCode;
            sample.actualStatus = actualStatus;
            if (setStatus != 0) {
                std::cerr << "setMotorTarget failed at sample " << i << "\n";
                samples.push_back(sample);
                disableMotors(sdk, targets);
                writeCsv(csvPath, samples);
                return 1;
            }
            if (!operationFeedbackReady(sdk, actuals, actualStatus)) {
                std::cerr << "operation-enabled feedback lost at sample " << i << "\n";
                samples.push_back(sample);
                disableMotors(sdk, targets);
                writeCsv(csvPath, samples);
                return 1;
            }

            nextWake += std::chrono::microseconds(periodUs);
            auto const beforeSleep = std::chrono::steady_clock::now();
            sample.overrunMs = elapsedMs(nextWake, beforeSleep);
            if (i > 0) {
                updateTiming(timing, intervalMs, sample.overrunMs);
            }
            samples.push_back(sample);

            if (beforeSleep < nextWake) {
                std::this_thread::sleep_until(nextWake);
            }
        }

        if (gStopRequested) {
            std::cerr << "stop requested; disabling motors\n";
        }

        disableMotors(sdk, targets);
        if (!writeCsv(csvPath, samples)) {
            return 1;
        }

        ResponseMetrics const metrics = computeMetrics(samples, stepCommandMs, startPosition, finalPosition);
        int badSamples = 0;
        for (auto const& sample : samples) {
            if (sample.actualStatus != 0 || !operationEnabled(sample.status) || sample.error != 0) {
                ++badSamples;
            }
        }

        double const intervalAvgMs = timing.intervals > 0 ? timing.intervalSumMs / timing.intervals : 0.0;
        double const intervalMinMs = timing.intervals > 0 ? timing.intervalMinMs : 0.0;
        std::cout << std::fixed << std::setprecision(6)
                  << "step_response_summary"
                  << " motor=" << motorOneBased
                  << " start=" << startPosition
                  << " target=" << finalPosition
                  << " step=" << stepRad
                  << " mode=" << mode
                  << " kp=" << pvtKp
                  << " kd=" << pvtKd
                  << " ff_torque=" << feedforwardTorque
                  << " period_us=" << periodUs
                  << " samples=" << samples.size()
                  << " interval_min_ms=" << intervalMinMs
                  << " interval_avg_ms=" << intervalAvgMs
                  << " interval_max_ms=" << timing.intervalMaxMs
                  << " overruns=" << timing.overruns
                  << " overrun_max_ms=" << timing.overrunMaxMs
                  << " bad_samples=" << badSamples
                  << " response_start_ms=" << metrics.responseStartMs
                  << " t10_ms=" << metrics.t10Ms
                  << " t50_ms=" << metrics.t50Ms
                  << " t90_ms=" << metrics.t90Ms
                  << " settling_ms=" << metrics.settlingMs
                  << " overshoot=" << metrics.overshoot
                  << " max_abs_error=" << metrics.maxAbsError
                  << " csv=" << csvPath << "\n";

        return gStopRequested ? 130 : 0;
    } catch (std::exception const& ex) {
        std::cerr << "step response test failed: " << ex.what() << "\n";
        return 1;
    }
}
