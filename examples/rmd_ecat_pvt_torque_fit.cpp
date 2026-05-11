#include "rmd_can_sdk/heima_driver_sdk.h"
#include "rmd_can_sdk/rmd_bench_workflow.h"
#include "rmd_can_sdk/rmd_can_config.h"

#include <algorithm>
#include <array>
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

struct PointResult {
    int point = 0;
    float target = 0.0f;
    int samples = 0;
    double posMean = 0.0;
    double targetMean = 0.0;
    double errorMean = 0.0;
    double torqueMean = 0.0;
    double torqueMin = std::numeric_limits<double>::infinity();
    double torqueMax = -std::numeric_limits<double>::infinity();
    double velocityAbsMax = 0.0;
    double tempMean = 0.0;
    double driveTempMean = 0.0;
    double voltageMean = 0.0;
    unsigned short lastStatus = 0xffff;
    unsigned short lastError = 0;
    int badSamples = 0;
};

struct FitResult {
    std::vector<double> coeffs;
    double rmse = 0.0;
};

void requestStop(int) {
    gStopRequested = 1;
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

bool solveLinearSystem(std::vector<std::vector<double>> a, std::vector<double> b, std::vector<double>& x) {
    int const n = static_cast<int>(b.size());
    for (int col = 0; col < n; ++col) {
        int pivot = col;
        for (int row = col + 1; row < n; ++row) {
            if (std::fabs(a[row][col]) > std::fabs(a[pivot][col])) {
                pivot = row;
            }
        }
        if (std::fabs(a[pivot][col]) < 1.0e-12) {
            return false;
        }
        if (pivot != col) {
            std::swap(a[pivot], a[col]);
            std::swap(b[pivot], b[col]);
        }
        double const div = a[col][col];
        for (int j = col; j < n; ++j) {
            a[col][j] /= div;
        }
        b[col] /= div;
        for (int row = 0; row < n; ++row) {
            if (row == col) {
                continue;
            }
            double const factor = a[row][col];
            for (int j = col; j < n; ++j) {
                a[row][j] -= factor * a[col][j];
            }
            b[row] -= factor * b[col];
        }
    }
    x = b;
    return true;
}

FitResult fitModel(std::vector<PointResult> const& points, int featureCount,
                   double (*feature)(PointResult const&, int)) {
    std::vector<std::vector<double>> normal(static_cast<std::size_t>(featureCount),
                                            std::vector<double>(static_cast<std::size_t>(featureCount), 0.0));
    std::vector<double> rhs(static_cast<std::size_t>(featureCount), 0.0);
    for (auto const& p : points) {
        for (int i = 0; i < featureCount; ++i) {
            double const fi = feature(p, i);
            rhs[static_cast<std::size_t>(i)] += fi * p.torqueMean;
            for (int j = 0; j < featureCount; ++j) {
                normal[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] += fi * feature(p, j);
            }
        }
    }

    FitResult result;
    if (!solveLinearSystem(normal, rhs, result.coeffs)) {
        result.rmse = std::numeric_limits<double>::infinity();
        return result;
    }
    double sumSq = 0.0;
    for (auto const& p : points) {
        double pred = 0.0;
        for (int i = 0; i < featureCount; ++i) {
            pred += result.coeffs[static_cast<std::size_t>(i)] * feature(p, i);
        }
        double const error = pred - p.torqueMean;
        sumSq += error * error;
    }
    result.rmse = points.empty() ? 0.0 : std::sqrt(sumSq / static_cast<double>(points.size()));
    return result;
}

double trigFeature(PointResult const& p, int index) {
    if (index == 0) {
        return std::sin(p.posMean);
    }
    if (index == 1) {
        return std::cos(p.posMean);
    }
    return 1.0;
}

double linearFeature(PointResult const& p, int index) {
    if (index == 0) {
        return p.posMean;
    }
    return 1.0;
}

bool writeCsv(std::string const& path, std::vector<PointResult> const& points) {
    std::ofstream out(path);
    if (!out) {
        std::cerr << "failed to open CSV output: " << path << "\n";
        return false;
    }
    out << "point,target,pos_mean,target_mean,error_mean,torque_mean,torque_min,torque_max,velocity_abs_max,"
           "temp_mean,drive_temp_mean,voltage_mean,last_status,last_error,bad_samples,samples\n";
    out << std::fixed << std::setprecision(6);
    for (auto const& p : points) {
        out << p.point << ','
            << p.target << ','
            << p.posMean << ','
            << p.targetMean << ','
            << p.errorMean << ','
            << p.torqueMean << ','
            << p.torqueMin << ','
            << p.torqueMax << ','
            << p.velocityAbsMax << ','
            << p.tempMean << ','
            << p.driveTempMean << ','
            << p.voltageMean << ','
            << p.lastStatus << ','
            << p.lastError << ','
            << p.badSamples << ','
            << p.samples << '\n';
    }
    return true;
}

} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);

    if (argc != 16) {
        std::cerr << "usage: " << argv[0]
                  << " <config.xml> <motor> <start_pos> <end_pos> <points> <max_current>"
                  << " <settle_samples> <period_us> <ramp_ms> <hold_ms> <fit_window_ms>"
                  << " <csv_path> <rt_priority> <kp> <kd>\n";
        return 2;
    }

    char const* configPath = argv[1];
    int const motorOneBased = parseInt(argv[2], "motor");
    float startPos = 0.0f;
    float endPos = 0.0f;
    if (!parseFloatArg(argv[3], "start_pos", startPos) ||
        !parseFloatArg(argv[4], "end_pos", endPos)) {
        return 2;
    }
    int const points = parseInt(argv[5], "points");
    int const maxCurrent = parseInt(argv[6], "max_current");
    int const settleSamples = parseInt(argv[7], "settle_samples");
    int const periodUs = parseInt(argv[8], "period_us");
    int const rampMs = parseInt(argv[9], "ramp_ms");
    int const holdMs = parseInt(argv[10], "hold_ms");
    int const fitWindowMs = parseInt(argv[11], "fit_window_ms");
    std::string const csvPath = argv[12];
    int const rtPriority = parseInt(argv[13], "rt_priority");
    float kp = 0.0f;
    float kd = 0.0f;
    if (!parseFloatArg(argv[14], "kp", kp) ||
        !parseFloatArg(argv[15], "kd", kd)) {
        return 2;
    }

    if (motorOneBased <= 0) {
        std::cerr << "motor must be positive\n";
        return 2;
    }
    if (points < 3) {
        std::cerr << "points must be at least 3\n";
        return 2;
    }
    if (maxCurrent <= 0 || maxCurrent > 65535 || settleSamples <= 0 || periodUs <= 0 ||
        rampMs <= 0 || holdMs <= 0 || fitWindowMs <= 0 || fitWindowMs > holdMs) {
        std::cerr << "numeric timing/current arguments are invalid\n";
        return 2;
    }
    if (periodUs > 10000) {
        std::cerr << "period_us must be <= 10000\n";
        return 2;
    }
    if (rtPriority < 0 || rtPriority > 90) {
        std::cerr << "rt_priority must be in [0, 90]\n";
        return 2;
    }
    if (!std::isfinite(kp) || !std::isfinite(kd) || kp < 0.0f || kd < 0.0f) {
        std::cerr << "kp and kd must be finite non-negative numbers\n";
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
        if (startPos < params->minimumPosition || startPos > params->maximumPosition ||
            endPos < params->minimumPosition || endPos > params->maximumPosition) {
            std::cerr << "scan range outside configured position limits ["
                      << params->minimumPosition << ", " << params->maximumPosition << "]\n";
            return 2;
        }

        auto& sdk = DriverSDK::DriverSDK::instance();
        std::vector<char> modes(static_cast<std::size_t>(countFromConfig), static_cast<char>(5));
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

        for (int active : sdk.getActiveMotors()) {
            std::size_t const index = static_cast<std::size_t>(active);
            targets[index].pos = actuals[index].pos;
            targets[index].vel = 0.0f;
            targets[index].tor = 0.0f;
            targets[index].kp = kp;
            targets[index].kd = kd;
            targets[index].enabled = 1;
        }

        applyRealtimeSettings(rtPriority);

        int const rampIterations = std::max(1, rampMs * 1000 / periodUs);
        int const holdIterations = std::max(1, holdMs * 1000 / periodUs);
        int const windowIterations = std::max(1, fitWindowMs * 1000 / periodUs);
        std::vector<PointResult> results;
        results.reserve(static_cast<std::size_t>(points));
        float commandedPosition = actuals[static_cast<std::size_t>(motorIndex)].pos;

        for (int point = 0; point < points && !gStopRequested; ++point) {
            float const alpha = points == 1 ? 0.0f : static_cast<float>(point) / static_cast<float>(points - 1);
            float const targetPosition = startPos + (endPos - startPos) * alpha;
            float const rampStart = commandedPosition;

            auto nextWake = std::chrono::steady_clock::now();
            for (int i = 0; i < rampIterations && !gStopRequested; ++i) {
                float const progress = static_cast<float>(i + 1) / static_cast<float>(rampIterations);
                targets[static_cast<std::size_t>(motorIndex)].pos =
                    rampStart + (targetPosition - rampStart) * progress;
                if (sdk.setMotorTarget(targets) != 0) {
                    std::cerr << "setMotorTarget failed during ramp at point " << point << "\n";
                    disableMotors(sdk, targets);
                    return 1;
                }
                sdk.getMotorActual(actuals);
                nextWake += std::chrono::microseconds(periodUs);
                std::this_thread::sleep_until(nextWake);
            }
            commandedPosition = targetPosition;
            targets[static_cast<std::size_t>(motorIndex)].pos = targetPosition;

            PointResult row;
            row.point = point;
            row.target = targetPosition;
            nextWake = std::chrono::steady_clock::now();
            for (int i = 0; i < holdIterations && !gStopRequested; ++i) {
                if (sdk.setMotorTarget(targets) != 0) {
                    std::cerr << "setMotorTarget failed during hold at point " << point << "\n";
                    disableMotors(sdk, targets);
                    return 1;
                }
                int const actualStatus = sdk.getMotorActual(actuals);
                auto const& actual = actuals[static_cast<std::size_t>(motorIndex)];
                if (i >= holdIterations - windowIterations) {
                    ++row.samples;
                    row.posMean += actual.pos;
                    row.targetMean += targetPosition;
                    row.errorMean += targetPosition - actual.pos;
                    row.torqueMean += actual.tor;
                    row.torqueMin = std::min(row.torqueMin, static_cast<double>(actual.tor));
                    row.torqueMax = std::max(row.torqueMax, static_cast<double>(actual.tor));
                    row.velocityAbsMax = std::max(row.velocityAbsMax, std::fabs(static_cast<double>(actual.vel)));
                    row.tempMean += actual.temp;
                    row.driveTempMean += actual.driveTemp;
                    row.voltageMean += actual.voltage;
                    row.lastStatus = actual.statusWord;
                    row.lastError = actual.errorCode;
                    if (actualStatus != 0 || !operationEnabled(actual.statusWord) || actual.errorCode != 0) {
                        ++row.badSamples;
                    }
                }
                nextWake += std::chrono::microseconds(periodUs);
                std::this_thread::sleep_until(nextWake);
            }
            if (row.samples > 0) {
                double const inv = 1.0 / static_cast<double>(row.samples);
                row.posMean *= inv;
                row.targetMean *= inv;
                row.errorMean *= inv;
                row.torqueMean *= inv;
                row.tempMean *= inv;
                row.driveTempMean *= inv;
                row.voltageMean *= inv;
            }
            results.push_back(row);
            std::cout << std::fixed << std::setprecision(6)
                      << "fit_point point=" << point
                      << " target=" << row.target
                      << " pos_mean=" << row.posMean
                      << " error_mean=" << row.errorMean
                      << " torque_mean=" << row.torqueMean
                      << " bad_samples=" << row.badSamples << "\n";
        }

        disableMotors(sdk, targets);
        if (!writeCsv(csvPath, results)) {
            return 1;
        }

        FitResult const trig = fitModel(results, 3, trigFeature);
        FitResult const linear = fitModel(results, 2, linearFeature);
        std::cout << std::fixed << std::setprecision(6)
                  << "fit_summary"
                  << " motor=" << motorOneBased
                  << " points=" << results.size()
                  << " start=" << startPos
                  << " end=" << endPos
                  << " kp=" << kp
                  << " kd=" << kd
                  << " max_current=" << maxCurrent;
        if (trig.coeffs.size() == 3) {
            std::cout << " trig_a_sin=" << trig.coeffs[0]
                      << " trig_b_cos=" << trig.coeffs[1]
                      << " trig_c=" << trig.coeffs[2]
                      << " trig_rmse=" << trig.rmse;
        }
        if (linear.coeffs.size() == 2) {
            std::cout << " linear_a=" << linear.coeffs[0]
                      << " linear_b=" << linear.coeffs[1]
                      << " linear_rmse=" << linear.rmse;
        }
        std::cout << " csv=" << csvPath << "\n";
        return gStopRequested ? 130 : 0;
    } catch (std::exception const& ex) {
        std::cerr << "PVT torque fit failed: " << ex.what() << "\n";
        return 1;
    }
}
