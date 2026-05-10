#include "rmd_can_sdk/heima_driver_sdk.h"
#include "rmd_can_sdk/rmd_can_config.h"
#include "rmd_can_sdk/rmd_protocol.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <climits>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

std::atomic<bool> stopRequested{false};
constexpr float Pi = 3.14159265358979323846f;
constexpr float RadToDeg = 180.0f / Pi;
constexpr float PosAbortDeg = 15.0f;
constexpr float VelAbortRadS = 8.0f;
constexpr float TorqueAbortNm = 18.0f;

struct Phase {
    std::string name;
    float torqueNm = 0.0f;
};

struct PhaseStats {
    std::string name;
    float commandTorqueNm = 0.0f;
    int samples = 0;
    int usedSamples = 0;
    int actualFailures = 0;
    double meanActualTorqueNm = 0.0;
    double meanPosErrorDeg = 0.0;
    double meanVelocityRadS = 0.0;
    double maxAbsActualTorqueNm = 0.0;
    double maxAbsPosErrorDeg = 0.0;
    double maxAbsVelocityRadS = 0.0;
};

void handleSignal(int) {
    stopRequested.store(true, std::memory_order_release);
}

double msBetween(std::chrono::steady_clock::time_point begin,
                 std::chrono::steady_clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

std::string timestamp() {
    auto now = std::time(nullptr);
    std::tm tm{};
    localtime_r(&now, &tm);
    std::ostringstream out;
    out << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return out.str();
}

std::string defaultOutputPrefix() {
    std::filesystem::create_directories("logs");
    return "logs/rmd_mit_torque_probe_" + timestamp();
}

bool parseFloatStrict(char const* text, float* value) {
    if (text == nullptr || *text == '\0') {
        return false;
    }
    char* end = nullptr;
    float parsed = std::strtof(text, &end);
    if (end == text || *end != '\0' || !std::isfinite(parsed)) {
        return false;
    }
    *value = parsed;
    return true;
}

bool parseIntStrict(char const* text, int* value) {
    if (text == nullptr || *text == '\0') {
        return false;
    }
    errno = 0;
    char* end = nullptr;
    long parsed = std::strtol(text, &end, 10);
    if (end == text || *end != '\0' || errno == ERANGE || parsed < INT_MIN || parsed > INT_MAX) {
        return false;
    }
    *value = static_cast<int>(parsed);
    return true;
}

void disableMit(DriverSDK::DriverSDK& sdk, std::vector<DriverSDK::motorTargetStruct>& targets) {
    for (auto& target : targets) {
        target.enabled = 0;
        target.tor = 0.0f;
    }
    sdk.setMotorTarget(targets);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

bool waitForActual(DriverSDK::DriverSDK& sdk,
                   std::vector<DriverSDK::motorActualStruct>& actuals,
                   std::size_t index,
                   DriverSDK::motorActualStruct& actual) {
    auto const deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);
    while (std::chrono::steady_clock::now() < deadline) {
        if (sdk.getMotorActual(actuals) == 0) {
            auto const candidate = actuals[index];
            if (candidate.statusWord == 0x0237 && candidate.errorCode == 0) {
                actual = candidate;
                return true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
}

void writeCommandBytes(RmdCanSdk::MotorParameters const& params,
                       float targetPos,
                       float kp,
                       float kd,
                       float torqueNm) {
    auto bytes = RmdCanSdk::packMit(params.polarity * targetPos + params.countBias,
                                    0.0f,
                                    kp,
                                    kd,
                                    params.polarity * torqueNm,
                                    params.maximumTorque);
    std::cout << "command bytes tor=" << torqueNm << "Nm:";
    for (unsigned char byte : bytes) {
        std::cout << ' ' << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<int>(byte);
    }
    std::cout << std::dec << std::setfill(' ') << "\n";
}

} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    std::string const configPath = argc > 1 ? argv[1] : "config/example_rmd_can.xml";
    float ffTorqueNm = 1.0f;
    float kp = 80.0f;
    float kd = 0.5f;
    int phaseMs = 1200;
    int sampleHz = 1000;
    std::string outputPrefix = defaultOutputPrefix();
    bool hasTargetPos = false;
    float targetPosRad = 0.0f;
    float torqueStepNm = 0.0f;
    float torqueMinNm = 0.0f;

    if (argc > 2 && !parseFloatStrict(argv[2], &ffTorqueNm)) {
        std::cerr << "ff_torque_nm must be a number\n";
        return 2;
    }
    if (argc > 3 && !parseFloatStrict(argv[3], &kp)) {
        std::cerr << "kp must be a number\n";
        return 2;
    }
    if (argc > 4 && !parseFloatStrict(argv[4], &kd)) {
        std::cerr << "kd must be a number\n";
        return 2;
    }
    if (argc > 5 && !parseIntStrict(argv[5], &phaseMs)) {
        std::cerr << "phase_ms must be an integer\n";
        return 2;
    }
    if (argc > 6 && !parseIntStrict(argv[6], &sampleHz)) {
        std::cerr << "sample_hz must be an integer\n";
        return 2;
    }
    if (argc > 7) {
        outputPrefix = argv[7];
    }
    if (argc > 8) {
        if (!parseFloatStrict(argv[8], &targetPosRad)) {
            std::cerr << "target_pos_rad must be a number\n";
            return 2;
        }
        hasTargetPos = true;
    }
    if (argc > 9 && !parseFloatStrict(argv[9], &torqueStepNm)) {
        std::cerr << "torque_step_nm must be a number\n";
        return 2;
    }
    if (argc > 10 && !parseFloatStrict(argv[10], &torqueMinNm)) {
        std::cerr << "torque_min_nm must be a number\n";
        return 2;
    }

    if (ffTorqueNm < 0.0f || ffTorqueNm > 10.0f) {
        std::cerr << "ff_torque_nm must be in [0,10]\n";
        return 2;
    }
    if (kp < RmdCanSdk::RmdKpMin || kp > RmdCanSdk::RmdKpMax) {
        std::cerr << "kp must be in " << RmdCanSdk::RmdKpMin << ".." << RmdCanSdk::RmdKpMax << "\n";
        return 2;
    }
    if (kd < RmdCanSdk::RmdKdMin || kd > RmdCanSdk::RmdKdMax) {
        std::cerr << "kd must be in " << RmdCanSdk::RmdKdMin << ".." << RmdCanSdk::RmdKdMax << "\n";
        return 2;
    }
    if (phaseMs < 300 || phaseMs > 5000) {
        std::cerr << "phase_ms must be in 300..5000\n";
        return 2;
    }
    if (sampleHz < 100 || sampleHz > 2000) {
        std::cerr << "sample_hz must be in 100..2000\n";
        return 2;
    }
    if (torqueStepNm < 0.0f || torqueStepNm > 10.0f) {
        std::cerr << "torque_step_nm must be in 0..10\n";
        return 2;
    }
    if (torqueMinNm < -10.0f || torqueMinNm > 10.0f) {
        std::cerr << "torque_min_nm must be in [-10,10]\n";
        return 2;
    }
    if (argc > 10 && torqueStepNm <= 0.0f) {
        std::cerr << "torque_step_nm must be >0 when torque_min_nm is provided\n";
        return 2;
    }
    if (torqueStepNm > 0.0f && torqueMinNm > ffTorqueNm) {
        std::cerr << "torque_min_nm must be <= ff_torque_nm\n";
        return 2;
    }

    std::filesystem::path prefixPath(outputPrefix);
    if (prefixPath.has_parent_path()) {
        std::filesystem::create_directories(prefixPath.parent_path());
    }
    std::string const samplePath = outputPrefix + "_samples.csv";
    std::string const summaryPath = outputPrefix + "_summary.csv";
    std::ofstream samples(samplePath);
    std::ofstream summary(summaryPath);
    samples << "phase,elapsed_ms,phase_ms,command_torque_nm,target_pos_deg,actual_pos_deg,"
               "pos_error_deg,actual_vel_rad_s,actual_torque_nm,status_word,error_code,actual_ok\n";
    summary << "phase,command_torque_nm,samples,used_samples,actual_failures,mean_actual_torque_nm,"
               "mean_pos_error_deg,mean_velocity_rad_s,max_abs_actual_torque_nm,max_abs_pos_error_deg,"
               "max_abs_velocity_rad_s\n";

    RmdCanSdk::Config config = RmdCanSdk::loadConfig(configPath);
    if (config.motors.size() != 1) {
        std::cerr << "this manual test requires exactly one configured RMD motor\n";
        return 2;
    }
    auto const& motor = config.motors.front();
    auto masterIt = std::find_if(config.masters.begin(), config.masters.end(), [&](RmdCanSdk::MasterConfig const& master) {
        return master.order == motor.master;
    });
    if (masterIt == config.masters.end()) {
        std::cerr << "configured motor references missing CAN master\n";
        return 2;
    }

    std::cout << "MIT torque probe config=" << configPath
              << " can=" << masterIt->device
              << " motor_id=" << motor.motorId
              << " alias=" << motor.alias
              << " ff_torque_nm=" << ffTorqueNm
              << " kp=" << kp
              << " kd=" << kd
              << " phase_ms=" << phaseMs
              << " sample_hz=" << sampleHz
              << " target_pos_rad=" << (hasTargetPos ? targetPosRad : 0.0f)
              << " torque_step_nm=" << torqueStepNm
              << " torque_min_nm=" << torqueMinNm << "\n"
              << "writing samples: " << samplePath << "\n"
              << "writing summary: " << summaryPath << "\n";

    auto& sdk = DriverSDK::DriverSDK::instance();
    sdk.init(configPath.c_str());
    int const count = sdk.getTotalMotorNr();
    std::vector<DriverSDK::motorTargetStruct> targets(static_cast<std::size_t>(count));
    std::vector<DriverSDK::motorActualStruct> actuals(static_cast<std::size_t>(count));
    std::size_t const index = static_cast<std::size_t>(motor.alias - 1);
    if (index >= targets.size()) {
        std::cerr << "motor alias is outside target vector\n";
        return 2;
    }

    DriverSDK::motorActualStruct initial{};
    if (!waitForActual(sdk, actuals, index, initial)) {
        std::cerr << "could not read initial MIT feedback\n";
        return 1;
    }
    float const targetPos = hasTargetPos ? targetPosRad : initial.pos;
    if (targetPos < motor.parameters.minimumPosition || targetPos > motor.parameters.maximumPosition) {
        std::cerr << "target position " << targetPos << "rad is outside configured safe range ["
                  << motor.parameters.minimumPosition << ", " << motor.parameters.maximumPosition << "]\n";
        return 1;
    }
    if (!hasTargetPos &&
        (initial.pos < motor.parameters.minimumPosition || initial.pos > motor.parameters.maximumPosition)) {
        std::cerr << "current position " << initial.pos << "rad is outside configured safe range ["
                  << motor.parameters.minimumPosition << ", " << motor.parameters.maximumPosition << "]\n";
        return 1;
    }
    std::cout << "target position " << targetPos << "rad (" << targetPos * RadToDeg << "deg)"
              << " initial=" << initial.pos << "rad (" << initial.pos * RadToDeg << "deg)\n";
    writeCommandBytes(motor.parameters, targetPos, kp, kd, 0.0f);
    if (torqueStepNm > 0.0f && torqueMinNm != 0.0f) {
        writeCommandBytes(motor.parameters, targetPos, kp, kd, torqueMinNm);
    }
    writeCommandBytes(motor.parameters, targetPos, kp, kd, ffTorqueNm);
    if (torqueStepNm == 0.0f) {
        writeCommandBytes(motor.parameters, targetPos, kp, kd, -ffTorqueNm);
    }

    targets[index].enabled = 1;
    targets[index].pos = initial.pos;
    targets[index].vel = 0.0f;
    targets[index].kp = kp;
    targets[index].kd = kd;
    targets[index].tor = 0.0f;

    if (std::fabs(targetPos - initial.pos) * RadToDeg > 2.0f) {
        auto const rampStart = std::chrono::steady_clock::now();
        auto const rampDuration = std::chrono::milliseconds(2500);
        auto const rampPeriod =
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1.0 / sampleHz));
        auto nextRamp = rampStart;
        std::cout << "preposition ramp from " << initial.pos * RadToDeg
                  << "deg to " << targetPos * RadToDeg << "deg with t_ff=0\n";
        while (!stopRequested.load(std::memory_order_acquire)) {
            auto const now = std::chrono::steady_clock::now();
            double const alpha = std::min(1.0, msBetween(rampStart, now) /
                                                static_cast<double>(rampDuration.count()));
            targets[index].pos = static_cast<float>(initial.pos + (targetPos - initial.pos) * alpha);
            targets[index].tor = 0.0f;
            sdk.setMotorTarget(targets);
            if (sdk.getMotorActual(actuals) == 0) {
                auto const actual = actuals[index];
                if (std::fabs(actual.vel) > VelAbortRadS || std::fabs(actual.tor) > TorqueAbortNm) {
                    std::cerr << "safety abort during preposition: vel_rad_s=" << actual.vel
                              << " torque_nm=" << actual.tor << "\n";
                    disableMit(sdk, targets);
                    return 1;
                }
            }
            if (alpha >= 1.0) {
                break;
            }
            nextRamp += rampPeriod;
            std::this_thread::sleep_until(nextRamp);
        }
        targets[index].pos = targetPos;
        targets[index].tor = 0.0f;
        sdk.setMotorTarget(targets);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    std::vector<Phase> phases;
    if (torqueStepNm > 0.0f) {
        if (torqueMinNm < 0.0f) {
            phases.push_back({"torque_0", 0.0f});
            if (ffTorqueNm > 0.0f) {
                for (float torqueNm = torqueStepNm; torqueNm < ffTorqueNm; torqueNm += torqueStepNm) {
                    std::ostringstream name;
                    name << "torque_" << torqueNm;
                    phases.push_back({name.str(), torqueNm});
                }
                std::ostringstream name;
                name << "torque_" << ffTorqueNm;
                phases.push_back({name.str(), ffTorqueNm});
            }
            for (float torqueNm = -torqueStepNm; torqueNm > torqueMinNm; torqueNm -= torqueStepNm) {
                std::ostringstream name;
                name << "torque_" << torqueNm;
                phases.push_back({name.str(), torqueNm});
            }
            {
                std::ostringstream name;
                name << "torque_" << torqueMinNm;
                phases.push_back({name.str(), torqueMinNm});
            }
        } else {
            for (float torqueNm = torqueMinNm; torqueNm < ffTorqueNm; torqueNm += torqueStepNm) {
                std::ostringstream name;
                name << "torque_" << torqueNm;
                phases.push_back({name.str(), torqueNm});
            }
            std::ostringstream name;
            name << "torque_" << ffTorqueNm;
            phases.push_back({name.str(), ffTorqueNm});
        }
    } else if (ffTorqueNm == 0.0f) {
        phases.push_back({"zero_hold", 0.0f});
    } else {
        phases = {
            {"zero_a", 0.0f},
            {"positive_a", ffTorqueNm},
            {"zero_b", 0.0f},
            {"negative_a", -ffTorqueNm},
            {"zero_c", 0.0f},
            {"positive_b", ffTorqueNm},
            {"zero_d", 0.0f},
            {"negative_b", -ffTorqueNm},
            {"zero_e", 0.0f},
        };
    }

    targets[index].pos = targetPos;
    targets[index].vel = 0.0f;
    targets[index].kp = kp;
    targets[index].kd = kd;
    targets[index].tor = 0.0f;

    auto const samplePeriod =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1.0 / sampleHz));
    auto const testStart = std::chrono::steady_clock::now();
    int const ignoreMs = std::min(300, phaseMs / 3);
    std::vector<PhaseStats> allStats;
    bool safetyAbort = false;

    for (Phase const& phase : phases) {
        PhaseStats stats;
        stats.name = phase.name;
        stats.commandTorqueNm = phase.torqueNm;
        auto const phaseStart = std::chrono::steady_clock::now();
        auto const deadline = phaseStart + std::chrono::milliseconds(phaseMs);
        auto nextSample = phaseStart;

        while (!stopRequested.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline) {
            targets[index].tor = phase.torqueNm;
            sdk.setMotorTarget(targets);
            nextSample += samplePeriod;
            std::this_thread::sleep_until(nextSample);
            auto const now = std::chrono::steady_clock::now();
            double const elapsedMs = msBetween(testStart, now);
            double const phaseElapsedMs = msBetween(phaseStart, now);
            bool const actualOk = sdk.getMotorActual(actuals) == 0;
            ++stats.samples;
            if (!actualOk) {
                ++stats.actualFailures;
                samples << phase.name << ',' << elapsedMs << ',' << phaseElapsedMs << ','
                        << phase.torqueNm << ',' << targetPos * RadToDeg
                        << ",0,0,0,0,0,0,0\n";
                continue;
            }

            auto const actual = actuals[index];
            double const actualPosDeg = static_cast<double>(actual.pos) * RadToDeg;
            double const posErrorDeg = actualPosDeg - static_cast<double>(targetPos) * RadToDeg;
            double const absPosErrorDeg = std::fabs(posErrorDeg);
            double const absVel = std::fabs(actual.vel);
            double const absTorque = std::fabs(actual.tor);
            bool const outsideConfigLimits =
                actual.pos < motor.parameters.minimumPosition || actual.pos > motor.parameters.maximumPosition;

            samples << phase.name << ',' << std::fixed << std::setprecision(3)
                    << elapsedMs << ',' << phaseElapsedMs << ','
                    << std::setprecision(6)
                    << phase.torqueNm << ','
                    << static_cast<double>(targetPos) * RadToDeg << ','
                    << actualPosDeg << ','
                    << posErrorDeg << ','
                    << actual.vel << ','
                    << actual.tor << ','
                    << actual.statusWord << ','
                    << actual.errorCode << ",1\n";

            if (phaseElapsedMs >= ignoreMs) {
                ++stats.usedSamples;
                stats.meanActualTorqueNm += actual.tor;
                stats.meanPosErrorDeg += posErrorDeg;
                stats.meanVelocityRadS += actual.vel;
                stats.maxAbsActualTorqueNm = std::max(stats.maxAbsActualTorqueNm, absTorque);
                stats.maxAbsPosErrorDeg = std::max(stats.maxAbsPosErrorDeg, absPosErrorDeg);
                stats.maxAbsVelocityRadS = std::max(stats.maxAbsVelocityRadS, absVel);
            }

            if (outsideConfigLimits || absPosErrorDeg > PosAbortDeg || absVel > VelAbortRadS || absTorque > TorqueAbortNm) {
                std::cerr << "safety abort in phase " << phase.name
                      << ": pos_error_deg=" << posErrorDeg
                      << " vel_rad_s=" << actual.vel
                      << " torque_nm=" << actual.tor
                      << " actual_pos_rad=" << actual.pos << "\n";
                safetyAbort = true;
                break;
            }
        }

        if (stats.usedSamples > 0) {
            stats.meanActualTorqueNm /= stats.usedSamples;
            stats.meanPosErrorDeg /= stats.usedSamples;
            stats.meanVelocityRadS /= stats.usedSamples;
        }
        summary << stats.name << ',' << std::fixed << std::setprecision(6)
                << stats.commandTorqueNm << ','
                << stats.samples << ','
                << stats.usedSamples << ','
                << stats.actualFailures << ','
                << stats.meanActualTorqueNm << ','
                << stats.meanPosErrorDeg << ','
                << stats.meanVelocityRadS << ','
                << stats.maxAbsActualTorqueNm << ','
                << stats.maxAbsPosErrorDeg << ','
                << stats.maxAbsVelocityRadS << '\n';
        summary.flush();
        std::cout << "phase=" << stats.name
                  << " cmd_tor=" << stats.commandTorqueNm
                  << " mean_actual_tor=" << stats.meanActualTorqueNm
                  << " mean_pos_error_deg=" << stats.meanPosErrorDeg
                  << " max_pos_error_deg=" << stats.maxAbsPosErrorDeg
                  << " failures=" << stats.actualFailures << "\n";
        allStats.push_back(stats);
        if (safetyAbort) {
            break;
        }
    }

    targets[index].tor = 0.0f;
    targets[index].pos = targetPos;
    sdk.setMotorTarget(targets);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    disableMit(sdk, targets);

    auto meanForSign = [&](int sign, double PhaseStats::*field) {
        double sum = 0.0;
        int countSign = 0;
        for (auto const& stats : allStats) {
            if ((sign > 0 && stats.commandTorqueNm > 0.0f) ||
                (sign < 0 && stats.commandTorqueNm < 0.0f) ||
                (sign == 0 && stats.commandTorqueNm == 0.0f)) {
                sum += stats.*field;
                ++countSign;
            }
        }
        return countSign == 0 ? 0.0 : sum / countSign;
    };

    double const zeroPos = meanForSign(0, &PhaseStats::meanPosErrorDeg);
    double const posPos = meanForSign(1, &PhaseStats::meanPosErrorDeg);
    double const negPos = meanForSign(-1, &PhaseStats::meanPosErrorDeg);
    double const posTorque = meanForSign(1, &PhaseStats::meanActualTorqueNm);
    double const negTorque = meanForSign(-1, &PhaseStats::meanActualTorqueNm);
    double const expectedBiasDeg = static_cast<double>(ffTorqueNm) / std::max(1.0f, kp) * RadToDeg;

    std::cout << "--- effect summary ---\n"
              << "zero_mean_pos_error_deg=" << zeroPos << "\n"
              << "positive_mean_pos_error_deg=" << posPos << "\n"
              << "negative_mean_pos_error_deg=" << negPos << "\n"
              << "pos_minus_neg_pos_error_deg=" << (posPos - negPos) << "\n"
              << "positive_mean_actual_torque_nm=" << posTorque << "\n"
              << "negative_mean_actual_torque_nm=" << negTorque << "\n"
              << "expected_one_side_bias_deg_if_torque_is_spring_offset≈" << expectedBiasDeg << "\n";

    return safetyAbort || stopRequested.load(std::memory_order_acquire) ? 1 : 0;
}
