#include "rmd_can_sdk/heima_driver_sdk.h"
#include "rmd_can_sdk/rmd_can_config.h"
#include "rmd_can_sdk/rmd_motion_plan.h"
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
constexpr float DegToRad = Pi / 180.0f;
constexpr float RadToDeg = 180.0f / Pi;
constexpr int SettleMs = 1000;

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
    return "logs/rmd_mit_sine_" + timestamp();
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

void writeSampleHeader(std::ofstream& out) {
    out << "phase,elapsed_ms,sample_ms,target_rad,target_deg,target_vel_rad_s,"
           "actual_pos_rad,actual_pos_deg,actual_vel_rad_s,actual_torque_nm,error_deg,"
           "status_word,error_code,actual_ok\n";
}

struct Stats {
    int samples = 0;
    int actualFailures = 0;
    double sumAbsErrorDeg = 0.0;
    double sumSquaredErrorDeg = 0.0;
    double maxAbsErrorDeg = 0.0;
    double maxAbsVelocityRadS = 0.0;
    double maxAbsTorqueNm = 0.0;
    double sumSampleDtMs = 0.0;
    double maxSampleDtMs = 0.0;
    int sampleDtCount = 0;
};

void disableMit(DriverSDK::DriverSDK& sdk, std::vector<DriverSDK::motorTargetStruct>& targets) {
    for (auto& target : targets) {
        target.enabled = 0;
        target.tor = 0.0f;
    }
    sdk.setMotorTarget(targets);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void writeSample(std::ofstream& samples,
                 char const* phase,
                 std::chrono::steady_clock::time_point testStart,
                 std::chrono::steady_clock::time_point phaseStart,
                 std::chrono::steady_clock::time_point sampleTime,
                 float targetPos,
                 float targetVel,
                 DriverSDK::motorActualStruct const& actual,
                 bool actualOk) {
    float const errorDeg = actualOk ? (actual.pos - targetPos) * RadToDeg : 0.0f;
    samples << phase << ','
            << std::fixed << std::setprecision(3)
            << msBetween(testStart, sampleTime) << ','
            << msBetween(phaseStart, sampleTime) << ','
            << std::setprecision(6)
            << targetPos << ','
            << targetPos * RadToDeg << ','
            << targetVel << ','
            << actual.pos << ','
            << actual.pos * RadToDeg << ','
            << actual.vel << ','
            << actual.tor << ','
            << errorDeg << ','
            << actual.statusWord << ','
            << actual.errorCode << ','
            << (actualOk ? 1 : 0) << "\n";
}

void runSettle(DriverSDK::DriverSDK& sdk,
               std::vector<DriverSDK::motorTargetStruct>& targets,
               std::vector<DriverSDK::motorActualStruct>& actuals,
               std::size_t index,
               std::chrono::steady_clock::duration samplePeriod,
               std::chrono::steady_clock::time_point testStart,
               std::ofstream& samples,
               float settlePos) {
    targets[index].pos = settlePos;
    targets[index].vel = 0.0f;
    targets[index].tor = 0.0f;
    sdk.setMotorTarget(targets);

    auto const phaseStart = std::chrono::steady_clock::now();
    auto nextSample = phaseStart + samplePeriod;
    auto const deadline = phaseStart + std::chrono::milliseconds(SettleMs);
    DriverSDK::motorActualStruct lastActual{};

    while (!stopRequested.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_until(nextSample);
        auto const sampleTime = std::chrono::steady_clock::now();
        nextSample += samplePeriod;
        bool const actualOk = sdk.getMotorActual(actuals) == 0 &&
            actuals[index].statusWord == 0x0237 && actuals[index].errorCode == 0;
        if (actualOk) {
            lastActual = actuals[index];
        }
        writeSample(samples, "settle", testStart, phaseStart, sampleTime,
                    settlePos, 0.0f, lastActual, actualOk);
    }
}

Stats runSine(DriverSDK::DriverSDK& sdk,
              std::vector<DriverSDK::motorTargetStruct>& targets,
              std::vector<DriverSDK::motorActualStruct>& actuals,
              std::size_t index,
              int durationSec,
              std::chrono::steady_clock::duration samplePeriod,
              RmdCanSdk::SineTargetSpec const& targetSpec,
              float frequencyHz,
              std::chrono::steady_clock::time_point testStart,
              std::ofstream& samples) {
    Stats stats;
    DriverSDK::motorActualStruct lastActual{};
    if (sdk.getMotorActual(actuals) == 0 &&
        actuals[index].statusWord == 0x0237 && actuals[index].errorCode == 0) {
        lastActual = actuals[index];
    }

    auto const phaseStart = std::chrono::steady_clock::now();
    auto const deadline = phaseStart + std::chrono::seconds(durationSec);
    auto nextSample = phaseStart + samplePeriod;
    auto nextPrint = phaseStart;
    auto lastSample = phaseStart;

    while (!stopRequested.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_until(nextSample);
        auto const sampleTime = std::chrono::steady_clock::now();
        nextSample += samplePeriod;

        double const elapsedSec = std::chrono::duration<double>(sampleTime - phaseStart).count();
        float const targetPos = RmdCanSdk::sineTargetAt(targetSpec, static_cast<float>(elapsedSec), frequencyHz);
        float const targetVel = targetSpec.amplitudeRad * 2.0f * Pi * frequencyHz *
            static_cast<float>(std::cos(2.0 * static_cast<double>(Pi) * frequencyHz * elapsedSec + targetSpec.phaseRad));

        targets[index].pos = targetPos;
        targets[index].vel = 0.0f;
        targets[index].tor = 0.0f;
        sdk.setMotorTarget(targets);

        bool const actualOk = sdk.getMotorActual(actuals) == 0 &&
            actuals[index].statusWord == 0x0237 && actuals[index].errorCode == 0;
        if (!actualOk) {
            ++stats.actualFailures;
        } else {
            lastActual = actuals[index];
            double const errorDeg = static_cast<double>(lastActual.pos - targetPos) * RadToDeg;
            double const absErrorDeg = std::fabs(errorDeg);
            ++stats.samples;
            stats.sumAbsErrorDeg += absErrorDeg;
            stats.sumSquaredErrorDeg += errorDeg * errorDeg;
            stats.maxAbsErrorDeg = std::max(stats.maxAbsErrorDeg, absErrorDeg);
            stats.maxAbsVelocityRadS = std::max(stats.maxAbsVelocityRadS, static_cast<double>(std::fabs(lastActual.vel)));
            stats.maxAbsTorqueNm = std::max(stats.maxAbsTorqueNm, static_cast<double>(std::fabs(lastActual.tor)));
        }

        double const dtMs = msBetween(lastSample, sampleTime);
        stats.sumSampleDtMs += dtMs;
        stats.maxSampleDtMs = std::max(stats.maxSampleDtMs, dtMs);
        ++stats.sampleDtCount;
        lastSample = sampleTime;

        if (sampleTime >= nextPrint) {
            std::cout << "sine t=" << std::fixed << std::setprecision(3) << elapsedSec
                      << "s target=" << targetPos * RadToDeg
                      << "deg actual=" << lastActual.pos * RadToDeg
                      << "deg err=" << (lastActual.pos - targetPos) * RadToDeg
                      << "deg vel=" << lastActual.vel
                      << "rad/s tor=" << lastActual.tor
                      << " status=0x" << std::hex << lastActual.statusWord << std::dec
                      << " error=0x" << std::hex << lastActual.errorCode << std::dec << "\n";
            nextPrint = sampleTime + std::chrono::milliseconds(500);
        }

        writeSample(samples, "sine", testStart, phaseStart, sampleTime,
                    targetPos, targetVel, lastActual, actualOk);
        if ((stats.samples + stats.actualFailures) % 500 == 0) {
            samples.flush();
        }
    }
    return stats;
}

void writeSummary(std::ofstream& out,
                  int durationSec,
                  int sampleHz,
                  float centerDeg,
                  float amplitudeDeg,
                  float phaseDeg,
                  float frequencyHz,
                  float kp,
                  float kd,
                  Stats const& stats) {
    double const meanAbsErrorDeg = stats.samples > 0 ? stats.sumAbsErrorDeg / stats.samples : 0.0;
    double const rmsErrorDeg = stats.samples > 0 ? std::sqrt(stats.sumSquaredErrorDeg / stats.samples) : 0.0;
    double const avgSampleDtMs = stats.sampleDtCount > 0 ? stats.sumSampleDtMs / stats.sampleDtCount : 0.0;
    out << "duration_s,sample_hz,center_deg,amplitude_deg,phase_deg,frequency_hz,kp,kd,samples,actual_failures,"
           "mean_abs_error_deg,rms_error_deg,max_abs_error_deg,max_abs_velocity_rad_s,"
           "max_abs_torque_nm,sample_dt_avg_ms,sample_dt_max_ms\n";
    out << durationSec << ','
        << sampleHz << ','
        << std::fixed << std::setprecision(6)
        << centerDeg << ','
        << amplitudeDeg << ','
        << phaseDeg << ','
        << frequencyHz << ','
        << kp << ','
        << kd << ','
        << stats.samples << ','
        << stats.actualFailures << ','
        << meanAbsErrorDeg << ','
        << rmsErrorDeg << ','
        << stats.maxAbsErrorDeg << ','
        << stats.maxAbsVelocityRadS << ','
        << stats.maxAbsTorqueNm << ','
        << avgSampleDtMs << ','
        << stats.maxSampleDtMs << "\n";
}

} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    std::string const configPath = argc > 1 ? argv[1] : "config/example_rmd_can.xml";
    int durationSec = argc > 2 ? std::atoi(argv[2]) : 20;
    int sampleHz = argc > 3 ? std::atoi(argv[3]) : 1000;
    float amplitudeDeg = argc > 4 ? std::atof(argv[4]) : 20.0f;
    float frequencyHz = argc > 5 ? std::atof(argv[5]) : 2.0f;
    float kp = argc > 6 ? std::atof(argv[6]) : 50.0f;
    float kd = argc > 7 ? std::atof(argv[7]) : 2.0f;
    std::string outputPrefix = argc > 8 ? argv[8] : defaultOutputPrefix();
    float centerDeg = argc > 9 ? std::atof(argv[9]) : 0.0f;
    float phaseDeg = argc > 10 ? std::atof(argv[10]) : 0.0f;

    if (argc > 2 && !parseIntStrict(argv[2], &durationSec)) {
        std::cerr << "duration_s must be an integer\n";
        return 2;
    }
    if (argc > 3 && !parseIntStrict(argv[3], &sampleHz)) {
        std::cerr << "sample_hz must be an integer\n";
        return 2;
    }
    if (argc > 4 && !parseFloatStrict(argv[4], &amplitudeDeg)) {
        std::cerr << "amplitude_deg must be a number\n";
        return 2;
    }
    if (argc > 5 && !parseFloatStrict(argv[5], &frequencyHz)) {
        std::cerr << "frequency_hz must be a number\n";
        return 2;
    }
    if (argc > 6 && !parseFloatStrict(argv[6], &kp)) {
        std::cerr << "kp must be a number\n";
        return 2;
    }
    if (argc > 7 && !parseFloatStrict(argv[7], &kd)) {
        std::cerr << "kd must be a number\n";
        return 2;
    }
    if (argc > 9 && !parseFloatStrict(argv[9], &centerDeg)) {
        std::cerr << "center_deg must be a number\n";
        return 2;
    }
    if (argc > 10 && !parseFloatStrict(argv[10], &phaseDeg)) {
        std::cerr << "phase_deg must be a number\n";
        return 2;
    }
    if (durationSec <= 0 || durationSec > 600) {
        std::cerr << "duration_s must be in 1..600\n";
        return 2;
    }
    if (sampleHz <= 0 || sampleHz > 2000) {
        std::cerr << "sample_hz must be in 1..2000\n";
        return 2;
    }
    if (amplitudeDeg <= 0.0f || amplitudeDeg > 180.0f) {
        std::cerr << "amplitude_deg must be in 0..180\n";
        return 2;
    }
    if (frequencyHz <= 0.0f || frequencyHz > 20.0f) {
        std::cerr << "frequency_hz must be in 0..20\n";
        return 2;
    }
    if (kp < 0.0f || kp > 500.0f) {
        std::cerr << "kp must be in 0..500\n";
        return 2;
    }
    if (kd < RmdCanSdk::RmdKdMin || kd > RmdCanSdk::RmdKdMax) {
        std::cerr << "kd must be in " << RmdCanSdk::RmdKdMin << ".." << RmdCanSdk::RmdKdMax << "\n";
        return 2;
    }

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

    float const centerRad = centerDeg * DegToRad;
    float const amplitudeRad = amplitudeDeg * DegToRad;
    float const phaseRad = phaseDeg * DegToRad;
    RmdCanSdk::SineTargetSpec targetSpec;
    targetSpec.centerRad = centerRad;
    targetSpec.amplitudeRad = amplitudeRad;
    targetSpec.phaseRad = phaseRad;
    targetSpec.kp = kp;
    targetSpec.kd = kd;
    auto const limitCheck = RmdCanSdk::checkSineTargetWithinLimits(targetSpec, motor.parameters, 0.0f);
    if (!limitCheck.valid) {
        std::cerr << "sine target outside configured limits: " << limitCheck.reason << "\n";
        return 2;
    }

    std::filesystem::path prefixPath(outputPrefix);
    if (prefixPath.has_parent_path()) {
        std::filesystem::create_directories(prefixPath.parent_path());
    }
    std::string const samplesPath = outputPrefix + "_samples.csv";
    std::string const summaryPath = outputPrefix + "_summary.csv";
    std::ofstream samples(samplesPath);
    std::ofstream summary(summaryPath);
    if (!samples || !summary) {
        std::cerr << "failed to open output files with prefix " << outputPrefix << "\n";
        return 2;
    }
    writeSampleHeader(samples);

    std::cout << "MIT sine test config=" << configPath
              << " can=" << masterIt->device
              << " motor_id=" << motor.motorId
              << " alias=" << motor.alias
              << " duration_s=" << durationSec
              << " sample_hz=" << sampleHz
              << " center_deg=" << centerDeg
              << " amplitude_deg=" << amplitudeDeg
              << " phase_deg=" << phaseDeg
              << " frequency_hz=" << frequencyHz
              << " kp=" << kp
              << " kd=" << kd << "\n";
    std::cout << "writing samples: " << samplesPath << "\n";
    std::cout << "writing summary: " << summaryPath << "\n";

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

    targets[index].enabled = 1;
    targets[index].kp = kp;
    targets[index].kd = kd;
    targets[index].vel = 0.0f;
    targets[index].tor = 0.0f;
    targets[index].pos = centerRad;

    auto const samplePeriod =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1.0 / sampleHz));
    float const maxTargetVelocity = amplitudeRad * 2.0f * Pi * frequencyHz;
    std::cout << "target trajectory: pos=center+amplitude*sin(2*pi*f*t), target_vel field remains 0 for position-based RL\n";
    std::cout << "target range=[" << limitCheck.minTargetRad * RadToDeg
              << "," << limitCheck.maxTargetRad * RadToDeg << "]deg\n";
    std::cout << "max target velocity from trajectory=" << maxTargetVelocity
              << "rad/s (" << maxTargetVelocity * RadToDeg << "deg/s)\n";
    float const startTargetRad = RmdCanSdk::sineTargetAt(targetSpec, 0.0f, frequencyHz);
    std::cout << "settling at " << startTargetRad * RadToDeg
              << "deg for " << SettleMs << "ms before sine phase\n";

    auto const testStart = std::chrono::steady_clock::now();
    runSettle(sdk, targets, actuals, index, samplePeriod, testStart, samples, startTargetRad);
    if (stopRequested.load(std::memory_order_acquire)) {
        disableMit(sdk, targets);
        return 1;
    }

    Stats const stats = runSine(sdk, targets, actuals, index,
                                durationSec, samplePeriod,
                                targetSpec, frequencyHz,
                                testStart, samples);
    writeSummary(summary, durationSec, sampleHz, centerDeg, amplitudeDeg, phaseDeg, frequencyHz, kp, kd, stats);
    samples.flush();
    summary.flush();
    disableMit(sdk, targets);
    return stopRequested.load(std::memory_order_acquire) ? 1 : 0;
}
