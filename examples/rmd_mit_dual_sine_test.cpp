#include "rmd_can_sdk/heima_driver_sdk.h"
#include "rmd_can_sdk/rmd_can_config.h"
#include "rmd_can_sdk/rmd_can_transport.h"
#include "rmd_can_sdk/rmd_motion_plan.h"
#include "rmd_can_sdk/rmd_protocol.h"
#include "rmd_can_sdk/rmd_types.h"

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
constexpr float DegToRad = RmdCanSdk::Pi / 180.0f;
constexpr float RadToDeg = 180.0f / RmdCanSdk::Pi;
constexpr int Motor3Id = 3;
constexpr int Motor4Id = 4;
constexpr float SafetyMarginDeg = 2.0f;
constexpr float SettleSec = 1.0f;

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
    return "logs/rmd_mit_dual_sine_" + timestamp();
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

void usage(char const* argv0) {
    std::cerr
        << "Usage: " << argv0 << " <config> [duration_s=8] [sample_hz=500] [frequency_hz=0.2] "
        << "[m3_amp_deg=2] [m3_kp=30] [m3_kd=1] "
        << "[m4_amp_deg=2] [m4_kp=20] [m4_kd=1] [output_prefix] "
        << "[m3_center_deg m3_phase_deg m4_center_deg m4_phase_deg [monitor_hz=25]]\n"
        << "Without centers, targets are relative to each motor's current MIT feedback position. "
        << "With centers, targets are absolute MIT positions. Standard 0x92 angle is monitored as a hard limit.\n";
}

void writeHeader(std::ofstream& out) {
    out << "phase,elapsed_ms,sample_ms,motor_id,alias,target_rad,target_deg,kp,kd,t_ff_nm,"
           "actual_pos_rad,actual_pos_deg,actual_vel_rad_s,actual_torque_nm,status_word,error_code,actual_ok,"
           "standard_angle_deg,standard_angle_valid,status1_error,standard_ok\n";
}

void writeSample(std::ofstream& out,
                 char const* phase,
                 std::chrono::steady_clock::time_point testStart,
                 std::chrono::steady_clock::time_point phaseStart,
                 std::chrono::steady_clock::time_point sampleTime,
                 RmdCanSdk::MotorConfig const& motor,
                 RmdCanSdk::SineTargetSpec const& spec,
                 float targetRad,
                 DriverSDK::motorActualStruct const& actual,
                 bool actualOk,
                 float standardAngleDeg,
                 bool standardAngleValid,
                 std::uint16_t status1Error,
                 bool standardOk) {
    out << phase << ','
        << std::fixed << std::setprecision(3)
        << msBetween(testStart, sampleTime) << ','
        << msBetween(phaseStart, sampleTime) << ','
        << motor.motorId << ','
        << motor.alias << ','
        << std::setprecision(6)
        << targetRad << ','
        << targetRad * RadToDeg << ','
        << spec.kp << ','
        << spec.kd << ','
        << spec.feedforwardTorqueNm << ','
        << actual.pos << ','
        << actual.pos * RadToDeg << ','
        << actual.vel << ','
        << actual.tor << ','
        << actual.statusWord << ','
        << actual.errorCode << ','
        << (actualOk ? 1 : 0) << ','
        << standardAngleDeg << ','
        << (standardAngleValid ? 1 : 0) << ','
        << status1Error << ','
        << (standardOk ? 1 : 0) << "\n";
}

struct MotorRun {
    RmdCanSdk::MotorConfig config;
    RmdCanSdk::SineTargetSpec spec;
    std::size_t index = 0;
    float initialRad = 0.0f;
    float minActualRad = 0.0f;
    float maxActualRad = 0.0f;
    float lastStandardAngleDeg = 0.0f;
    float minStandardAngleDeg = 0.0f;
    float maxStandardAngleDeg = 0.0f;
    bool standardAngleValid = false;
    std::uint16_t status1Error = 0;
    float maxAbsTorqueNm = 0.0f;
    int samples = 0;
    int badSamples = 0;
    int badStandardSamples = 0;
};

struct StandardSafetySample {
    bool angleValid = false;
    float angleDeg = 0.0f;
    bool status1Valid = false;
    std::uint16_t status1Error = 0xffff;
};

bool findMotor(RmdCanSdk::Config const& config, int motorId, RmdCanSdk::MotorConfig* out) {
    auto it = std::find_if(config.motors.begin(), config.motors.end(), [&](RmdCanSdk::MotorConfig const& motor) {
        return motor.motorId == motorId;
    });
    if (it == config.motors.end()) {
        return false;
    }
    *out = *it;
    return true;
}

bool masterDeviceForMotor(RmdCanSdk::Config const& config,
                          RmdCanSdk::MotorConfig const& motor,
                          std::string* device) {
    auto it = std::find_if(config.masters.begin(), config.masters.end(), [&](RmdCanSdk::MasterConfig const& master) {
        return master.order == motor.master;
    });
    if (it == config.masters.end()) {
        return false;
    }
    *device = it->device;
    return true;
}

StandardSafetySample readStandardSafety(RmdCanSdk::SocketCanTransport& transport, int motorId) {
    StandardSafetySample sample;
    int const txId = RmdCanSdk::standardTxId(motorId);
    int const rxId = RmdCanSdk::standardRxId(motorId);
    auto sendAndWait = [&](unsigned char cmd) -> bool {
        unsigned char request[8] = {cmd, 0, 0, 0, 0, 0, 0, 0};
        if (transport.send(txId, request, 8) < 0) {
            return false;
        }
        auto const deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(12);
        while (std::chrono::steady_clock::now() < deadline) {
            RmdCanSdk::CanFrame frame;
            int const ret = transport.receive(frame, 2);
            if (ret <= 0 || frame.id != rxId || frame.length != 8 || frame.data[0] != cmd) {
                continue;
            }
            std::vector<unsigned char> data(frame.data.begin(), frame.data.begin() + frame.length);
            if (cmd == 0x92) {
                auto parsed = RmdCanSdk::parseMultiturnAngleReply(data);
                if (parsed.valid) {
                    sample.angleValid = true;
                    sample.angleDeg = parsed.angleDeg;
                    return true;
                }
            } else if (cmd == 0x9A) {
                auto parsed = RmdCanSdk::parseStatus1Reply(data);
                if (parsed.valid) {
                    sample.status1Valid = true;
                    sample.status1Error = parsed.errorState;
                    return true;
                }
            }
        }
        return false;
    };
    sendAndWait(0x92);
    sendAndWait(0x9A);
    return sample;
}

bool updateStandardSafety(MotorRun& motor,
                          StandardSafetySample const& sample,
                          float marginRad,
                          std::string* abortReason) {
    if (!sample.angleValid) {
        motor.badStandardSamples++;
        std::ostringstream reason;
        reason << "motor " << motor.config.motorId << " missing 0x92 safety angle";
        *abortReason = reason.str();
        return false;
    }
    motor.lastStandardAngleDeg = sample.angleDeg;
    motor.standardAngleValid = true;
    motor.minStandardAngleDeg = std::min(motor.minStandardAngleDeg, sample.angleDeg);
    motor.maxStandardAngleDeg = std::max(motor.maxStandardAngleDeg, sample.angleDeg);
    if (!sample.status1Valid) {
        motor.badStandardSamples++;
        std::ostringstream reason;
        reason << "motor " << motor.config.motorId << " missing 0x9A safety status";
        *abortReason = reason.str();
        return false;
    }
    motor.status1Error = sample.status1Error;
    if (sample.status1Error != 0) {
        motor.badStandardSamples++;
        std::ostringstream reason;
        reason << "motor " << motor.config.motorId << " status1 error=0x"
               << std::hex << sample.status1Error;
        *abortReason = reason.str();
        return false;
    }
    auto const limit = RmdCanSdk::checkAngleWithinLimits(sample.angleDeg * DegToRad,
                                                         motor.config.parameters,
                                                         marginRad);
    if (!limit.valid) {
        motor.badStandardSamples++;
        std::ostringstream reason;
        reason << "motor " << motor.config.motorId << " standard angle "
               << sample.angleDeg << "deg outside monitored range: " << limit.reason;
        *abortReason = reason.str();
        return false;
    }
    return true;
}

bool waitForInitialFeedback(DriverSDK::DriverSDK& sdk,
                            std::vector<DriverSDK::motorActualStruct>& actuals,
                            MotorRun& motor3,
                            MotorRun& motor4) {
    for (int i = 0; i < 200 && !stopRequested.load(std::memory_order_acquire); ++i) {
        sdk.getMotorActual(actuals);
        auto const& a3 = actuals[motor3.index];
        auto const& a4 = actuals[motor4.index];
        if (a3.statusWord == 0x0237 && a3.errorCode == 0 &&
            a4.statusWord == 0x0237 && a4.errorCode == 0) {
            motor3.initialRad = a3.pos;
            motor4.initialRad = a4.pos;
            motor3.minActualRad = motor3.maxActualRad = a3.pos;
            motor4.minActualRad = motor4.maxActualRad = a4.pos;
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
}

void disableMit(DriverSDK::DriverSDK& sdk, std::vector<DriverSDK::motorTargetStruct>& targets) {
    for (auto& target : targets) {
        target.enabled = 0;
        target.tor = 0.0f;
    }
    sdk.setMotorTarget(targets);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

bool updateTargetsAndLog(DriverSDK::DriverSDK& sdk,
                         std::vector<DriverSDK::motorTargetStruct>& targets,
                         std::vector<DriverSDK::motorActualStruct>& actuals,
                         MotorRun& motor3,
                         MotorRun& motor4,
                         float target3,
                         float target4,
                         char const* phase,
                         std::chrono::steady_clock::time_point testStart,
                         std::chrono::steady_clock::time_point phaseStart,
                         std::ofstream& samples,
                         bool monitorStandard,
                         RmdCanSdk::SocketCanTransport* safetyTransport,
                         float marginRad,
                         std::string* abortReason) {
    targets[motor3.index].pos = target3;
    targets[motor3.index].vel = 0.0f;
    targets[motor3.index].kp = motor3.spec.kp;
    targets[motor3.index].kd = motor3.spec.kd;
    targets[motor3.index].tor = motor3.spec.feedforwardTorqueNm;
    targets[motor3.index].enabled = 1;

    targets[motor4.index].pos = target4;
    targets[motor4.index].vel = 0.0f;
    targets[motor4.index].kp = motor4.spec.kp;
    targets[motor4.index].kd = motor4.spec.kd;
    targets[motor4.index].tor = motor4.spec.feedforwardTorqueNm;
    targets[motor4.index].enabled = 1;

    sdk.setMotorTarget(targets);
    if (monitorStandard && safetyTransport != nullptr) {
        if (!updateStandardSafety(motor3, readStandardSafety(*safetyTransport, motor3.config.motorId), marginRad, abortReason) ||
            !updateStandardSafety(motor4, readStandardSafety(*safetyTransport, motor4.config.motorId), marginRad, abortReason)) {
            return false;
        }
    }
    sdk.getMotorActual(actuals);
    auto const now = std::chrono::steady_clock::now();

    for (auto* motor : {&motor3, &motor4}) {
        auto const& actual = actuals[motor->index];
        bool const actualOk = actual.statusWord == 0x0237 && actual.errorCode == 0;
        float const target = motor == &motor3 ? target3 : target4;
        bool const standardOk = motor->standardAngleValid && motor->status1Error == 0;
        writeSample(samples, phase, testStart, phaseStart, now, motor->config, motor->spec, target, actual, actualOk,
                    motor->lastStandardAngleDeg, motor->standardAngleValid, motor->status1Error, standardOk);
        motor->samples++;
        if (!actualOk) {
            motor->badSamples++;
            std::ostringstream reason;
            reason << "motor " << motor->config.motorId << " unhealthy status=0x"
                   << std::hex << actual.statusWord << std::dec << " error=" << actual.errorCode;
            *abortReason = reason.str();
            return false;
        }
        motor->minActualRad = std::min(motor->minActualRad, actual.pos);
        motor->maxActualRad = std::max(motor->maxActualRad, actual.pos);
        motor->maxAbsTorqueNm = std::max(motor->maxAbsTorqueNm, std::fabs(actual.tor));
    }
    return true;
}

bool runAuditSelfTest() {
    RmdCanSdk::MotorParameters motor3;
    motor3.minimumPosition = -0.7f;
    motor3.maximumPosition = 1.2f;
    RmdCanSdk::SineTargetSpec m3;
    m3.centerRad = 0.0f;
    m3.amplitudeRad = 10.0f * DegToRad;
    if (!RmdCanSdk::checkSineTargetWithinLimits(m3, motor3, SafetyMarginDeg * DegToRad).valid) {
        std::cerr << "audit self-test failed for motor 3 target\n";
        return false;
    }
    RmdCanSdk::MotorParameters motor4;
    motor4.minimumPosition = -1.5f;
    motor4.maximumPosition = 0.05f;
    RmdCanSdk::SineTargetSpec m4;
    m4.centerRad = -75.0f * DegToRad;
    m4.amplitudeRad = 8.0f * DegToRad;
    if (!RmdCanSdk::checkSineTargetWithinLimits(m4, motor4, SafetyMarginDeg * DegToRad).valid) {
        std::cerr << "audit self-test failed for motor 4 target\n";
        return false;
    }
    m4.centerRad = -85.0f * DegToRad;
    if (RmdCanSdk::checkSineTargetWithinLimits(m4, motor4, SafetyMarginDeg * DegToRad).valid) {
        std::cerr << "audit self-test failed to reject unsafe motor 4 target\n";
        return false;
    }
    std::cout << "rmd_mit_dual_sine_test audit self-test passed\n";
    return true;
}

} // namespace

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--audit-self-test") {
        return runAuditSelfTest() ? 0 : 1;
    }
    if (argc < 2) {
        usage(argv[0]);
        return 2;
    }
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    std::string const configPath = argv[1];
    int durationSec = 8;
    int sampleHz = 500;
    float frequencyHz = 0.2f;
    float m3AmpDeg = 2.0f;
    float m3Kp = 30.0f;
    float m3Kd = 1.0f;
    float m4AmpDeg = 2.0f;
    float m4Kp = 20.0f;
    float m4Kd = 1.0f;
    std::string outputPrefix = argc > 11 ? argv[11] : defaultOutputPrefix();
    bool const absoluteMode = argc > 12;
    float m3CenterDeg = 0.0f;
    float m3PhaseDeg = 0.0f;
    float m4CenterDeg = 0.0f;
    float m4PhaseDeg = 0.0f;
    int monitorHz = 25;

    if (absoluteMode && argc != 16 && argc != 17) {
        std::cerr << "absolute mode requires m3_center_deg m3_phase_deg m4_center_deg m4_phase_deg\n";
        return 2;
    }
    if (argc > 17) {
        usage(argv[0]);
        return 2;
    }

    if (argc > 2 && !parseIntStrict(argv[2], &durationSec)) {
        std::cerr << "duration_s must be an integer\n";
        return 2;
    }
    if (argc > 3 && !parseIntStrict(argv[3], &sampleHz)) {
        std::cerr << "sample_hz must be an integer\n";
        return 2;
    }
    if (argc > 4 && !parseFloatStrict(argv[4], &frequencyHz)) {
        std::cerr << "frequency_hz must be numeric\n";
        return 2;
    }
    if (argc > 5 && !parseFloatStrict(argv[5], &m3AmpDeg)) {
        std::cerr << "m3_amp_deg must be numeric\n";
        return 2;
    }
    if (argc > 6 && !parseFloatStrict(argv[6], &m3Kp)) {
        std::cerr << "m3_kp must be numeric\n";
        return 2;
    }
    if (argc > 7 && !parseFloatStrict(argv[7], &m3Kd)) {
        std::cerr << "m3_kd must be numeric\n";
        return 2;
    }
    if (argc > 8 && !parseFloatStrict(argv[8], &m4AmpDeg)) {
        std::cerr << "m4_amp_deg must be numeric\n";
        return 2;
    }
    if (argc > 9 && !parseFloatStrict(argv[9], &m4Kp)) {
        std::cerr << "m4_kp must be numeric\n";
        return 2;
    }
    if (argc > 10 && !parseFloatStrict(argv[10], &m4Kd)) {
        std::cerr << "m4_kd must be numeric\n";
        return 2;
    }
    if (absoluteMode && !parseFloatStrict(argv[12], &m3CenterDeg)) {
        std::cerr << "m3_center_deg must be numeric\n";
        return 2;
    }
    if (absoluteMode && !parseFloatStrict(argv[13], &m3PhaseDeg)) {
        std::cerr << "m3_phase_deg must be numeric\n";
        return 2;
    }
    if (absoluteMode && !parseFloatStrict(argv[14], &m4CenterDeg)) {
        std::cerr << "m4_center_deg must be numeric\n";
        return 2;
    }
    if (absoluteMode && !parseFloatStrict(argv[15], &m4PhaseDeg)) {
        std::cerr << "m4_phase_deg must be numeric\n";
        return 2;
    }
    if (absoluteMode && argc > 16 && !parseIntStrict(argv[16], &monitorHz)) {
        std::cerr << "monitor_hz must be numeric\n";
        return 2;
    }
    if (durationSec <= 0 || durationSec > 60) {
        std::cerr << "duration_s must be in 1..60\n";
        return 2;
    }
    if (sampleHz <= 0 || sampleHz > 1000) {
        std::cerr << "sample_hz must be in 1..1000\n";
        return 2;
    }
    if (frequencyHz <= 0.0f || frequencyHz > 2.0f) {
        std::cerr << "frequency_hz must be in 0..2\n";
        return 2;
    }
    if (monitorHz < 0 || monitorHz > 50) {
        std::cerr << "monitor_hz must be in 0..50\n";
        return 2;
    }
    if (m3AmpDeg < 0.0f || m4AmpDeg < 0.0f) {
        std::cerr << "amplitudes must be non-negative\n";
        return 2;
    }
    if (m3Kp < 0.0f || m3Kp > 500.0f || m4Kp < 0.0f || m4Kp > 500.0f) {
        std::cerr << "kp values must be in 0..500\n";
        return 2;
    }
    if (m3Kd < RmdCanSdk::RmdKdMin || m3Kd > RmdCanSdk::RmdKdMax ||
        m4Kd < RmdCanSdk::RmdKdMin || m4Kd > RmdCanSdk::RmdKdMax) {
        std::cerr << "kd values must be in " << RmdCanSdk::RmdKdMin << ".." << RmdCanSdk::RmdKdMax << "\n";
        return 2;
    }

    RmdCanSdk::Config config = RmdCanSdk::loadConfig(configPath);
    if (config.motors.size() != 2) {
        std::cerr << "this dual sine test requires exactly two configured RMD motors\n";
        return 2;
    }

    MotorRun motor3;
    MotorRun motor4;
    if (!findMotor(config, Motor3Id, &motor3.config) || !findMotor(config, Motor4Id, &motor4.config)) {
        std::cerr << "config must contain motor IDs 3 and 4\n";
        return 2;
    }
    motor3.index = static_cast<std::size_t>(motor3.config.alias - 1);
    motor4.index = static_cast<std::size_t>(motor4.config.alias - 1);
    motor3.spec.centerRad = 0.0f;
    motor3.spec.amplitudeRad = m3AmpDeg * DegToRad;
    motor3.spec.phaseRad = 0.0f;
    motor3.spec.kp = m3Kp;
    motor3.spec.kd = m3Kd;
    motor4.spec.centerRad = 0.0f;
    motor4.spec.amplitudeRad = m4AmpDeg * DegToRad;
    motor4.spec.phaseRad = 0.0f;
    motor4.spec.kp = m4Kp;
    motor4.spec.kd = m4Kd;
    if (absoluteMode) {
        motor3.spec.centerRad = m3CenterDeg * DegToRad;
        motor3.spec.phaseRad = m3PhaseDeg * DegToRad;
        motor4.spec.centerRad = m4CenterDeg * DegToRad;
        motor4.spec.phaseRad = m4PhaseDeg * DegToRad;
    }

    float const marginRad = SafetyMarginDeg * DegToRad;
    if (!absoluteMode &&
        (motor3.spec.amplitudeRad > 5.0f * DegToRad || motor4.spec.amplitudeRad > 5.0f * DegToRad)) {
        std::cerr << "relative amplitudes are limited to <=5deg for this safety test\n";
        return 2;
    }
    if (absoluteMode) {
        auto const m3ConfigLimit = RmdCanSdk::checkSineTargetWithinLimits(motor3.spec, motor3.config.parameters, marginRad);
        auto const m4ConfigLimit = RmdCanSdk::checkSineTargetWithinLimits(motor4.spec, motor4.config.parameters, marginRad);
        if (!m3ConfigLimit.valid || !m4ConfigLimit.valid) {
            std::cerr << "absolute target exceeds configured safety limits: "
                      << (!m3ConfigLimit.valid ? m3ConfigLimit.reason : m4ConfigLimit.reason) << "\n";
            return 2;
        }
    }

    std::string safetyDevice;
    if (!masterDeviceForMotor(config, motor3.config, &safetyDevice) ||
        motor4.config.master != motor3.config.master) {
        std::cerr << "both motors must reference the same configured CAN master for this safety monitor\n";
        return 2;
    }
    RmdCanSdk::SocketCanTransport safetyTransport;
    if (safetyTransport.open(safetyDevice) != 0) {
        std::cerr << "failed to open safety monitor CAN device " << safetyDevice << "\n";
        return 2;
    }
    auto initialStd3 = readStandardSafety(safetyTransport, motor3.config.motorId);
    auto initialStd4 = readStandardSafety(safetyTransport, motor4.config.motorId);
    std::string initialSafetyReason;
    motor3.minStandardAngleDeg = motor3.maxStandardAngleDeg = initialStd3.angleDeg;
    motor4.minStandardAngleDeg = motor4.maxStandardAngleDeg = initialStd4.angleDeg;
    if (!updateStandardSafety(motor3, initialStd3, marginRad, &initialSafetyReason) ||
        !updateStandardSafety(motor4, initialStd4, marginRad, &initialSafetyReason)) {
        std::cerr << "initial standard safety check failed: " << initialSafetyReason << "\n";
        return 1;
    }

    std::filesystem::path prefixPath(outputPrefix);
    if (prefixPath.has_parent_path()) {
        std::filesystem::create_directories(prefixPath.parent_path());
    }
    std::string const samplesPath = outputPrefix + "_samples.csv";
    std::ofstream samples(samplesPath);
    if (!samples) {
        std::cerr << "failed to open output file " << samplesPath << "\n";
        return 2;
    }
    writeHeader(samples);

    std::cout << "MIT dual sine config=" << configPath
              << " duration_s=" << durationSec
              << " sample_hz=" << sampleHz
              << " frequency_hz=" << frequencyHz
              << " mode=" << (absoluteMode ? "absolute" : "relative")
              << " monitor_hz=" << monitorHz
              << " output=" << samplesPath
              << " safety_can=" << safetyDevice << "\n";
    std::cout << "initial standard angles: motor3=" << motor3.lastStandardAngleDeg
              << "deg motor4=" << motor4.lastStandardAngleDeg
              << "deg safety_margin=" << SafetyMarginDeg << "deg\n";

    auto& sdk = DriverSDK::DriverSDK::instance();
    sdk.init(configPath.c_str());
    int const count = sdk.getTotalMotorNr();
    if (motor3.index >= static_cast<std::size_t>(count) || motor4.index >= static_cast<std::size_t>(count)) {
        std::cerr << "motor alias is outside SDK target vector\n";
        return 2;
    }
    std::vector<DriverSDK::motorTargetStruct> targets(static_cast<std::size_t>(count));
    std::vector<DriverSDK::motorActualStruct> actuals(static_cast<std::size_t>(count));
    if (!waitForInitialFeedback(sdk, actuals, motor3, motor4)) {
        disableMit(sdk, targets);
        std::cerr << "could not read healthy initial MIT feedback from both motors\n";
        return 1;
    }
    if (!absoluteMode) {
        motor3.spec.centerRad = motor3.initialRad;
        motor4.spec.centerRad = motor4.initialRad;
    }
    RmdCanSdk::MotorParameters protocolLimits;
    protocolLimits.minimumPosition = RmdCanSdk::RmdPMin;
    protocolLimits.maximumPosition = RmdCanSdk::RmdPMax;
    auto const mit3Limit = RmdCanSdk::checkSineTargetWithinLimits(motor3.spec, protocolLimits, 0.0f);
    auto const mit4Limit = RmdCanSdk::checkSineTargetWithinLimits(motor4.spec, protocolLimits, 0.0f);
    auto const config3Limit = RmdCanSdk::checkSineTargetWithinLimits(motor3.spec, motor3.config.parameters, marginRad);
    auto const config4Limit = RmdCanSdk::checkSineTargetWithinLimits(motor4.spec, motor4.config.parameters, marginRad);
    if (!mit3Limit.valid || !mit4Limit.valid || !config3Limit.valid || !config4Limit.valid) {
        disableMit(sdk, targets);
        std::cerr << "MIT target exceeds safety limits: "
                  << (!mit3Limit.valid ? mit3Limit.reason :
                      !mit4Limit.valid ? mit4Limit.reason :
                      !config3Limit.valid ? config3Limit.reason : config4Limit.reason)
                  << "\n";
        return 2;
    }
    std::cout << "motor3 alias=" << motor3.config.alias
              << " initial_mit=" << motor3.initialRad * RadToDeg
              << "deg center=" << motor3.spec.centerRad * RadToDeg
              << "deg amp=" << m3AmpDeg
              << "deg phase=" << motor3.spec.phaseRad * RadToDeg
              << "deg kp=" << motor3.spec.kp << " kd=" << motor3.spec.kd
              << " t_ff=0 mit_target_range_deg=[" << mit3Limit.minTargetRad * RadToDeg
              << "," << mit3Limit.maxTargetRad * RadToDeg << "]\n";
    std::cout << "motor4 alias=" << motor4.config.alias
              << " initial_mit=" << motor4.initialRad * RadToDeg
              << "deg center=" << motor4.spec.centerRad * RadToDeg
              << "deg amp=" << m4AmpDeg
              << "deg phase=" << motor4.spec.phaseRad * RadToDeg
              << "deg kp=" << motor4.spec.kp << " kd=" << motor4.spec.kd
              << " t_ff=0 mit_target_range_deg=[" << mit4Limit.minTargetRad * RadToDeg
              << "," << mit4Limit.maxTargetRad * RadToDeg << "]\n";

    auto const samplePeriod =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1.0 / sampleHz));
    auto const testStart = std::chrono::steady_clock::now();
    std::string abortReason;
    int const monitorEvery = monitorHz > 0 ? std::max(1, sampleHz / monitorHz) : INT_MAX;
    int sampleCounter = 0;

    auto runPhase = [&](char const* phase, float seconds, auto targetFn) {
        auto const phaseStart = std::chrono::steady_clock::now();
        while (!stopRequested.load(std::memory_order_acquire)) {
            auto const now = std::chrono::steady_clock::now();
            float const elapsed = static_cast<float>(std::chrono::duration<double>(now - phaseStart).count());
            if (elapsed >= seconds) {
                break;
            }
            auto const targetsNow = targetFn(elapsed / seconds, elapsed);
            bool const monitorStandard = (sampleCounter++ % monitorEvery) == 0;
            if (!updateTargetsAndLog(sdk, targets, actuals, motor3, motor4,
                                     targetsNow.first, targetsNow.second,
                                     phase, testStart, phaseStart, samples,
                                     monitorStandard, &safetyTransport, marginRad, &abortReason)) {
                return false;
            }
            std::this_thread::sleep_until(now + samplePeriod);
        }
        return true;
    };

    float const motor3StartTarget = RmdCanSdk::sineTargetAt(motor3.spec, 0.0f, frequencyHz);
    float const motor4StartTarget = RmdCanSdk::sineTargetAt(motor4.spec, 0.0f, frequencyHz);
    bool ok = runPhase(absoluteMode ? "settle_absolute_start" : "settle_relative_center", SettleSec, [&](float, float) {
        return std::pair<float, float>{
            motor3StartTarget,
            motor4StartTarget,
        };
    });
    if (ok) {
        ok = runPhase("sine", static_cast<float>(durationSec), [&](float, float elapsed) {
            return std::pair<float, float>{
                RmdCanSdk::sineTargetAt(motor3.spec, elapsed, frequencyHz),
                RmdCanSdk::sineTargetAt(motor4.spec, elapsed, frequencyHz),
            };
        });
    }

    samples.flush();
    disableMit(sdk, targets);

    auto printSummary = [](char const* label, MotorRun const& motor) {
        std::cout << label
                  << " samples=" << motor.samples
                  << " bad_samples=" << motor.badSamples
                  << " actual_range_deg=[" << motor.minActualRad * RadToDeg
                  << "," << motor.maxActualRad * RadToDeg << "]"
                  << " standard_range_deg=[" << motor.minStandardAngleDeg
                  << "," << motor.maxStandardAngleDeg << "]"
                  << " bad_standard_samples=" << motor.badStandardSamples
                  << " max_abs_torque_nm=" << motor.maxAbsTorqueNm << "\n";
    };
    printSummary("motor3", motor3);
    printSummary("motor4", motor4);
    if (!ok || stopRequested.load(std::memory_order_acquire)) {
        std::cerr << "dual sine aborted: " << (abortReason.empty() ? "signal requested" : abortReason) << "\n";
        return 1;
    }
    std::cout << "MIT dual sine completed\n";
    return 0;
}
