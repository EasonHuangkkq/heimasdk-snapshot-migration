#include "rmd_can_sdk/heima_driver_sdk.h"
#include "rmd_can_sdk/rmd_can_config.h"
#include "rmd_can_sdk/rmd_can_transport.h"
#include "rmd_can_sdk/rmd_protocol.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
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
#include <climits>
#include <vector>

namespace {

std::atomic<bool> stopRequested{false};
constexpr float DegToRad = 3.14159265358979323846f / 180.0f;
constexpr float RadToDeg = 180.0f / 3.14159265358979323846f;
constexpr float VisibleMotionPositionThresholdRad = 1.0f * DegToRad;
constexpr float VisibleMotionVelocityThresholdRadS = 5.0f * DegToRad;
constexpr float DefaultSensitiveMotionPositionThresholdDeg = 0.1f;
constexpr float DefaultSensitiveMotionVelocityThresholdDegS = 2.0f;
constexpr float DefaultTargetToleranceDeg = 10.0f;
constexpr float DefaultPreZeroToleranceDeg = 10.0f;
constexpr int MaxA4SpeedDps = 100;
constexpr int MitPreZeroTimeoutMs = 3000;

struct MotionWindow {
    bool detected = false;
    double lowerMs = -1.0;
    double upperMs = -1.0;
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
    return "logs/rmd_mit_sweep_" + timestamp();
}

short i16le(unsigned char lo, unsigned char hi) {
    return static_cast<short>(static_cast<unsigned short>(lo) |
                              (static_cast<unsigned short>(hi) << 8));
}

void putU16(unsigned char* data, int offset, unsigned short value) {
    data[offset] = static_cast<unsigned char>(value & 0xff);
    data[offset + 1] = static_cast<unsigned char>((value >> 8) & 0xff);
}

void putI32(unsigned char* data, int offset, int value) {
    data[offset] = static_cast<unsigned char>(value & 0xff);
    data[offset + 1] = static_cast<unsigned char>((value >> 8) & 0xff);
    data[offset + 2] = static_cast<unsigned char>((value >> 16) & 0xff);
    data[offset + 3] = static_cast<unsigned char>((value >> 24) & 0xff);
}

bool requestReply(RmdCanSdk::SocketCanTransport& transport,
                  int motorId,
                  unsigned char const data[8],
                  unsigned char expectedCmd,
                  RmdCanSdk::CanFrame& out) {
    if (transport.send(RmdCanSdk::standardTxId(motorId), data, 8) < 0) {
        std::cerr << "send failed for cmd 0x" << std::hex << static_cast<int>(data[0]) << std::dec << "\n";
        return false;
    }
    int const rxId = RmdCanSdk::standardRxId(motorId);
    for (int i = 0; i < 10 && !stopRequested.load(std::memory_order_acquire); ++i) {
        RmdCanSdk::CanFrame frame;
        int const ret = transport.receive(frame, 100);
        if (ret <= 0) {
            continue;
        }
        if (frame.id == rxId && frame.length == 8 && frame.data[0] == expectedCmd) {
            out = frame;
            return true;
        }
    }
    return false;
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

bool readStatus1(RmdCanSdk::SocketCanTransport& transport,
                 int motorId,
                 RmdCanSdk::Status1Feedback& status) {
    unsigned char data[8] = {0x9A, 0, 0, 0, 0, 0, 0, 0};
    RmdCanSdk::CanFrame frame;
    if (!requestReply(transport, motorId, data, 0x9A, frame)) {
        return false;
    }
    std::vector<unsigned char> bytes(frame.data.begin(), frame.data.begin() + frame.length);
    status = RmdCanSdk::parseStatus1Reply(bytes);
    if (!status.valid) {
        return false;
    }
    std::cout << "status1 temp=" << status.temperatureC
              << " mos=" << status.mosTemperatureC
              << " brake_release_cmd=" << (status.brakeReleaseCommandActive ? "release" : "lock")
              << " voltage=" << status.voltageV
              << " error=0x" << std::hex << status.errorState << std::dec << "\n";
    return true;
}

bool readStatus2(RmdCanSdk::SocketCanTransport& transport, int motorId, int& angleDeg, int& speedDps, float& iqA) {
    unsigned char data[8] = {0x9C, 0, 0, 0, 0, 0, 0, 0};
    RmdCanSdk::CanFrame frame;
    if (!requestReply(transport, motorId, data, 0x9C, frame)) {
        return false;
    }
    iqA = static_cast<float>(i16le(frame.data[2], frame.data[3])) * 0.01f;
    speedDps = i16le(frame.data[4], frame.data[5]);
    angleDeg = i16le(frame.data[6], frame.data[7]);
    std::cout << "status2 angle=" << angleDeg << "deg speed=" << speedDps << "dps iq=" << iqA << "A\n";
    return true;
}

bool commandA4(RmdCanSdk::SocketCanTransport& transport, int motorId, float targetDeg, int maxSpeedDps) {
    unsigned char data[8] = {0xA4, 0, 0, 0, 0, 0, 0, 0};
    putU16(data, 2, static_cast<unsigned short>(maxSpeedDps));
    putI32(data, 4, static_cast<int>(std::lround(targetDeg * 100.0f)));
    RmdCanSdk::CanFrame frame;
    if (!requestReply(transport, motorId, data, 0xA4, frame)) {
        return false;
    }
    std::cout << "A4 accepted target=" << targetDeg << "deg max_speed=" << maxSpeedDps << "dps\n";
    return true;
}

bool commandMotorRun(RmdCanSdk::SocketCanTransport& transport, int motorId) {
    unsigned char data[8] = {0x88, 0, 0, 0, 0, 0, 0, 0};
    if (transport.send(RmdCanSdk::standardTxId(motorId), data, 8) < 0) {
        std::cerr << "send failed for motor run cmd 0x88\n";
        return false;
    }
    std::cout << "motor run command sent (0x88, no reply required)\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return true;
}

bool moveToZero(std::string const& channel, int motorId, int maxSpeedDps) {
    if (maxSpeedDps <= 0 || maxSpeedDps > MaxA4SpeedDps) {
        std::cerr << "refusing zero max_speed outside 1.." << MaxA4SpeedDps << " dps\n";
        return false;
    }

    RmdCanSdk::SocketCanTransport transport;
    if (transport.open(channel) != 0) {
        std::cerr << "failed to open " << channel << "\n";
        return false;
    }

    RmdCanSdk::Status1Feedback status1;
    if (!readStatus1(transport, motorId, status1)) {
        std::cerr << "failed to read status1\n";
        return false;
    }
    if (status1.errorState != 0) {
        std::cerr << "refusing motion: motor error_state=0x" << std::hex << status1.errorState << std::dec << "\n";
        return false;
    }
    if (!status1.brakeReleaseCommandActive) {
        std::cout << "brake release command state is lock; continuing because 0x9A DATA[3] is not a mechanical brake sensor\n";
    }
    if (!commandMotorRun(transport, motorId)) {
        std::cerr << "motor run command failed to send\n";
        return false;
    }

    int angleDeg = 0;
    int speedDps = 0;
    float iqA = 0.0f;
    if (!readStatus2(transport, motorId, angleDeg, speedDps, iqA)) {
        std::cerr << "failed to read status2\n";
        return false;
    }
    if (!commandA4(transport, motorId, 0.0f, maxSpeedDps)) {
        std::cerr << "A4 zero command did not receive expected reply\n";
        return false;
    }

    auto const deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
    while (!stopRequested.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if (!readStatus2(transport, motorId, angleDeg, speedDps, iqA)) {
            continue;
        }
        if (std::abs(angleDeg) <= 1 && std::abs(speedDps) <= 2) {
            std::cout << "zero reached\n";
            return true;
        }
    }

    std::cerr << "timeout before reaching zero\n";
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

void printWindow(char const* label,
                 std::chrono::steady_clock::time_point commandTime,
                 std::chrono::steady_clock::time_point lower,
                 std::chrono::steady_clock::time_point upper) {
    std::cout << label << "_window_ms=[" << std::fixed << std::setprecision(3)
              << msBetween(commandTime, lower) << ", " << msBetween(commandTime, upper)
              << "] observed_ms=" << msBetween(commandTime, upper) << "\n";
}

void updateMotionWindow(MotionWindow& window,
                        float moved,
                        float velocityChanged,
                        float positionThresholdRad,
                        float velocityThresholdRadS,
                        char const* label,
                        std::chrono::steady_clock::time_point commandTime,
                        std::chrono::steady_clock::time_point lastSample,
                        std::chrono::steady_clock::time_point sampleTime) {
    if (window.detected ||
        (moved < positionThresholdRad && velocityChanged < velocityThresholdRadS)) {
        return;
    }
    window.detected = true;
    window.lowerMs = msBetween(commandTime, lastSample);
    window.upperMs = msBetween(commandTime, sampleTime);
    printWindow(label, commandTime, lastSample, sampleTime);
}

void writeSampleHeader(std::ofstream& out) {
    out << "phase,step,cycle,elapsed_ms,sample_ms,command_rad,command_deg,eval_rad,eval_deg,"
           "start_pos_rad,start_pos_deg,pos_rad,pos_deg,vel_rad_s,tor_nm,eval_error_deg,"
           "sensitive_motion_detected,motion_detected,target_reached,status_word,error_code,actual_ok\n";
}

void writeSummaryHeader(std::ofstream& out) {
    out << "phase,step,cycle,command_rad,command_deg,eval_rad,eval_deg,start_pos_rad,start_pos_deg,start_vel_rad_s,"
           "kp,kd,tolerance_deg,ramp_ms,"
           "sensitive_motion_pos_deg,sensitive_motion_vel_deg_s,"
           "sensitive_motion_detected,sensitive_motion_start_lower_ms,sensitive_motion_start_upper_ms,"
           "motion_detected,motion_start_lower_ms,motion_start_upper_ms,target_reached,"
           "target_reached_lower_ms,target_reached_upper_ms,final_pos_rad,final_pos_deg,"
           "final_vel_rad_s,final_eval_error_deg,max_overshoot_deg,max_abs_vel_rad_s,max_abs_torque_nm,"
           "samples,actual_failures,timeout_ms\n";
}

void writeSample(std::ofstream* samples,
                 char const* phase,
                 int step,
                 int cycle,
                 std::chrono::steady_clock::time_point testStart,
                 std::chrono::steady_clock::time_point commandTime,
                 std::chrono::steady_clock::time_point sampleTime,
                 float commandPos,
                 float evalPos,
                 float startPos,
                 DriverSDK::motorActualStruct const& actual,
                 bool sensitiveMotionDetected,
                 bool visibleMotionDetected,
                 bool targetReached,
                 bool actualOk) {
    if (samples == nullptr || !*samples) {
        return;
    }
    float const evalErrorDeg = std::fabs(actual.pos - evalPos) * RadToDeg;
    *samples << phase << ','
             << step << ','
             << cycle << ','
             << std::fixed << std::setprecision(3)
             << msBetween(testStart, sampleTime) << ','
             << msBetween(commandTime, sampleTime) << ','
             << std::setprecision(6)
             << commandPos << ','
             << commandPos * RadToDeg << ','
             << evalPos << ','
             << evalPos * RadToDeg << ','
             << startPos << ','
             << startPos * RadToDeg << ','
             << actual.pos << ','
             << actual.pos * RadToDeg << ','
             << actual.vel << ','
             << actual.tor << ','
             << evalErrorDeg << ','
             << (sensitiveMotionDetected ? 1 : 0) << ','
             << (visibleMotionDetected ? 1 : 0) << ','
             << (targetReached ? 1 : 0) << ','
             << actual.statusWord << ','
             << actual.errorCode << ','
             << (actualOk ? 1 : 0) << "\n";
}

void writeSummary(std::ofstream* summary,
                  char const* phase,
                  int step,
                  int cycle,
                  float commandPos,
                  float evalPos,
                  float startPos,
                  float startVel,
                  float kp,
                  float kd,
                  float targetToleranceRad,
                  int rampMs,
                  float sensitiveMotionPosThresholdRad,
                  float sensitiveMotionVelThresholdRadS,
                  MotionWindow const& sensitiveMotion,
                  MotionWindow const& visibleMotion,
                  bool targetReached,
                  double targetLowerMs,
                  double targetUpperMs,
                  DriverSDK::motorActualStruct const& finalActual,
                  double maxOvershootDeg,
                  double maxAbsVel,
                  double maxAbsTorque,
                  int samplesWritten,
                  int actualFailures,
                  int timeoutMs) {
    if (summary == nullptr || !*summary) {
        return;
    }
    float const finalErrorDeg = std::fabs(finalActual.pos - evalPos) * RadToDeg;
    *summary << phase << ','
             << step << ','
             << cycle << ','
             << std::fixed << std::setprecision(6)
             << commandPos << ','
             << commandPos * RadToDeg << ','
             << evalPos << ','
             << evalPos * RadToDeg << ','
             << startPos << ','
             << startPos * RadToDeg << ','
             << startVel << ','
             << kp << ','
             << kd << ','
             << targetToleranceRad * RadToDeg << ','
             << rampMs << ','
             << sensitiveMotionPosThresholdRad * RadToDeg << ','
             << sensitiveMotionVelThresholdRadS * RadToDeg << ','
             << (sensitiveMotion.detected ? 1 : 0) << ','
             << std::setprecision(3)
             << sensitiveMotion.lowerMs << ','
             << sensitiveMotion.upperMs << ','
             << (visibleMotion.detected ? 1 : 0) << ','
             << visibleMotion.lowerMs << ','
             << visibleMotion.upperMs << ','
             << (targetReached ? 1 : 0) << ','
             << targetLowerMs << ','
             << targetUpperMs << ','
             << std::setprecision(6)
             << finalActual.pos << ','
             << finalActual.pos * RadToDeg << ','
             << finalActual.vel << ','
             << finalErrorDeg << ','
             << maxOvershootDeg << ','
             << maxAbsVel << ','
             << maxAbsTorque << ','
             << samplesWritten << ','
             << actualFailures << ','
             << timeoutMs << "\n";
    summary->flush();
}

bool runMitPreZero(DriverSDK::DriverSDK& sdk,
                   std::vector<DriverSDK::motorTargetStruct>& targets,
                   std::vector<DriverSDK::motorActualStruct>& actuals,
                   std::size_t index,
                   std::chrono::steady_clock::duration samplePeriod,
                   std::chrono::nanoseconds controlPeriod,
                   float kp,
                   float kd,
                   float targetToleranceRad,
                   float sensitiveMotionPosThresholdRad,
                   float sensitiveMotionVelThresholdRadS,
                   std::chrono::steady_clock::time_point testStart,
                   std::ofstream* samples,
                   std::ofstream* summary) {
    sdk.getMotorActual(actuals);
    float const startPos = actuals[index].pos;
    float const startVel = actuals[index].vel;
    auto finalActual = actuals[index];

    targets[index].pos = 0.0f;
    targets[index].vel = 0.0f;
    targets[index].tor = 0.0f;
    auto const commandTime = std::chrono::steady_clock::now();
    sdk.setMotorTarget(targets);

    std::cout << "MIT prezero target pos=0rad kp=" << kp << " kd=" << kd << " tor=0"
              << " start_pos=" << startPos << "rad (" << startPos * RadToDeg
              << "deg) start_vel=" << startVel << "rad/s\n";
    std::cout << "timing reference: command_time is before setMotorTarget(); true CAN tx can lag by up to about "
              << std::chrono::duration<double, std::milli>(controlPeriod).count()
              << "ms, and detection windows are limited by sample period\n";

    MotionWindow sensitiveMotion;
    MotionWindow visibleMotion;
    bool targetReached = false;
    double targetLowerMs = -1.0;
    double targetUpperMs = -1.0;
    double maxOvershootDeg = 0.0;
    double maxAbsVel = std::fabs(startVel);
    double maxAbsTorque = std::fabs(finalActual.tor);
    int samplesWritten = 0;
    int actualFailures = 0;
    auto lastSample = commandTime;
    auto nextSample = commandTime + samplePeriod;
    auto nextPrint = commandTime;
    auto const deadline = commandTime + std::chrono::milliseconds(MitPreZeroTimeoutMs);

    while (!stopRequested.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_until(nextSample);
        auto const sampleTime = std::chrono::steady_clock::now();
        nextSample += samplePeriod;

        if (sdk.getMotorActual(actuals) != 0) {
            ++actualFailures;
            writeSample(samples, "prezero", 0, 0, testStart, commandTime, sampleTime,
                        0.0f, 0.0f, startPos, finalActual,
                        sensitiveMotion.detected, visibleMotion.detected, targetReached, false);
            ++samplesWritten;
            lastSample = sampleTime;
            continue;
        }
        auto const& actual = actuals[index];
        finalActual = actual;
        float const moved = std::fabs(actual.pos - startPos);
        float const velocityChanged = std::fabs(actual.vel - startVel);
        float const targetError = std::fabs(actual.pos);
        maxAbsVel = std::max(maxAbsVel, static_cast<double>(std::fabs(actual.vel)));
        maxAbsTorque = std::max(maxAbsTorque, static_cast<double>(std::fabs(actual.tor)));
        maxOvershootDeg = std::max(maxOvershootDeg, static_cast<double>(std::fabs(actual.pos)) * RadToDeg);

        updateMotionWindow(sensitiveMotion, moved, velocityChanged,
                           sensitiveMotionPosThresholdRad, sensitiveMotionVelThresholdRadS,
                           "prezero_sensitive_motion_start", commandTime, lastSample, sampleTime);
        updateMotionWindow(visibleMotion, moved, velocityChanged,
                           VisibleMotionPositionThresholdRad, VisibleMotionVelocityThresholdRadS,
                           "prezero_motion_start", commandTime, lastSample, sampleTime);
        if (!targetReached && targetError <= targetToleranceRad) {
            targetReached = true;
            targetLowerMs = msBetween(commandTime, lastSample);
            targetUpperMs = msBetween(commandTime, sampleTime);
            printWindow("prezero_target_reached", commandTime, lastSample, sampleTime);
            writeSample(samples, "prezero", 0, 0, testStart, commandTime, sampleTime,
                        0.0f, 0.0f, startPos, actual,
                        sensitiveMotion.detected, visibleMotion.detected, targetReached, true);
            ++samplesWritten;
            writeSummary(summary, "prezero", 0, 0, 0.0f, 0.0f, startPos, startVel,
                         kp, kd, targetToleranceRad, 0,
                         sensitiveMotionPosThresholdRad, sensitiveMotionVelThresholdRadS,
                         sensitiveMotion, visibleMotion,
                         targetReached, targetLowerMs, targetUpperMs, finalActual,
                         maxOvershootDeg, maxAbsVel, maxAbsTorque, samplesWritten,
                         actualFailures, MitPreZeroTimeoutMs);
            return true;
        }

        if (sampleTime >= nextPrint) {
            std::cout << "prezero actual pos=" << std::fixed << std::setprecision(4) << actual.pos
                      << "rad (" << actual.pos * RadToDeg << "deg)"
                      << " vel=" << actual.vel
                      << "rad/s err=" << targetError * RadToDeg
                      << "deg tor=" << actual.tor
                      << " status=0x" << std::hex << actual.statusWord << std::dec
                      << " error=0x" << std::hex << actual.errorCode << std::dec << "\n";
            nextPrint = sampleTime + std::chrono::milliseconds(100);
        }

        writeSample(samples, "prezero", 0, 0, testStart, commandTime, sampleTime,
                    0.0f, 0.0f, startPos, actual,
                    sensitiveMotion.detected, visibleMotion.detected, targetReached, true);
        ++samplesWritten;
        if (samples != nullptr && samplesWritten % 200 == 0) {
            samples->flush();
        }
        lastSample = sampleTime;
    }

    std::cout << "prezero_target_reached not detected within " << MitPreZeroTimeoutMs
              << "ms; tolerance=" << targetToleranceRad * RadToDeg << "deg\n";
    writeSummary(summary, "prezero", 0, 0, 0.0f, 0.0f, startPos, startVel,
                 kp, kd, targetToleranceRad, 0,
                 sensitiveMotionPosThresholdRad, sensitiveMotionVelThresholdRadS,
                 sensitiveMotion, visibleMotion,
                 targetReached, targetLowerMs, targetUpperMs, finalActual,
                 maxOvershootDeg, maxAbsVel, maxAbsTorque, samplesWritten,
                 actualFailures, MitPreZeroTimeoutMs);
    return false;
}

void runMitStep(DriverSDK::DriverSDK& sdk,
                std::vector<DriverSDK::motorTargetStruct>& targets,
                std::vector<DriverSDK::motorActualStruct>& actuals,
                std::size_t index,
                float commandPos,
                float evalPos,
                int step,
                int cycle,
                std::chrono::steady_clock::duration samplePeriod,
                std::chrono::milliseconds holdTime,
                std::chrono::nanoseconds controlPeriod,
                float kp,
                float kd,
                float targetToleranceRad,
                int rampMs,
                float sensitiveMotionPosThresholdRad,
                float sensitiveMotionVelThresholdRadS,
                std::chrono::steady_clock::time_point testStart,
                std::ofstream* samples,
                std::ofstream* summary) {
    sdk.getMotorActual(actuals);
    float const startPos = actuals[index].pos;
    float const startVel = actuals[index].vel;
    float const startCommandPos = targets[index].pos;
    auto finalActual = actuals[index];

    auto const commandTime = std::chrono::steady_clock::now();
    if (rampMs <= 0) {
        targets[index].pos = commandPos;
        targets[index].vel = 0.0f;
        targets[index].tor = 0.0f;
        sdk.setMotorTarget(targets);
    }

    std::cout << "MIT command pos=" << commandPos << "rad (" << commandPos * RadToDeg
              << "deg) eval pos=" << evalPos << "rad (" << evalPos * RadToDeg
              << "deg) ramp_ms=" << rampMs << " kp=" << kp << " kd=" << kd << " tor=0 cycle=" << cycle
              << " start_pos=" << startPos << "rad start_vel=" << startVel << "rad/s\n";
    std::cout << "timing reference: command_time is before setMotorTarget(); true CAN tx can lag by up to about "
              << std::chrono::duration<double, std::milli>(controlPeriod).count()
              << "ms, and detection windows are limited by sample period\n";

    MotionWindow sensitiveMotion;
    MotionWindow visibleMotion;
    bool targetReached = false;
    double targetLowerMs = -1.0;
    double targetUpperMs = -1.0;
    double maxOvershootDeg = 0.0;
    double maxAbsVel = std::fabs(startVel);
    double maxAbsTorque = std::fabs(finalActual.tor);
    int samplesWritten = 0;
    int actualFailures = 0;
    auto lastSample = commandTime;
    auto nextSample = commandTime + samplePeriod;
    auto nextPrint = commandTime;
    auto const deadline = commandTime + holdTime;

    while (!stopRequested.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_until(nextSample);
        auto const sampleTime = std::chrono::steady_clock::now();
        nextSample += samplePeriod;
        float currentCommandPos = commandPos;
        if (rampMs > 0) {
            double const elapsedMs = msBetween(commandTime, sampleTime);
            double const progress = std::clamp(elapsedMs / static_cast<double>(rampMs), 0.0, 1.0);
            currentCommandPos = startCommandPos + static_cast<float>((commandPos - startCommandPos) * progress);
            targets[index].pos = currentCommandPos;
            targets[index].vel = 0.0f;
            targets[index].tor = 0.0f;
            sdk.setMotorTarget(targets);
        }

        if (sdk.getMotorActual(actuals) != 0) {
            ++actualFailures;
            writeSample(samples, "sweep", step, cycle, testStart, commandTime, sampleTime,
                        currentCommandPos, evalPos, startPos, finalActual,
                        sensitiveMotion.detected, visibleMotion.detected, targetReached, false);
            ++samplesWritten;
            lastSample = sampleTime;
            continue;
        }
        auto const& actual = actuals[index];
        finalActual = actual;
        float const moved = std::fabs(actual.pos - startPos);
        float const velocityChanged = std::fabs(actual.vel - startVel);
        float const targetError = std::fabs(actual.pos - evalPos);
        double const direction = evalPos >= startPos ? 1.0 : -1.0;
        double const overshootDeg = std::max(0.0, (static_cast<double>(actual.pos - evalPos) * direction) * RadToDeg);
        maxOvershootDeg = std::max(maxOvershootDeg, overshootDeg);
        maxAbsVel = std::max(maxAbsVel, static_cast<double>(std::fabs(actual.vel)));
        maxAbsTorque = std::max(maxAbsTorque, static_cast<double>(std::fabs(actual.tor)));

        updateMotionWindow(sensitiveMotion, moved, velocityChanged,
                           sensitiveMotionPosThresholdRad, sensitiveMotionVelThresholdRadS,
                           "sensitive_motion_start", commandTime, lastSample, sampleTime);
        updateMotionWindow(visibleMotion, moved, velocityChanged,
                           VisibleMotionPositionThresholdRad, VisibleMotionVelocityThresholdRadS,
                           "motion_start", commandTime, lastSample, sampleTime);
        if (!targetReached && targetError <= targetToleranceRad) {
            targetReached = true;
            targetLowerMs = msBetween(commandTime, lastSample);
            targetUpperMs = msBetween(commandTime, sampleTime);
            printWindow("target_reached", commandTime, lastSample, sampleTime);
        }

        if (sampleTime >= nextPrint) {
            std::cout << "actual pos=" << std::fixed << std::setprecision(4) << actual.pos
                      << "rad (" << actual.pos * RadToDeg << "deg)"
                      << " cmd=" << currentCommandPos * RadToDeg << "deg"
                      << " vel=" << actual.vel
                      << "rad/s err=" << targetError * RadToDeg
                      << "deg tor=" << actual.tor
                      << " status=0x" << std::hex << actual.statusWord << std::dec
                      << " error=0x" << std::hex << actual.errorCode << std::dec << "\n";
            nextPrint = sampleTime + std::chrono::milliseconds(100);
        }

        writeSample(samples, "sweep", step, cycle, testStart, commandTime, sampleTime,
                    currentCommandPos, evalPos, startPos, actual,
                    sensitiveMotion.detected, visibleMotion.detected, targetReached, true);
        ++samplesWritten;
        if (samples != nullptr && samplesWritten % 200 == 0) {
            samples->flush();
        }
        lastSample = sampleTime;
    }

    if (!sensitiveMotion.detected) {
        std::cout << "sensitive_motion_start not detected within " << holdTime.count()
                  << "ms; thresholds pos_delta=" << sensitiveMotionPosThresholdRad * RadToDeg
                  << "deg or vel_delta=" << sensitiveMotionVelThresholdRadS * RadToDeg << "deg/s\n";
    }
    if (!visibleMotion.detected) {
        std::cout << "motion_start not detected within " << holdTime.count()
                  << "ms; thresholds pos_delta=" << VisibleMotionPositionThresholdRad * RadToDeg
                  << "deg or vel_delta=" << VisibleMotionVelocityThresholdRadS * RadToDeg << "deg/s\n";
    }
    if (!targetReached) {
        std::cout << "target_reached not detected within " << holdTime.count()
                  << "ms; tolerance=" << targetToleranceRad * RadToDeg << "deg\n";
    }
    writeSummary(summary, "sweep", step, cycle, commandPos, evalPos, startPos, startVel,
                 kp, kd, targetToleranceRad, rampMs,
                 sensitiveMotionPosThresholdRad, sensitiveMotionVelThresholdRadS,
                 sensitiveMotion, visibleMotion,
                 targetReached, targetLowerMs, targetUpperMs, finalActual,
                 maxOvershootDeg, maxAbsVel, maxAbsTorque, samplesWritten,
                 actualFailures, static_cast<int>(holdTime.count()));
}

} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    std::string const configPath = argc > 1 ? argv[1] : "config/example_rmd_can.xml";
    int const zeroSpeedDps = argc > 2 ? std::atoi(argv[2]) : 0;
    int const cycles = argc > 3 ? std::atoi(argv[3]) : 4;
    int const holdMs = argc > 4 ? std::atoi(argv[4]) : 2500;
    int const sampleHz = argc > 5 ? std::atoi(argv[5]) : 1000;
    float const sweepDeg = argc > 6 ? std::atof(argv[6]) : RadToDeg;
    float const kp = argc > 7 ? std::atof(argv[7]) : 30.0f;
    float const kd = argc > 8 ? std::atof(argv[8]) : 2.0f;
    float const toleranceDeg = argc > 9 ? std::atof(argv[9]) : DefaultTargetToleranceDeg;
    float const preZeroToleranceDeg = argc > 10 ? std::atof(argv[10]) : DefaultPreZeroToleranceDeg;
    float evalDeg = sweepDeg;
    int outputPrefixArg = 11;
    if (argc > 11 && parseFloatStrict(argv[11], &evalDeg)) {
        outputPrefixArg = 12;
    }
    int rampMs = 0;
    if (argc > outputPrefixArg && parseIntStrict(argv[outputPrefixArg], &rampMs)) {
        ++outputPrefixArg;
    }
    float sensitiveMotionPosDeg = DefaultSensitiveMotionPositionThresholdDeg;
    float sensitiveMotionVelDegS = DefaultSensitiveMotionVelocityThresholdDegS;
    if (argc > outputPrefixArg && parseFloatStrict(argv[outputPrefixArg], &sensitiveMotionPosDeg)) {
        ++outputPrefixArg;
        if (argc > outputPrefixArg && parseFloatStrict(argv[outputPrefixArg], &sensitiveMotionVelDegS)) {
            ++outputPrefixArg;
        }
    }
    std::string const outputPrefix = argc > outputPrefixArg ? argv[outputPrefixArg] : defaultOutputPrefix();
    if (zeroSpeedDps < 0 || zeroSpeedDps > MaxA4SpeedDps) {
        std::cerr << "zero_speed_dps must be in 0.." << MaxA4SpeedDps << "; use 0 to skip A4 zeroing\n";
        return 2;
    }
    if (sampleHz <= 0 || sampleHz > 2000) {
        std::cerr << "sample_hz must be in 1..2000\n";
        return 2;
    }
    if (sweepDeg <= 0.0f || sweepDeg > 180.0f) {
        std::cerr << "sweep_deg must be in 0..180\n";
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
    if (toleranceDeg <= 0.0f || toleranceDeg > 180.0f) {
        std::cerr << "tolerance_deg must be in 0..180\n";
        return 2;
    }
    if (preZeroToleranceDeg <= 0.0f || preZeroToleranceDeg > 180.0f) {
        std::cerr << "prezero_tolerance_deg must be in 0..180\n";
        return 2;
    }
    if (evalDeg <= 0.0f || evalDeg > 180.0f) {
        std::cerr << "eval_deg must be in 0..180\n";
        return 2;
    }
    if (rampMs < 0 || rampMs > holdMs) {
        std::cerr << "ramp_ms must be in 0..hold_ms\n";
        return 2;
    }
    if (sensitiveMotionPosDeg <= 0.0f || sensitiveMotionPosDeg > VisibleMotionPositionThresholdRad * RadToDeg) {
        std::cerr << "sensitive_motion_pos_deg must be in 0.." << VisibleMotionPositionThresholdRad * RadToDeg << "\n";
        return 2;
    }
    if (sensitiveMotionVelDegS <= 0.0f || sensitiveMotionVelDegS > VisibleMotionVelocityThresholdRadS * RadToDeg) {
        std::cerr << "sensitive_motion_vel_deg_s must be in 0.." << VisibleMotionVelocityThresholdRadS * RadToDeg << "\n";
        return 2;
    }
    float const targetToleranceRad = toleranceDeg * DegToRad;
    float const preZeroToleranceRad = preZeroToleranceDeg * DegToRad;
    float const sensitiveMotionPosThresholdRad = sensitiveMotionPosDeg * DegToRad;
    float const sensitiveMotionVelThresholdRadS = sensitiveMotionVelDegS * DegToRad;

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
    writeSummaryHeader(summary);

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

    std::cout << "manual test config=" << configPath
              << " can=" << masterIt->device
              << " motor_id=" << motor.motorId
              << " alias=" << motor.alias
              << " zero_speed=" << zeroSpeedDps
              << "dps cycles=" << cycles
              << " hold_ms=" << holdMs
              << " sample_hz=" << sampleHz
              << " command_deg=" << sweepDeg
              << " eval_deg=" << evalDeg
              << " ramp_ms=" << rampMs
              << " kp=" << kp
              << " kd=" << kd
              << " tolerance_deg=" << toleranceDeg
              << " prezero_tolerance_deg=" << preZeroToleranceDeg
              << " sensitive_motion_pos_deg=" << sensitiveMotionPosDeg
              << " sensitive_motion_vel_deg_s=" << sensitiveMotionVelDegS << "\n";
    std::cout << "writing samples: " << samplesPath << "\n";
    std::cout << "writing summary: " << summaryPath << "\n";

    if (zeroSpeedDps > 0) {
        std::cout << "running optional A4 zero stage before MIT because zero_speed_dps>0\n";
        if (!moveToZero(masterIt->device, motor.motorId, zeroSpeedDps)) {
            return 1;
        }
    } else {
        std::cout << "skipping optional A4 zero stage because zero_speed_dps=0\n";
    }
    if (stopRequested.load(std::memory_order_acquire)) {
        return 1;
    }

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
    targets[index].vel = 0.0f;
    targets[index].kp = kp;
    targets[index].kd = kd;
    targets[index].tor = 0.0f;
    targets[index].pos = 0.0f;

    auto const samplePeriod =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1.0 / sampleHz));
    auto const controlPeriod = std::chrono::nanoseconds(config.periodNs * std::max(1, masterIt->division));
    std::cout << "measurement thresholds: sensitive motion pos_delta>=" << sensitiveMotionPosDeg
              << "deg or vel_delta>=" << sensitiveMotionVelDegS
              << "deg/s; visible motion pos_delta>=1deg or vel_delta>=5deg/s, target tolerance="
              << toleranceDeg << "deg around eval target, prezero tolerance=" << preZeroToleranceDeg << "deg\n";
    std::cout << "sample_period_ms=" << std::chrono::duration<double, std::milli>(samplePeriod).count()
              << " control_period_ms=" << std::chrono::duration<double, std::milli>(controlPeriod).count() << "\n";

    auto const testStart = std::chrono::steady_clock::now();
    if (!runMitPreZero(sdk, targets, actuals, index, samplePeriod, controlPeriod,
                       kp, kd, preZeroToleranceRad,
                       sensitiveMotionPosThresholdRad, sensitiveMotionVelThresholdRadS,
                       testStart, &samples, &summary)) {
        disableMit(sdk, targets);
        return 1;
    }

    float const commandTargetRad = sweepDeg * DegToRad;
    float const evalTargetRad = evalDeg * DegToRad;
    int step = 0;
    for (int cycle = 0; cycle < cycles && !stopRequested.load(std::memory_order_acquire); ++cycle) {
        for (float sign : {1.0f, -1.0f}) {
            ++step;
            runMitStep(sdk, targets, actuals, index, sign * commandTargetRad, sign * evalTargetRad, step, cycle + 1, samplePeriod,
                       std::chrono::milliseconds(holdMs), controlPeriod,
                       kp, kd, targetToleranceRad, rampMs,
                       sensitiveMotionPosThresholdRad, sensitiveMotionVelThresholdRadS,
                       testStart, &samples, &summary);
        }
    }

    targets[index].pos = 0.0f;
    targets[index].vel = 0.0f;
    targets[index].tor = 0.0f;
    sdk.setMotorTarget(targets);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    disableMit(sdk, targets);
    return stopRequested.load(std::memory_order_acquire) ? 1 : 0;
}
