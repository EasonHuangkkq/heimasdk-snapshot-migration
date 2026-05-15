#include "rmd_can_sdk/heima_driver_sdk.h"
#include "rmd_can_sdk/rmd_bench_workflow.h"
#include "rmd_can_sdk/rmd_can_config.h"
#include "rmd_can_sdk/rmd_joint_path.h"
#include "rmd_can_sdk/rmd_leg_kinematics.h"

#include <algorithm>
#include <chrono>
#include <cmath>
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

void requestStop(int) {
    gStopRequested = 1;
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

void printUsage(char const* program) {
    std::cerr << "usage: " << program
              << " <config.xml> <duration_ms> <max_current> <settle_samples> <period_ms> <ramp_ms>"
              << " right|both [demo|ankle_pitch_swing|ankle_pitch_swing_repeat|"
                 "ankle_pitch_swing_fast_repeat|ankle_roll_swing_fast_repeat|"
                 "knee_ankle_swing_repeat|knee_ankle_swing_slow_repeat|"
                 "knee_pitch_swing_repeat|knee_pitch_swing_deep_repeat|"
                 "knee_pitch_range_fast_repeat|knee_pitch_range_faster_repeat|"
                 "squat|squat_repeat|squat_slow_repeat]\n";
    std::cerr << "example: " << program
              << " config/configOriginal_heima.xml 10000 500 450 5 8000 right\n";
    std::cerr << "audit: " << program << " --audit-self-test\n";
    std::cerr << "audit ankle swing: " << program << " --audit-ankle-pitch-swing\n";
    std::cerr << "audit ankle repeat: " << program << " --audit-ankle-pitch-repeat\n";
    std::cerr << "audit ankle pitch fast repeat: " << program << " --audit-ankle-pitch-fast-repeat\n";
    std::cerr << "audit ankle roll fast repeat: " << program << " --audit-ankle-roll-fast-repeat\n";
    std::cerr << "audit knee ankle both repeat: " << program << " --audit-knee-ankle-both-repeat\n";
    std::cerr << "audit knee ankle both slow repeat: " << program << " --audit-knee-ankle-both-slow-repeat\n";
    std::cerr << "audit knee pitch repeat: " << program << " --audit-knee-pitch-repeat\n";
    std::cerr << "audit knee pitch range fast repeat: " << program << " --audit-knee-pitch-range-fast-repeat\n";
    std::cerr << "audit knee pitch range faster repeat: " << program << " --audit-knee-pitch-range-faster-repeat\n";
    std::cerr << "audit squat repeat: " << program << " --audit-squat-repeat\n";
    std::cerr << "env: RMD_ECAT_RT_CPU/RMD_ECAT_RT_PRIORITY configure the EtherCAT backend; "
                 "RMD_ECAT_APP_CPU/RMD_ECAT_APP_RT_PRIORITY configure this app loop\n";
}

std::vector<RmdCanSdk::LegJointWaypoint> makeDemoPath(int durationMs) {
    int const midMs = durationMs / 2;
    return std::vector<RmdCanSdk::LegJointWaypoint>{
        RmdCanSdk::LegJointWaypoint{0, RmdCanSdk::LegJointTargets{0.35, -0.70, 0.35, 0.0}},
        RmdCanSdk::LegJointWaypoint{midMs, RmdCanSdk::LegJointTargets{0.37, -0.68, 0.33, 0.02}},
        RmdCanSdk::LegJointWaypoint{durationMs, RmdCanSdk::LegJointTargets{0.35, -0.70, 0.35, 0.0}},
    };
}

std::vector<RmdCanSdk::LegJointWaypoint> makeAnklePitchSwingPath(int durationMs) {
    int const midMs = durationMs / 2;
    return std::vector<RmdCanSdk::LegJointWaypoint>{
        RmdCanSdk::LegJointWaypoint{0, RmdCanSdk::LegJointTargets{0.35, -0.70, 0.0, 0.0}},
        RmdCanSdk::LegJointWaypoint{midMs, RmdCanSdk::LegJointTargets{0.35, -0.70, 0.35, 0.0}},
        RmdCanSdk::LegJointWaypoint{durationMs, RmdCanSdk::LegJointTargets{0.35, -0.70, 0.0, 0.0}},
    };
}

std::vector<RmdCanSdk::LegJointWaypoint> makeAnkleRollSwingPath(int stepMs) {
    return std::vector<RmdCanSdk::LegJointWaypoint>{
        RmdCanSdk::LegJointWaypoint{0, RmdCanSdk::LegJointTargets{0.35, -0.70, 0.175, 0.0}},
        RmdCanSdk::LegJointWaypoint{stepMs, RmdCanSdk::LegJointTargets{0.35, -0.70, 0.175, 0.35}},
        RmdCanSdk::LegJointWaypoint{2 * stepMs, RmdCanSdk::LegJointTargets{0.35, -0.70, 0.175, -0.35}},
        RmdCanSdk::LegJointWaypoint{3 * stepMs, RmdCanSdk::LegJointTargets{0.35, -0.70, 0.175, 0.0}},
    };
}

std::vector<RmdCanSdk::LegJointWaypoint> makeKneeAnkleSwingPath(int durationMs) {
    int const midMs = durationMs / 2;
    return std::vector<RmdCanSdk::LegJointWaypoint>{
        RmdCanSdk::LegJointWaypoint{0, RmdCanSdk::LegJointTargets{0.35, -1.0, 0.0, 0.0}},
        RmdCanSdk::LegJointWaypoint{midMs, RmdCanSdk::LegJointTargets{0.35, 0.0, 0.35, 0.0}},
        RmdCanSdk::LegJointWaypoint{durationMs, RmdCanSdk::LegJointTargets{0.35, -1.0, 0.0, 0.0}},
    };
}

std::vector<RmdCanSdk::LegJointWaypoint> makeKneePitchSwingPath(int durationMs, double deepKneePitchRad) {
    int const midMs = durationMs / 2;
    return std::vector<RmdCanSdk::LegJointWaypoint>{
        RmdCanSdk::LegJointWaypoint{0, RmdCanSdk::LegJointTargets{0.35, -0.70, 0.35, 0.0}},
        RmdCanSdk::LegJointWaypoint{midMs, RmdCanSdk::LegJointTargets{0.35, deepKneePitchRad, 0.35, 0.0}},
        RmdCanSdk::LegJointWaypoint{durationMs, RmdCanSdk::LegJointTargets{0.35, -0.70, 0.35, 0.0}},
    };
}

std::vector<RmdCanSdk::LegJointWaypoint> makeKneePitchRangePath(int stepMs) {
    std::vector<double> const kneeNodes{0.0, -0.30, -0.60, -0.90, -1.20, -0.90, -0.60, -0.30, 0.0};
    std::vector<RmdCanSdk::LegJointWaypoint> path;
    path.reserve(kneeNodes.size());
    for (std::size_t i = 0; i < kneeNodes.size(); ++i) {
        path.push_back(RmdCanSdk::LegJointWaypoint{
            static_cast<int>(i) * stepMs,
            RmdCanSdk::LegJointTargets{0.35, kneeNodes[i], 0.35, 0.0},
        });
    }
    return path;
}

std::vector<RmdCanSdk::LegJointWaypoint> makeSquatPath(int durationMs) {
    int const midMs = durationMs / 2;
    return std::vector<RmdCanSdk::LegJointWaypoint>{
        RmdCanSdk::LegJointWaypoint{0, RmdCanSdk::LegJointTargets{0.35, -0.70, 0.35, 0.0}},
        RmdCanSdk::LegJointWaypoint{midMs, RmdCanSdk::LegJointTargets{0.50, -1.00, 0.38, 0.0}},
        RmdCanSdk::LegJointWaypoint{durationMs, RmdCanSdk::LegJointTargets{0.35, -0.70, 0.35, 0.0}},
    };
}

std::vector<RmdCanSdk::LegJointWaypoint> makePathForMode(std::string const& mode, int durationMs) {
    if (mode == "demo") {
        return makeDemoPath(durationMs);
    }
    if (mode == "ankle_pitch_swing") {
        return makeAnklePitchSwingPath(durationMs);
    }
    if (mode == "ankle_pitch_swing_repeat") {
        return makeAnklePitchSwingPath(4000);
    }
    if (mode == "ankle_pitch_swing_fast_repeat") {
        return makeAnklePitchSwingPath(1600);
    }
    if (mode == "ankle_roll_swing_fast_repeat") {
        return makeAnkleRollSwingPath(800);
    }
    if (mode == "knee_ankle_swing_repeat") {
        return makeKneeAnkleSwingPath(8000);
    }
    if (mode == "knee_ankle_swing_slow_repeat") {
        return makeKneeAnkleSwingPath(16000);
    }
    if (mode == "knee_pitch_swing_repeat") {
        return makeKneePitchSwingPath(8000, -0.90);
    }
    if (mode == "knee_pitch_swing_deep_repeat") {
        return makeKneePitchSwingPath(10000, -1.00);
    }
    if (mode == "knee_pitch_range_fast_repeat") {
        return makeKneePitchRangePath(500);
    }
    if (mode == "knee_pitch_range_faster_repeat") {
        return makeKneePitchRangePath(300);
    }
    if (mode == "squat") {
        return makeSquatPath(durationMs);
    }
    if (mode == "squat_repeat") {
        return makeSquatPath(12000);
    }
    if (mode == "squat_slow_repeat") {
        return makeSquatPath(20000);
    }
    throw std::invalid_argument("unknown joint path mode: " + mode);
}

bool repeatsPath(std::string const& mode) {
    return mode == "ankle_pitch_swing_repeat" || mode == "ankle_pitch_swing_fast_repeat" ||
           mode == "ankle_roll_swing_fast_repeat" || mode == "knee_ankle_swing_repeat" ||
           mode == "knee_ankle_swing_slow_repeat" || mode == "knee_pitch_swing_repeat" ||
           mode == "knee_pitch_swing_deep_repeat" || mode == "knee_pitch_range_fast_repeat" ||
           mode == "knee_pitch_range_faster_repeat" || mode == "squat_repeat" || mode == "squat_slow_repeat";
}

int pathDurationMs(std::vector<RmdCanSdk::LegJointWaypoint> const& path) {
    return path.empty() ? 0 : path.back().timeMs;
}

int sampleTimeForMode(std::string const& mode,
                      std::vector<RmdCanSdk::LegJointWaypoint> const& path,
                      int elapsedMs) {
    if (!repeatsPath(mode)) {
        return elapsedMs;
    }
    int const cycleMs = pathDurationMs(path);
    if (cycleMs <= 0) {
        return 0;
    }
    return elapsedMs % cycleMs;
}

void printResolvedMotors(char const* prefix,
                         int timeMs,
                         RmdCanSdk::LegJointTargets const& joints,
                         RmdCanSdk::LegMotorTargets const& motors) {
    std::cout << std::fixed << std::setprecision(6)
              << prefix << " time_ms=" << timeMs
              << " hip_pitch=" << joints.hipPitchRad
              << " knee_pitch=" << joints.kneePitchRad
              << " ankle_pitch=" << joints.anklePitchRad
              << " ankle_roll=" << joints.ankleRollRad
              << " motor3=" << motors.hipPitchMotorRad
              << " motor4=" << motors.kneeMotorRad
              << " motor5=" << motors.ankleMotorERad
              << " motor6=" << motors.ankleMotorFRad
              << "\n";
}

void printResolvedBothLegMotors(char const* prefix,
                                int timeMs,
                                RmdCanSdk::LegJointTargets const& joints,
                                RmdCanSdk::LegMotorTargets const& motors) {
    std::cout << std::fixed << std::setprecision(6)
              << prefix << " time_ms=" << timeMs
              << " hip_pitch=" << joints.hipPitchRad
              << " knee_pitch=" << joints.kneePitchRad
              << " ankle_pitch=" << joints.anklePitchRad
              << " ankle_roll=" << joints.ankleRollRad
              << " right_motor3=" << motors.hipPitchMotorRad
              << " right_motor4=" << motors.kneeMotorRad
              << " right_motor5=" << motors.ankleMotorERad
              << " right_motor6=" << motors.ankleMotorFRad
              << " left_motor9=" << motors.hipPitchMotorRad
              << " left_motor10=" << motors.kneeMotorRad
              << " left_motor11=" << motors.ankleMotorERad
              << " left_motor12=" << motors.ankleMotorFRad
              << "\n";
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

bool assignMotorTarget(RmdCanSdk::Config const& config,
                       std::vector<float>& requestedTargets,
                       std::vector<bool>& hasRequestedTarget,
                       int motorAlias,
                       double position) {
    int const motorIndex = motorAlias - 1;
    if (motorAlias <= 0 || motorIndex >= config.totalMotorCount) {
        std::cerr << "motor alias outside configured motor count: " << motorAlias << "\n";
        return false;
    }
    RmdCanSdk::MotorParameters const* params = findParamsForAlias(config, motorAlias);
    if (params == nullptr) {
        std::cerr << "missing motor parameters for alias " << motorAlias << "\n";
        return false;
    }
    if (position < params->minimumPosition || position > params->maximumPosition) {
        std::cerr << "target for motor " << motorAlias
                  << " outside configured position limits ["
                  << params->minimumPosition << ", " << params->maximumPosition << "]\n";
        return false;
    }
    requestedTargets[static_cast<std::size_t>(motorIndex)] = static_cast<float>(position);
    hasRequestedTarget[static_cast<std::size_t>(motorIndex)] = true;
    return true;
}

bool assignResolvedRightLegTargets(RmdCanSdk::Config const& config,
                                   RmdCanSdk::LegMotorTargets const& motors,
                                   std::vector<float>& requestedTargets,
                                   std::vector<bool>& hasRequestedTarget) {
    return assignMotorTarget(config, requestedTargets, hasRequestedTarget, 3, motors.hipPitchMotorRad) &&
           assignMotorTarget(config, requestedTargets, hasRequestedTarget, 4, motors.kneeMotorRad) &&
           assignMotorTarget(config, requestedTargets, hasRequestedTarget, 5, motors.ankleMotorERad) &&
           assignMotorTarget(config, requestedTargets, hasRequestedTarget, 6, motors.ankleMotorFRad);
}

bool assignResolvedLeftLegTargets(RmdCanSdk::Config const& config,
                                  RmdCanSdk::LegMotorTargets const& motors,
                                  std::vector<float>& requestedTargets,
                                  std::vector<bool>& hasRequestedTarget) {
    return assignMotorTarget(config, requestedTargets, hasRequestedTarget, 9, motors.hipPitchMotorRad) &&
           assignMotorTarget(config, requestedTargets, hasRequestedTarget, 10, motors.kneeMotorRad) &&
           assignMotorTarget(config, requestedTargets, hasRequestedTarget, 11, motors.ankleMotorERad) &&
           assignMotorTarget(config, requestedTargets, hasRequestedTarget, 12, motors.ankleMotorFRad);
}

bool assignResolvedTargetsForSide(RmdCanSdk::Config const& config,
                                  std::string const& side,
                                  RmdCanSdk::LegMotorTargets const& motors,
                                  std::vector<float>& requestedTargets,
                                  std::vector<bool>& hasRequestedTarget) {
    if (side == "right") {
        return assignResolvedRightLegTargets(config, motors, requestedTargets, hasRequestedTarget);
    }
    if (side == "both") {
        return assignResolvedRightLegTargets(config, motors, requestedTargets, hasRequestedTarget) &&
               assignResolvedLeftLegTargets(config, motors, requestedTargets, hasRequestedTarget);
    }
    std::cerr << "unsupported side: " << side << "\n";
    return false;
}

bool validateResolvedPathLimits(RmdCanSdk::Config const& config,
                                std::string const& side,
                                std::string const& mode,
                                std::vector<RmdCanSdk::LegJointWaypoint> const& path,
                                int durationMs,
                                int periodMs) {
    std::vector<float> requestedTargets(static_cast<std::size_t>(config.totalMotorCount), 0.0f);
    std::vector<bool> hasRequestedTarget(static_cast<std::size_t>(config.totalMotorCount), false);
    for (int t = 0; t <= durationMs; t += periodMs) {
        std::fill(hasRequestedTarget.begin(), hasRequestedTarget.end(), false);
        int const sampleTime = sampleTimeForMode(mode, path, t);
        RmdCanSdk::LegJointTargets const joints = RmdCanSdk::sampleLegJointPath(path, sampleTime);
        RmdCanSdk::LegMotorTargets const motors = RmdCanSdk::solveRightLegMotorsFromJoints(joints);
        if (!assignResolvedTargetsForSide(config, side, motors, requestedTargets, hasRequestedTarget)) {
            std::cerr << "resolved path limit check failed at time_ms=" << t << "\n";
            return false;
        }
    }
    return true;
}

int runAuditSelfTest() {
    auto const path = makeDemoPath(2000);
    std::string error;
    if (!RmdCanSdk::validateLegJointPath(path, &error)) {
        std::cerr << "joint path audit failed: " << error << "\n";
        return 1;
    }
    for (int t : {0, 1000, 2000}) {
        RmdCanSdk::LegJointTargets const joints = RmdCanSdk::sampleLegJointPath(path, t);
        RmdCanSdk::LegMotorTargets const motors = RmdCanSdk::solveRightLegMotorsFromJoints(joints);
        printResolvedMotors("audit", t, joints, motors);
    }
    std::cout << "joint path demo audit passed\n";
    return 0;
}

int runAnklePitchSwingAudit() {
    auto const path = makeAnklePitchSwingPath(1000);
    std::string error;
    if (!RmdCanSdk::validateLegJointPath(path, &error)) {
        std::cerr << "ankle pitch swing audit failed: " << error << "\n";
        return 1;
    }
    for (int t : {0, 500, 1000}) {
        RmdCanSdk::LegJointTargets const joints = RmdCanSdk::sampleLegJointPath(path, t);
        RmdCanSdk::LegMotorTargets const motors = RmdCanSdk::solveRightLegMotorsFromJoints(joints);
        printResolvedMotors("ankle_swing", t, joints, motors);
    }
    std::cout << "ankle pitch swing audit passed\n";
    return 0;
}

int runAnklePitchRepeatAudit() {
    std::string const mode = "ankle_pitch_swing_repeat";
    auto const path = makePathForMode(mode, 120000);
    std::string error;
    if (!RmdCanSdk::validateLegJointPath(path, &error)) {
        std::cerr << "ankle pitch repeat audit failed: " << error << "\n";
        return 1;
    }
    for (int totalTime : {0, 2000, 4000, 6000, 119995}) {
        int const sampleTime = sampleTimeForMode(mode, path, totalTime);
        RmdCanSdk::LegJointTargets const joints = RmdCanSdk::sampleLegJointPath(path, sampleTime);
        RmdCanSdk::LegMotorTargets const motors = RmdCanSdk::solveRightLegMotorsFromJoints(joints);
        printResolvedMotors("ankle_repeat", sampleTime, joints, motors);
    }
    std::cout << "ankle pitch repeat audit passed\n";
    return 0;
}

int runAnklePitchFastRepeatAudit() {
    std::string const mode = "ankle_pitch_swing_fast_repeat";
    auto const path = makePathForMode(mode, 120000);
    std::string error;
    if (!RmdCanSdk::validateLegJointPath(path, &error)) {
        std::cerr << "ankle pitch fast repeat audit failed: " << error << "\n";
        return 1;
    }
    for (int totalTime : {0, 400, 800, 1200, 1600, 119995}) {
        int const sampleTime = sampleTimeForMode(mode, path, totalTime);
        RmdCanSdk::LegJointTargets const joints = RmdCanSdk::sampleLegJointPath(path, sampleTime);
        RmdCanSdk::LegMotorTargets const motors = RmdCanSdk::solveRightLegMotorsFromJoints(joints);
        printResolvedBothLegMotors("ankle_pitch_fast", sampleTime, joints, motors);
    }
    std::cout << "ankle pitch fast repeat audit passed\n";
    return 0;
}

int runAnkleRollFastRepeatAudit() {
    std::string const mode = "ankle_roll_swing_fast_repeat";
    auto const path = makePathForMode(mode, 120000);
    std::string error;
    if (!RmdCanSdk::validateLegJointPath(path, &error)) {
        std::cerr << "ankle roll fast repeat audit failed: " << error << "\n";
        return 1;
    }
    for (int totalTime : {0, 400, 800, 1200, 1600, 2000, 2400, 119995}) {
        int const sampleTime = sampleTimeForMode(mode, path, totalTime);
        RmdCanSdk::LegJointTargets const joints = RmdCanSdk::sampleLegJointPath(path, sampleTime);
        RmdCanSdk::LegMotorTargets const motors = RmdCanSdk::solveRightLegMotorsFromJoints(joints);
        printResolvedBothLegMotors("ankle_roll_fast", sampleTime, joints, motors);
    }
    std::cout << "ankle roll fast repeat audit passed\n";
    return 0;
}

int runKneeAnkleBothRepeatAudit() {
    std::string const mode = "knee_ankle_swing_repeat";
    auto const path = makePathForMode(mode, 120000);
    std::string error;
    if (!RmdCanSdk::validateLegJointPath(path, &error)) {
        std::cerr << "knee ankle both repeat audit failed: " << error << "\n";
        return 1;
    }
    for (int totalTime : {0, 2000, 4000, 6000, 8000, 119995}) {
        int const sampleTime = sampleTimeForMode(mode, path, totalTime);
        RmdCanSdk::LegJointTargets const joints = RmdCanSdk::sampleLegJointPath(path, sampleTime);
        RmdCanSdk::LegMotorTargets const motors = RmdCanSdk::solveRightLegMotorsFromJoints(joints);
        printResolvedBothLegMotors("knee_ankle_both", sampleTime, joints, motors);
    }
    std::cout << "knee ankle both repeat audit passed\n";
    return 0;
}

int runKneeAnkleBothSlowRepeatAudit() {
    std::string const mode = "knee_ankle_swing_slow_repeat";
    auto const path = makePathForMode(mode, 120000);
    std::string error;
    if (!RmdCanSdk::validateLegJointPath(path, &error)) {
        std::cerr << "knee ankle both slow repeat audit failed: " << error << "\n";
        return 1;
    }
    for (int totalTime : {0, 4000, 8000, 12000, 16000, 119995}) {
        int const sampleTime = sampleTimeForMode(mode, path, totalTime);
        RmdCanSdk::LegJointTargets const joints = RmdCanSdk::sampleLegJointPath(path, sampleTime);
        RmdCanSdk::LegMotorTargets const motors = RmdCanSdk::solveRightLegMotorsFromJoints(joints);
        printResolvedBothLegMotors("knee_ankle_both_slow", sampleTime, joints, motors);
    }
    std::cout << "knee ankle both slow repeat audit passed\n";
    return 0;
}

int runKneePitchRepeatAudit() {
    std::string const mode = "knee_pitch_swing_repeat";
    auto const path = makePathForMode(mode, 120000);
    std::string error;
    if (!RmdCanSdk::validateLegJointPath(path, &error)) {
        std::cerr << "knee pitch repeat audit failed: " << error << "\n";
        return 1;
    }
    for (int totalTime : {0, 2000, 4000, 6000, 8000, 119995}) {
        int const sampleTime = sampleTimeForMode(mode, path, totalTime);
        RmdCanSdk::LegJointTargets const joints = RmdCanSdk::sampleLegJointPath(path, sampleTime);
        RmdCanSdk::LegMotorTargets const motors = RmdCanSdk::solveRightLegMotorsFromJoints(joints);
        printResolvedBothLegMotors("knee_pitch_repeat", sampleTime, joints, motors);
    }
    std::cout << "knee pitch repeat audit passed\n";
    return 0;
}

int runKneePitchRangeFastRepeatAudit() {
    std::string const mode = "knee_pitch_range_fast_repeat";
    auto const path = makePathForMode(mode, 120000);
    std::string error;
    if (!RmdCanSdk::validateLegJointPath(path, &error)) {
        std::cerr << "knee pitch range fast repeat audit failed: " << error << "\n";
        return 1;
    }
    for (int totalTime : {0, 500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 119995}) {
        int const sampleTime = sampleTimeForMode(mode, path, totalTime);
        RmdCanSdk::LegJointTargets const joints = RmdCanSdk::sampleLegJointPath(path, sampleTime);
        RmdCanSdk::LegMotorTargets const motors = RmdCanSdk::solveRightLegMotorsFromJoints(joints);
        printResolvedBothLegMotors("knee_pitch_range_fast", sampleTime, joints, motors);
    }
    std::cout << "knee pitch range fast repeat audit passed\n";
    return 0;
}

int runKneePitchRangeFasterRepeatAudit() {
    std::string const mode = "knee_pitch_range_faster_repeat";
    auto const path = makePathForMode(mode, 120000);
    std::string error;
    if (!RmdCanSdk::validateLegJointPath(path, &error)) {
        std::cerr << "knee pitch range faster repeat audit failed: " << error << "\n";
        return 1;
    }
    for (int totalTime : {0, 300, 600, 900, 1200, 1500, 1800, 2100, 2400, 119995}) {
        int const sampleTime = sampleTimeForMode(mode, path, totalTime);
        RmdCanSdk::LegJointTargets const joints = RmdCanSdk::sampleLegJointPath(path, sampleTime);
        RmdCanSdk::LegMotorTargets const motors = RmdCanSdk::solveRightLegMotorsFromJoints(joints);
        printResolvedBothLegMotors("knee_pitch_range_faster", sampleTime, joints, motors);
    }
    std::cout << "knee pitch range faster repeat audit passed\n";
    return 0;
}

int runSquatRepeatAudit() {
    std::string const mode = "squat_repeat";
    auto const path = makePathForMode(mode, 120000);
    std::string error;
    if (!RmdCanSdk::validateLegJointPath(path, &error)) {
        std::cerr << "squat repeat audit failed: " << error << "\n";
        return 1;
    }
    for (int totalTime : {0, 3000, 6000, 9000, 12000, 119995}) {
        int const sampleTime = sampleTimeForMode(mode, path, totalTime);
        RmdCanSdk::LegJointTargets const joints = RmdCanSdk::sampleLegJointPath(path, sampleTime);
        RmdCanSdk::LegMotorTargets const motors = RmdCanSdk::solveRightLegMotorsFromJoints(joints);
        printResolvedBothLegMotors("squat_repeat", sampleTime, joints, motors);
    }
    std::cout << "squat repeat audit passed\n";
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);

    if (argc == 2 && std::string(argv[1]) == "--audit-self-test") {
        return runAuditSelfTest();
    }
    if (argc == 2 && std::string(argv[1]) == "--audit-ankle-pitch-swing") {
        return runAnklePitchSwingAudit();
    }
    if (argc == 2 && std::string(argv[1]) == "--audit-ankle-pitch-repeat") {
        return runAnklePitchRepeatAudit();
    }
    if (argc == 2 && std::string(argv[1]) == "--audit-ankle-pitch-fast-repeat") {
        return runAnklePitchFastRepeatAudit();
    }
    if (argc == 2 && std::string(argv[1]) == "--audit-ankle-roll-fast-repeat") {
        return runAnkleRollFastRepeatAudit();
    }
    if (argc == 2 && std::string(argv[1]) == "--audit-knee-ankle-both-repeat") {
        return runKneeAnkleBothRepeatAudit();
    }
    if (argc == 2 && std::string(argv[1]) == "--audit-knee-ankle-both-slow-repeat") {
        return runKneeAnkleBothSlowRepeatAudit();
    }
    if (argc == 2 && std::string(argv[1]) == "--audit-knee-pitch-repeat") {
        return runKneePitchRepeatAudit();
    }
    if (argc == 2 && std::string(argv[1]) == "--audit-knee-pitch-range-fast-repeat") {
        return runKneePitchRangeFastRepeatAudit();
    }
    if (argc == 2 && std::string(argv[1]) == "--audit-knee-pitch-range-faster-repeat") {
        return runKneePitchRangeFasterRepeatAudit();
    }
    if (argc == 2 && std::string(argv[1]) == "--audit-squat-repeat") {
        return runSquatRepeatAudit();
    }

    if (argc != 8 && argc != 9) {
        printUsage(argv[0]);
        return 2;
    }

    char const* configPath = argv[1];
    int const durationMs = parsePositive(argv[2], 10000);
    int const maxCurrent = parsePositive(argv[3], 500);
    int const settleSamples = parsePositive(argv[4], 450);
    int const periodMs = parsePositive(argv[5], 5);
    int const rampMs = parseNonNegative(argv[6], 8000);
    if (durationMs <= 0 || maxCurrent <= 0 || maxCurrent > 65535 || settleSamples <= 0 || periodMs <= 0 ||
        rampMs < 0) {
        std::cerr << "invalid numeric argument\n";
        return 2;
    }

    std::string const side = argv[7];
    if (side != "right" && side != "both") {
        std::cerr << "only right or both legs are supported in this demo\n";
        return 2;
    }
    std::string const mode = argc == 9 ? argv[8] : "demo";
    if (mode != "demo" && mode != "ankle_pitch_swing" && mode != "ankle_pitch_swing_repeat" &&
        mode != "ankle_pitch_swing_fast_repeat" && mode != "ankle_roll_swing_fast_repeat" &&
        mode != "knee_ankle_swing_repeat" && mode != "knee_ankle_swing_slow_repeat" &&
        mode != "knee_pitch_swing_repeat" && mode != "knee_pitch_swing_deep_repeat" &&
        mode != "knee_pitch_range_fast_repeat" && mode != "knee_pitch_range_faster_repeat" &&
        mode != "squat" && mode != "squat_repeat" && mode != "squat_slow_repeat") {
        std::cerr << "unknown joint path mode: " << mode << "\n";
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
        auto const path = makePathForMode(mode, durationMs);
        std::string error;
        if (!RmdCanSdk::validateLegJointPath(path, &error)) {
            std::cerr << "invalid demo path: " << error << "\n";
            return 2;
        }
        for (auto const& waypoint : path) {
            auto const motors = RmdCanSdk::solveRightLegMotorsFromJoints(waypoint.joints);
            if (side == "both") {
                printResolvedBothLegMotors("waypoint", waypoint.timeMs, waypoint.joints, motors);
            } else {
                printResolvedMotors("waypoint", waypoint.timeMs, waypoint.joints, motors);
            }
        }

        RmdCanSdk::Config const parsedConfig = RmdCanSdk::loadConfig(configPath);
        if (parsedConfig.totalMotorCount <= 0) {
            std::cerr << "config has no motors\n";
            return 2;
        }
        if (!validateResolvedPathLimits(parsedConfig, side, mode, path, durationMs, periodMs)) {
            return 2;
        }

        auto& sdk = DriverSDK::DriverSDK::instance();
        std::vector<char> modes(static_cast<std::size_t>(parsedConfig.totalMotorCount), static_cast<char>(8));
        std::vector<unsigned short> maxCurrents(static_cast<std::size_t>(parsedConfig.totalMotorCount),
                                                static_cast<unsigned short>(maxCurrent));
        sdk.setMode(modes);
        sdk.setMaxCurr(maxCurrents);
        sdk.init(configPath);
        if (appRealtimeRequested) {
            RmdCanSdk::applyRealtimeSettings(appRealtimeSettings, "app");
        }

        int const count = sdk.getTotalMotorNr();
        std::vector<DriverSDK::motorActualStruct> actuals(static_cast<std::size_t>(count));
        std::vector<DriverSDK::motorTargetStruct> targets(static_cast<std::size_t>(count));
        std::vector<float> startPositions(static_cast<std::size_t>(count), 0.0f);
        std::vector<float> currentResolvedTargets(static_cast<std::size_t>(count), 0.0f);
        std::vector<bool> hasPathTarget(static_cast<std::size_t>(count), false);

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

        RmdCanSdk::LegMotorTargets const firstMotors =
            RmdCanSdk::solveRightLegMotorsFromJoints(path.front().joints);
        if (!assignResolvedTargetsForSide(parsedConfig, side, firstMotors, currentResolvedTargets, hasPathTarget)) {
            return 2;
        }
        for (std::size_t i = 0; i < hasPathTarget.size(); ++i) {
            if (hasPathTarget[i] && !isActiveMotor(sdk, static_cast<int>(i))) {
                std::cerr << "requested motor " << i + 1 << " is not active in this backend\n";
                return 2;
            }
        }

        for (int active : sdk.getActiveMotors()) {
            std::size_t const index = static_cast<std::size_t>(active);
            startPositions[index] = actuals[index].pos;
            targets[index].pos = actuals[index].pos;
            targets[index].vel = 0.0f;
            targets[index].tor = 0.0f;
            targets[index].kp = 0.0f;
            targets[index].kd = 0.0f;
            targets[index].enabled = 1;
            std::cout << "joint path motor " << active + 1
                      << " start=" << startPositions[index]
                      << (hasPathTarget[index] ? " path-target" : " hold-current")
                      << " max_current=" << maxCurrent
                      << " ramp_ms=" << rampMs
                      << "\n";
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

        int const totalMs = rampMs + durationMs;
        int const iterations = std::max(1, totalMs / periodMs);
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

            int const loopMs = i * periodMs;
            int const pathMs = std::max(0, loopMs - rampMs);
            int const samplePathMs = sampleTimeForMode(mode, path, pathMs);
            RmdCanSdk::LegJointTargets const joints =
                loopMs < rampMs ? path.front().joints : RmdCanSdk::sampleLegJointPath(path, samplePathMs);
            RmdCanSdk::LegMotorTargets const motors = RmdCanSdk::solveRightLegMotorsFromJoints(joints);
            std::fill(hasPathTarget.begin(), hasPathTarget.end(), false);
            if (!assignResolvedTargetsForSide(parsedConfig, side, motors, currentResolvedTargets, hasPathTarget)) {
                disableMotors(sdk, targets);
                return 2;
            }
            float const rampProgress = smoothRampProgressForElapsed(loopMs, rampMs);
            for (int active : sdk.getActiveMotors()) {
                std::size_t const index = static_cast<std::size_t>(active);
                float const finalPosition = hasPathTarget[index] ? currentResolvedTargets[index] : startPositions[index];
                targets[index].pos = loopMs < rampMs
                                         ? startPositions[index] + (finalPosition - startPositions[index]) * rampProgress
                                         : finalPosition;
            }

            if (sdk.setMotorTarget(targets) != 0) {
                std::cerr << "setMotorTarget failed while following joint path\n";
                disableMotors(sdk, targets);
                return 1;
            }

            if ((i % std::max(1, 500 / periodMs)) == 0 || i + 1 == iterations) {
                int const actualStatus = sdk.getMotorActual(actuals);
                bool const ready = operationFeedbackReady(sdk, actuals, actualStatus);
                if (!ready) {
                    ++consecutiveMissedFeedback;
                } else {
                    consecutiveMissedFeedback = 0;
                }
                printActuals("path", i, sdk, actuals);
                if (side == "both") {
                    printResolvedBothLegMotors("path_target", samplePathMs, joints, motors);
                } else {
                    printResolvedMotors("path_target", samplePathMs, joints, motors);
                }
                if (consecutiveMissedFeedback >= MaxConsecutiveMissedFeedback) {
                    std::cerr << "operation-enabled feedback lost while following joint path for "
                              << consecutiveMissedFeedback << " consecutive monitor samples\n";
                    disableMotors(sdk, targets);
                    return 1;
                }
            }

            nextWake += std::chrono::milliseconds(periodMs);
            auto const beforeSleep = std::chrono::steady_clock::now();
            recordLoopOverrun(timingStats, std::max(0.0, elapsedMs(nextWake, beforeSleep)));
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
        std::cerr << "joint path demo failed: " << ex.what() << "\n";
        return 1;
    }

    return 0;
}
