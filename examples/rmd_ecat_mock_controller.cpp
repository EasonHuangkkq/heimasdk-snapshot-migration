#include "rmd_can_sdk/heima_driver_sdk.h"
#include "rmd_can_sdk/rmd_bench_workflow.h"
#include "rmd_can_sdk/rmd_can_config.h"
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
constexpr double Pi = 3.14159265358979323846;
constexpr char MockRlPolicyFlag[] = "--mock-rl-joint-policy";

using RmdCanSdk::applyRealtimeSettings;
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

struct RlLegObservation {
    RmdCanSdk::LegJointTargets q;
    RmdCanSdk::LegJointTargets dq;
};

struct RlObservation {
    RlLegObservation right;
    RlLegObservation left;
};

struct RlObservationState {
    bool hasPrevious = false;
    RmdCanSdk::LegJointTargets previousRight;
    RmdCanSdk::LegJointTargets previousLeft;
};

struct MockRlAction {
    RmdCanSdk::LegJointTargets right;
    RmdCanSdk::LegJointTargets left;
};

struct MockRlResolvedTargets {
    RmdCanSdk::LegMotorTargets right;
    RmdCanSdk::LegMotorTargets left;
};

void requestStop(int) {
    gStopRequested = 1;
}

void printUsage(char const* program) {
    std::cerr << "usage: " << program
              << " <config.xml> <duration_ms> <policy_hz> <max_current> <settle_samples>"
              << " <ramp_ms> <rt_priority> [--mock-rl-joint-policy|motor=position ...]\n";
    std::cerr << "example: " << program
              << " config/configOriginal_heima.xml 300000 100 500 450 8000 0"
              << " 3=0.25 4=-0.3 5=0.34 6=0.29 9=0.25 10=-0.3 11=0.34 12=0.29\n";
    std::cerr << "mock RL example: " << program
              << " config/configOriginal_heima.xml 10000 100 500 450 2000 60 "
              << MockRlPolicyFlag << "\n";
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

int parsePositive(char const* text) {
    if (text == nullptr || *text == '\0') {
        return -1;
    }
    char* end = nullptr;
    long const value = std::strtol(text, &end, 10);
    if (end == text || *end != '\0' || value <= 0 || value > 100000000L) {
        return -1;
    }
    return static_cast<int>(value);
}

int parseNonNegative(char const* text) {
    if (text == nullptr || *text == '\0') {
        return -1;
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
    if (end == text || *end != '\0' || !std::isfinite(value)) {
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

bool isMockRlControlledMotorAlias(int alias) {
    return alias == 3 || alias == 4 || alias == 5 || alias == 6 ||
           alias == 9 || alias == 10 || alias == 11 || alias == 12;
}

bool assignMotorTargetByAlias(RmdCanSdk::Config const& config,
                              std::vector<float>& targets,
                              std::vector<bool>& hasTarget,
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
        std::cerr << "mock RL target for motor " << motorAlias
                  << " outside configured position limits ["
                  << params->minimumPosition << ", " << params->maximumPosition << "]\n";
        return false;
    }

    targets[static_cast<std::size_t>(motorIndex)] = static_cast<float>(position);
    hasTarget[static_cast<std::size_t>(motorIndex)] = true;
    return true;
}

MockRlAction sampleMockRlAction(double policyTimeSeconds) {
    double const phase = 2.0 * Pi * 0.25 * policyTimeSeconds;
    double const wave = std::sin(phase);

    RmdCanSdk::LegJointTargets leg;
    leg.hipPitchRad = 0.35 + 0.10 * wave;
    leg.kneePitchRad = -0.70 + 0.22 * wave;
    leg.anklePitchRad = 0.35;
    leg.ankleRollRad = 0.0;
    return MockRlAction{leg, leg};
}

MockRlResolvedTargets resolveMockRlAction(MockRlAction const& action,
                                          MockRlResolvedTargets const& previous) {
    MockRlResolvedTargets resolved;
    resolved.right = RmdCanSdk::solveRightLegMotorsFromJoints(action.right, previous.right);
    resolved.left = RmdCanSdk::solveRightLegMotorsFromJoints(action.left, previous.left);
    return resolved;
}

bool assignMockRlResolvedTargets(RmdCanSdk::Config const& config,
                                 MockRlResolvedTargets const& resolved,
                                 std::vector<float>& targets,
                                 std::vector<bool>& hasTarget) {
    return assignMotorTargetByAlias(config, targets, hasTarget, 3, resolved.right.hipPitchMotorRad) &&
           assignMotorTargetByAlias(config, targets, hasTarget, 4, resolved.right.kneeMotorRad) &&
           assignMotorTargetByAlias(config, targets, hasTarget, 5, resolved.right.ankleMotorERad) &&
           assignMotorTargetByAlias(config, targets, hasTarget, 6, resolved.right.ankleMotorFRad) &&
           assignMotorTargetByAlias(config, targets, hasTarget, 9, resolved.left.hipPitchMotorRad) &&
           assignMotorTargetByAlias(config, targets, hasTarget, 10, resolved.left.kneeMotorRad) &&
           assignMotorTargetByAlias(config, targets, hasTarget, 11, resolved.left.ankleMotorERad) &&
           assignMotorTargetByAlias(config, targets, hasTarget, 12, resolved.left.ankleMotorFRad);
}

void assignMockRlResolvedTargetsUnchecked(MockRlResolvedTargets const& resolved,
                                          std::vector<float>& targets,
                                          std::vector<bool>& hasTarget) {
    targets[2] = static_cast<float>(resolved.right.hipPitchMotorRad);
    targets[3] = static_cast<float>(resolved.right.kneeMotorRad);
    targets[4] = static_cast<float>(resolved.right.ankleMotorERad);
    targets[5] = static_cast<float>(resolved.right.ankleMotorFRad);
    targets[8] = static_cast<float>(resolved.left.hipPitchMotorRad);
    targets[9] = static_cast<float>(resolved.left.kneeMotorRad);
    targets[10] = static_cast<float>(resolved.left.ankleMotorERad);
    targets[11] = static_cast<float>(resolved.left.ankleMotorFRad);
    hasTarget[2] = true;
    hasTarget[3] = true;
    hasTarget[4] = true;
    hasTarget[5] = true;
    hasTarget[8] = true;
    hasTarget[9] = true;
    hasTarget[10] = true;
    hasTarget[11] = true;
}

bool validateMockRlPolicyLimits(RmdCanSdk::Config const& config, int durationMs, int policyHz) {
    long long const iterationsRaw =
        (static_cast<long long>(durationMs) * static_cast<long long>(policyHz) + 999LL) / 1000LL;
    int const iterations = static_cast<int>(std::max(1LL, iterationsRaw));
    std::vector<float> targets(static_cast<std::size_t>(config.totalMotorCount), 0.0f);
    std::vector<bool> hasTarget(static_cast<std::size_t>(config.totalMotorCount), false);
    MockRlResolvedTargets previous;
    for (int i = 0; i < iterations; ++i) {
        MockRlAction const action = sampleMockRlAction(static_cast<double>(i) / policyHz);
        MockRlResolvedTargets const resolved = resolveMockRlAction(action, previous);
        std::fill(hasTarget.begin(), hasTarget.end(), false);
        if (!assignMockRlResolvedTargets(config, resolved, targets, hasTarget)) {
            std::cerr << "mock RL limit check failed at sample=" << i << "\n";
            return false;
        }
        previous = resolved;
    }
    return true;
}

bool imuNonZero(DriverSDK::imuStruct const& imu) {
    constexpr float Epsilon = 1.0e-6f;
    for (float value : imu.rpy) {
        if (std::fabs(value) > Epsilon) {
            return true;
        }
    }
    for (float value : imu.gyr) {
        if (std::fabs(value) > Epsilon) {
            return true;
        }
    }
    for (float value : imu.acc) {
        if (std::fabs(value) > Epsilon) {
            return true;
        }
    }
    return false;
}

bool near(double actual, double expected, double tolerance) {
    return std::fabs(actual - expected) <= tolerance;
}

RmdCanSdk::LegJointTargets initialLegHint(std::vector<DriverSDK::motorActualStruct> const& actuals,
                                          int hipAlias) {
    RmdCanSdk::LegJointTargets hint;
    hint.hipPitchRad = actuals[static_cast<std::size_t>(hipAlias - 1)].pos;
    hint.kneePitchRad = -0.70;
    hint.anklePitchRad = 0.0;
    hint.ankleRollRad = 0.0;
    return hint;
}

RmdCanSdk::LegJointTargets diffJoints(RmdCanSdk::LegJointTargets const& current,
                                      RmdCanSdk::LegJointTargets const& previous,
                                      double dtSeconds) {
    if (dtSeconds <= 0.0) {
        return {};
    }
    RmdCanSdk::LegJointTargets out;
    out.hipPitchRad = (current.hipPitchRad - previous.hipPitchRad) / dtSeconds;
    out.kneePitchRad = (current.kneePitchRad - previous.kneePitchRad) / dtSeconds;
    out.anklePitchRad = (current.anklePitchRad - previous.anklePitchRad) / dtSeconds;
    out.ankleRollRad = (current.ankleRollRad - previous.ankleRollRad) / dtSeconds;
    return out;
}

RlLegObservation buildLegObservation(std::vector<DriverSDK::motorActualStruct> const& actuals,
                                     int hipAlias,
                                     int kneeAlias,
                                     int ankleEAlias,
                                     int ankleFAlias,
                                     RmdCanSdk::LegJointTargets const& previous,
                                     double dtSeconds) {
    RlLegObservation out;
    out.q.hipPitchRad = actuals[static_cast<std::size_t>(hipAlias - 1)].pos;
    out.q.kneePitchRad = RmdCanSdk::solveKneeJointFromMotor(
        actuals[static_cast<std::size_t>(kneeAlias - 1)].pos,
        previous.kneePitchRad);
    RmdCanSdk::AnkleJointAngles const ankle = RmdCanSdk::solveAnkleJointsFromMotors(
        actuals[static_cast<std::size_t>(ankleEAlias - 1)].pos,
        actuals[static_cast<std::size_t>(ankleFAlias - 1)].pos,
        previous.anklePitchRad,
        previous.ankleRollRad);
    out.q.anklePitchRad = ankle.pitchRad;
    out.q.ankleRollRad = ankle.rollRad;
    out.dq = diffJoints(out.q, previous, dtSeconds);
    return out;
}

RlObservation buildRlObservation(std::vector<DriverSDK::motorActualStruct> const& actuals,
                                 RlObservationState& state,
                                 double dtSeconds) {
    RmdCanSdk::LegJointTargets const rightPrevious =
        state.hasPrevious ? state.previousRight : initialLegHint(actuals, 3);
    RmdCanSdk::LegJointTargets const leftPrevious =
        state.hasPrevious ? state.previousLeft : initialLegHint(actuals, 9);

    RlObservation out;
    out.right = buildLegObservation(actuals, 3, 4, 5, 6, rightPrevious, state.hasPrevious ? dtSeconds : 0.0);
    out.left = buildLegObservation(actuals, 9, 10, 11, 12, leftPrevious, state.hasPrevious ? dtSeconds : 0.0);
    state.previousRight = out.right.q;
    state.previousLeft = out.left.q;
    state.hasPrevious = true;
    return out;
}

void printRlObservation(int sample, RlObservation const& obs, bool imuValid) {
    std::cout << std::fixed << std::setprecision(6)
              << "rl_obs sample=" << sample
              << " r_hip_pitch=" << obs.right.q.hipPitchRad
              << " r_knee_pitch=" << obs.right.q.kneePitchRad
              << " r_ankle_pitch=" << obs.right.q.anklePitchRad
              << " r_ankle_roll=" << obs.right.q.ankleRollRad
              << " l_hip_pitch=" << obs.left.q.hipPitchRad
              << " l_knee_pitch=" << obs.left.q.kneePitchRad
              << " l_ankle_pitch=" << obs.left.q.anklePitchRad
              << " l_ankle_roll=" << obs.left.q.ankleRollRad
              << " r_hip_vel=" << obs.right.dq.hipPitchRad
              << " r_knee_vel=" << obs.right.dq.kneePitchRad
              << " r_ankle_pitch_vel=" << obs.right.dq.anklePitchRad
              << " r_ankle_roll_vel=" << obs.right.dq.ankleRollRad
              << " l_hip_vel=" << obs.left.dq.hipPitchRad
              << " l_knee_vel=" << obs.left.dq.kneePitchRad
              << " l_ankle_pitch_vel=" << obs.left.dq.anklePitchRad
              << " l_ankle_roll_vel=" << obs.left.dq.ankleRollRad
              << " imu_valid=" << (imuValid ? 1 : 0)
              << "\n";
}

void printMockRlAction(int sample, double policyTimeSeconds, MockRlAction const& action) {
    std::cout << std::fixed << std::setprecision(6)
              << "mock_rl_action sample=" << sample
              << " policy_time_s=" << policyTimeSeconds
              << " r_hip_pitch=" << action.right.hipPitchRad
              << " r_knee_pitch=" << action.right.kneePitchRad
              << " r_ankle_pitch=" << action.right.anklePitchRad
              << " r_ankle_roll=" << action.right.ankleRollRad
              << " l_hip_pitch=" << action.left.hipPitchRad
              << " l_knee_pitch=" << action.left.kneePitchRad
              << " l_ankle_pitch=" << action.left.anklePitchRad
              << " l_ankle_roll=" << action.left.ankleRollRad
              << "\n";
}

void printMockRlResolvedTargets(int sample, MockRlResolvedTargets const& resolved) {
    std::cout << std::fixed << std::setprecision(6)
              << "mock_rl_resolved sample=" << sample
              << " right_motor3=" << resolved.right.hipPitchMotorRad
              << " right_motor4=" << resolved.right.kneeMotorRad
              << " right_motor5=" << resolved.right.ankleMotorERad
              << " right_motor6=" << resolved.right.ankleMotorFRad
              << " left_motor9=" << resolved.left.hipPitchMotorRad
              << " left_motor10=" << resolved.left.kneeMotorRad
              << " left_motor11=" << resolved.left.ankleMotorERad
              << " left_motor12=" << resolved.left.ankleMotorFRad
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

void printLoopSummary(int sample,
                      DriverSDK::imuStruct const& imu,
                      bool imuValid,
                      int feedbackMisses,
                      LoopTimingStats const& stats) {
    std::cout << "mock_sample " << sample
              << " imu_valid=" << (imuValid ? 1 : 0)
              << " rpy=" << imu.rpy[0] << "," << imu.rpy[1] << "," << imu.rpy[2]
              << " gyr=" << imu.gyr[0] << "," << imu.gyr[1] << "," << imu.gyr[2]
              << " acc=" << imu.acc[0] << "," << imu.acc[1] << "," << imu.acc[2]
              << " feedback_misses=" << feedbackMisses
              << " overruns=" << stats.overruns
              << "\n";
}

void printControllerSummary(LoopTimingStats const& stats,
                            int policyHz,
                            int iterations,
                            int feedbackMisses,
                            int imuValidSamples,
                            int imuZeroAfterFirstValid) {
    double const averageMs = stats.samples > 0 ? stats.intervalSumMs / stats.samples : 0.0;
    double const minMs = stats.samples > 0 ? stats.intervalMinMs : 0.0;
    std::cout << "mock_controller_summary"
              << " policy_hz=" << policyHz
              << " iterations=" << iterations
              << " interval_samples=" << stats.samples
              << " interval_min_ms=" << minMs
              << " interval_avg_ms=" << averageMs
              << " interval_max_ms=" << stats.intervalMaxMs
              << " overruns=" << stats.overruns
              << " overrun_max_ms=" << stats.overrunMaxMs
              << " feedback_misses=" << feedbackMisses
              << " imu_valid_samples=" << imuValidSamples
              << " imu_zero_after_first_valid=" << imuZeroAfterFirstValid
              << "\n";
}

int runRlObservationAudit() {
    std::vector<DriverSDK::motorActualStruct> actuals(13);
    RmdCanSdk::LegJointTargets joints;
    joints.hipPitchRad = 0.35;
    joints.kneePitchRad = -0.70;
    joints.anklePitchRad = 0.35;
    joints.ankleRollRad = 0.0;
    RmdCanSdk::LegMotorTargets const motors = RmdCanSdk::solveRightLegMotorsFromJoints(joints);

    actuals[2].pos = static_cast<float>(motors.hipPitchMotorRad);
    actuals[3].pos = static_cast<float>(motors.kneeMotorRad);
    actuals[4].pos = static_cast<float>(motors.ankleMotorERad);
    actuals[5].pos = static_cast<float>(motors.ankleMotorFRad);
    actuals[8].pos = static_cast<float>(motors.hipPitchMotorRad);
    actuals[9].pos = static_cast<float>(motors.kneeMotorRad);
    actuals[10].pos = static_cast<float>(motors.ankleMotorERad);
    actuals[11].pos = static_cast<float>(motors.ankleMotorFRad);

    RlObservationState state;
    RlObservation const obs = buildRlObservation(actuals, state, 0.0);
    printRlObservation(0, obs, true);

    if (!near(obs.right.q.hipPitchRad, 0.35, 1.0e-5) ||
        !near(obs.right.q.kneePitchRad, -0.70, 1.0e-5) ||
        !near(obs.right.q.anklePitchRad, 0.35, 1.0e-5) ||
        !near(obs.right.q.ankleRollRad, 0.0, 1.0e-5) ||
        !near(obs.left.q.hipPitchRad, 0.35, 1.0e-5) ||
        !near(obs.left.q.kneePitchRad, -0.70, 1.0e-5) ||
        !near(obs.left.q.anklePitchRad, 0.35, 1.0e-5) ||
        !near(obs.left.q.ankleRollRad, 0.0, 1.0e-5)) {
        std::cerr << "rl observation audit failed\n";
        return 1;
    }

    std::cout << "rl observation audit passed\n";
    return 0;
}

int runMockRlPolicyAudit() {
    std::vector<DriverSDK::motorActualStruct> actuals(13);
    RlObservationState observationState;
    MockRlResolvedTargets previousResolved;

    for (double t : {0.0, 1.0, 2.0, 3.0}) {
        int const sample = static_cast<int>(t * 100.0);
        MockRlAction const action = sampleMockRlAction(t);
        MockRlResolvedTargets const resolved = resolveMockRlAction(action, previousResolved);
        printMockRlAction(sample, t, action);
        printMockRlResolvedTargets(sample, resolved);

        actuals[2].pos = static_cast<float>(resolved.right.hipPitchMotorRad);
        actuals[3].pos = static_cast<float>(resolved.right.kneeMotorRad);
        actuals[4].pos = static_cast<float>(resolved.right.ankleMotorERad);
        actuals[5].pos = static_cast<float>(resolved.right.ankleMotorFRad);
        actuals[8].pos = static_cast<float>(resolved.left.hipPitchMotorRad);
        actuals[9].pos = static_cast<float>(resolved.left.kneeMotorRad);
        actuals[10].pos = static_cast<float>(resolved.left.ankleMotorERad);
        actuals[11].pos = static_cast<float>(resolved.left.ankleMotorFRad);

        RlObservation const obs = buildRlObservation(actuals, observationState, t == 0.0 ? 0.0 : 1.0);
        printRlObservation(sample, obs, true);
        if (!near(obs.right.q.hipPitchRad, action.right.hipPitchRad, 1.0e-5) ||
            !near(obs.right.q.kneePitchRad, action.right.kneePitchRad, 1.0e-5) ||
            !near(obs.right.q.anklePitchRad, action.right.anklePitchRad, 1.0e-5) ||
            !near(obs.left.q.hipPitchRad, action.left.hipPitchRad, 1.0e-5) ||
            !near(obs.left.q.kneePitchRad, action.left.kneePitchRad, 1.0e-5) ||
            !near(obs.left.q.anklePitchRad, action.left.anklePitchRad, 1.0e-5)) {
            std::cerr << "mock rl policy audit failed: resolved motors do not round-trip to action\n";
            return 1;
        }
        if (std::fabs(resolved.right.ankleMotorERad) > 0.5 ||
            std::fabs(resolved.left.ankleMotorERad) > 0.5) {
            std::cerr << "mock rl policy audit failed: ankle E motor target exceeds validated config limit\n";
            return 1;
        }
        previousResolved = resolved;
    }

    MockRlAction const high = sampleMockRlAction(1.0);
    MockRlAction const low = sampleMockRlAction(3.0);
    if (!near(high.right.hipPitchRad, 0.45, 1.0e-6) ||
        !near(high.right.kneePitchRad, -0.48, 1.0e-6) ||
        !near(high.right.anklePitchRad, 0.35, 1.0e-6) ||
        !near(low.right.hipPitchRad, 0.25, 1.0e-6) ||
        !near(low.right.kneePitchRad, -0.92, 1.0e-6) ||
        !near(low.right.anklePitchRad, 0.35, 1.0e-6)) {
        std::cerr << "mock rl policy audit failed: unexpected action amplitude\n";
        return 1;
    }

    std::cout << "mock rl policy audit passed\n";
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);

    if (argc == 2 && std::string(argv[1]) == "--audit-rl-observation") {
        return runRlObservationAudit();
    }
    if (argc == 2 && std::string(argv[1]) == "--audit-mock-rl-policy") {
        return runMockRlPolicyAudit();
    }

    if (argc < 8) {
        printUsage(argv[0]);
        return 2;
    }

    char const* configPath = argv[1];
    int const durationMs = parsePositive(argv[2]);
    int const policyHz = parsePositive(argv[3]);
    int const maxCurrent = parsePositive(argv[4]);
    int const settleSamples = parsePositive(argv[5]);
    int const rampMs = parseNonNegative(argv[6]);
    int const rtPriority = parseNonNegative(argv[7]);
    if (durationMs <= 0 || policyHz <= 0 || policyHz > 1000 || maxCurrent <= 0 ||
        maxCurrent > 65535 || settleSamples <= 0 || rampMs < 0 || rtPriority < 0 || rtPriority > 99) {
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
        bool mockRlJointPolicy = false;
        for (int arg = 8; arg < argc; ++arg) {
            if (std::string(argv[arg]) == MockRlPolicyFlag) {
                mockRlJointPolicy = true;
                continue;
            }
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
        if (mockRlJointPolicy &&
            std::any_of(hasRequestedTarget.begin(), hasRequestedTarget.end(), [](bool value) { return value; })) {
            std::cerr << MockRlPolicyFlag << " cannot be combined with explicit motor=position targets\n";
            return 2;
        }
        if (mockRlJointPolicy && !validateMockRlPolicyLimits(parsedConfig, durationMs, policyHz)) {
            return 2;
        }

        auto& sdk = DriverSDK::DriverSDK::instance();
        std::vector<char> modes(static_cast<std::size_t>(countFromConfig), static_cast<char>(8));
        std::vector<unsigned short> maxCurrents(static_cast<std::size_t>(countFromConfig),
                                                static_cast<unsigned short>(maxCurrent));
        sdk.setMode(modes);
        sdk.setMaxCurr(maxCurrents);
        sdk.init(configPath);
        applyRealtimeSettings(rtPriority);

        for (std::size_t i = 0; i < hasRequestedTarget.size(); ++i) {
            if (hasRequestedTarget[i] && !isActiveMotor(sdk, static_cast<int>(i))) {
                std::cerr << "requested motor " << i + 1 << " is not active in this backend\n";
                return 2;
            }
            if (mockRlJointPolicy && isMockRlControlledMotorAlias(static_cast<int>(i) + 1) &&
                !isActiveMotor(sdk, static_cast<int>(i))) {
                std::cerr << "mock RL motor " << i + 1 << " is not active in this backend\n";
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
            finalPositions[index] =
                (!mockRlJointPolicy && hasRequestedTarget[index]) ? requestedTargets[index] : actuals[index].pos;
            targets[index].pos = startPositions[index];
            targets[index].vel = 0.0f;
            targets[index].tor = 0.0f;
            targets[index].kp = 0.0f;
            targets[index].kd = 0.0f;
            targets[index].enabled = 1;
            bool const mockRlControlled = mockRlJointPolicy && isMockRlControlledMotorAlias(active + 1);
            std::cout << "mock target motor " << active + 1
                      << " start=" << startPositions[index]
                      << " final=" << finalPositions[index]
                      << " max_current=" << maxCurrent
                      << " ramp_ms=" << rampMs
                      << (mockRlControlled ? " mock-rl-action"
                                           : (hasRequestedTarget[index] ? " requested" : " hold-current"))
                      << "\n";
        }

        int const sleepMs = std::max(1, 1000 / policyHz);
        int consecutiveOperationEnabled = 0;
        for (int sample = 0; sample < settleSamples && !gStopRequested; ++sample) {
            if (sdk.setMotorTarget(targets) != 0) {
                std::cerr << "setMotorTarget failed while enabling motors\n";
                disableMotors(sdk, targets, 50, sleepMs);
                return 1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(sleepMs));
            int const actualStatus = sdk.getMotorActual(actuals);
            if (operationFeedbackReady(sdk, actuals, actualStatus)) {
                ++consecutiveOperationEnabled;
            } else {
                consecutiveOperationEnabled = 0;
            }
            if ((sample % std::max(1, policyHz / 10)) == 0 || consecutiveOperationEnabled >= 10 ||
                sample + 1 == settleSamples) {
                printActuals("enable", sample, sdk, actuals);
            }
            if (consecutiveOperationEnabled >= 10) {
                break;
            }
        }
        if (gStopRequested) {
            std::cerr << "stop requested while enabling motors\n";
            disableMotors(sdk, targets, 50, sleepMs);
            return 130;
        }
        if (consecutiveOperationEnabled < 10) {
            std::cerr << "motors did not reach operation enabled; consecutive enabled samples="
                      << consecutiveOperationEnabled << "\n";
            disableMotors(sdk, targets, 50, sleepMs);
            return 1;
        }

        long long const iterationsRaw =
            (static_cast<long long>(durationMs) * static_cast<long long>(policyHz) + 999LL) / 1000LL;
        int const iterations = static_cast<int>(std::max(1LL, iterationsRaw));
        std::chrono::nanoseconds const period(1000000000LL / policyHz);

        int consecutiveMissedFeedback = 0;
        int feedbackMisses = 0;
        int imuValidSamples = 0;
        int imuZeroAfterFirstValid = 0;
        bool sawValidImu = false;
        DriverSDK::imuStruct lastImu{};
        RlObservationState rlObservationState;
        RlObservation lastRlObservation;
        MockRlAction lastMockRlAction{};
        MockRlResolvedTargets lastMockRlResolved{};
        std::vector<float> currentMockRlTargets(static_cast<std::size_t>(count), 0.0f);
        std::vector<bool> hasCurrentMockRlTarget(static_cast<std::size_t>(count), false);
        LoopTimingStats timingStats;
        auto nextWake = std::chrono::steady_clock::now();
        auto previousLoopStart = nextWake;
        for (int i = 0; i < iterations && !gStopRequested; ++i) {
            auto const loopStart = std::chrono::steady_clock::now();
            double loopIntervalMs = 0.0;
            if (i > 0) {
                loopIntervalMs = elapsedMs(previousLoopStart, loopStart);
                recordLoopInterval(timingStats, loopIntervalMs);
            }
            previousLoopStart = loopStart;

            int const actualStatus = sdk.getMotorActual(actuals);
            DriverSDK::imuStruct imu{};
            sdk.getIMU(imu);
            lastImu = imu;

            bool const imuValid = imuNonZero(imu);
            lastRlObservation = buildRlObservation(actuals, rlObservationState, loopIntervalMs / 1000.0);
            if (imuValid) {
                ++imuValidSamples;
                sawValidImu = true;
            } else if (sawValidImu) {
                ++imuZeroAfterFirstValid;
            }

            if (!operationFeedbackReady(sdk, actuals, actualStatus)) {
                ++feedbackMisses;
                ++consecutiveMissedFeedback;
                printActuals("mock_feedback_miss", i, sdk, actuals);
                if (consecutiveMissedFeedback >= MaxConsecutiveMissedFeedback) {
                    std::cerr << "operation-enabled feedback lost in mock controller for "
                              << consecutiveMissedFeedback << " consecutive policy samples\n";
                    disableMotors(sdk, targets, 50, sleepMs);
                    return 1;
                }
            } else {
                consecutiveMissedFeedback = 0;
            }

            double const elapsedLoopMs = static_cast<double>(i) * 1000.0 / static_cast<double>(policyHz);
            float const progress = smoothRampProgressForElapsed(elapsedLoopMs, rampMs);
            if (mockRlJointPolicy) {
                double const policyTimeSeconds = std::max(0.0, (elapsedLoopMs - rampMs) / 1000.0);
                lastMockRlAction = sampleMockRlAction(policyTimeSeconds);
                lastMockRlResolved = resolveMockRlAction(lastMockRlAction, lastMockRlResolved);
                std::fill(hasCurrentMockRlTarget.begin(), hasCurrentMockRlTarget.end(), false);
                assignMockRlResolvedTargetsUnchecked(lastMockRlResolved,
                                                     currentMockRlTargets,
                                                     hasCurrentMockRlTarget);
                for (int active : sdk.getActiveMotors()) {
                    std::size_t const index = static_cast<std::size_t>(active);
                    float const finalPosition = hasCurrentMockRlTarget[index]
                                                    ? currentMockRlTargets[index]
                                                    : startPositions[index];
                    targets[index].pos =
                        startPositions[index] + (finalPosition - startPositions[index]) * progress;
                }
            } else {
                for (int active : sdk.getActiveMotors()) {
                    std::size_t const index = static_cast<std::size_t>(active);
                    targets[index].pos =
                        startPositions[index] + (finalPositions[index] - startPositions[index]) * progress;
                }
            }
            if (sdk.setMotorTarget(targets) != 0) {
                std::cerr << "setMotorTarget failed in mock controller loop\n";
                disableMotors(sdk, targets, 50, sleepMs);
                return 1;
            }

            if ((i % std::max(1, policyHz)) == 0 || i + 1 == iterations) {
                printLoopSummary(i, imu, imuValid, feedbackMisses, timingStats);
                printRlObservation(i, lastRlObservation, imuValid);
                if (mockRlJointPolicy) {
                    double const policyTimeSeconds = std::max(0.0, (elapsedLoopMs - rampMs) / 1000.0);
                    printMockRlAction(i, policyTimeSeconds, lastMockRlAction);
                    printMockRlResolvedTargets(i, lastMockRlResolved);
                }
            }

            nextWake += period;
            auto const beforeSleep = std::chrono::steady_clock::now();
            recordLoopOverrun(timingStats, elapsedMs(nextWake, beforeSleep));
            if (beforeSleep < nextWake) {
                std::this_thread::sleep_until(nextWake);
            }
        }

        if (gStopRequested) {
            std::cerr << "stop requested; disabling motors\n";
        }
        int exitCode = gStopRequested ? 130 : 0;
        int const finalStatus = sdk.getMotorActual(actuals);
        bool const finalFeedbackReady = operationFeedbackReady(sdk, actuals, finalStatus);
        if (finalStatus == 0) {
            printActuals("mock_final", iterations, sdk, actuals);
        }
        printLoopSummary(iterations, lastImu, imuNonZero(lastImu), feedbackMisses, timingStats);
        printRlObservation(iterations, lastRlObservation, imuNonZero(lastImu));
        if (mockRlJointPolicy) {
            double const elapsedLoopMs =
                static_cast<double>(std::max(0, iterations - 1)) * 1000.0 / static_cast<double>(policyHz);
            printMockRlAction(iterations,
                              std::max(0.0, (elapsedLoopMs - rampMs) / 1000.0),
                              lastMockRlAction);
            printMockRlResolvedTargets(iterations, lastMockRlResolved);
        }
        printControllerSummary(timingStats,
                               policyHz,
                               iterations,
                               feedbackMisses,
                               imuValidSamples,
                               imuZeroAfterFirstValid);
        if (!finalFeedbackReady) {
            std::cerr << "final motor feedback is not operation-enabled and error-free\n";
            exitCode = exitCode == 0 ? 1 : exitCode;
        }
        if (imuValidSamples == 0) {
            std::cerr << "IMU never produced a nonzero sample in mock controller loop\n";
            exitCode = exitCode == 0 ? 1 : exitCode;
        }
        disableMotors(sdk, targets, 50, sleepMs);
        return exitCode;
    } catch (std::exception const& ex) {
        std::cerr << "mock controller failed: " << ex.what() << "\n";
        return 1;
    }

    return 0;
}
