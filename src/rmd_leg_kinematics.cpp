#include "rmd_can_sdk/rmd_leg_kinematics.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace RmdCanSdk {
namespace {

struct Vec2 {
    double x = 0.0;
    double z = 0.0;
};

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct AnkleMotors {
    double e = 0.0;
    double f = 0.0;
};

double wrapDistance(double a, double b) {
    return std::abs(std::atan2(std::sin(a - b), std::cos(a - b)));
}

double chooseClosest(double first, double second, double reference) {
    return wrapDistance(first, reference) <= wrapDistance(second, reference) ? first : second;
}

double checkedAcos(double value) {
    return std::acos(std::clamp(value, -1.0, 1.0));
}

void requireReachable(double d, double firstRadius, double secondRadius, char const* message) {
    if (d < 1.0e-9 || d > firstRadius + secondRadius || d < std::abs(firstRadius - secondRadius)) {
        throw std::runtime_error(message);
    }
}

double distance(Vec3 a, Vec3 b) {
    double const dx = a.x - b.x;
    double const dy = a.y - b.y;
    double const dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

Vec3 add(Vec3 a, Vec3 b) {
    return Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 rotX(Vec3 c, double angle) {
    double const co = std::cos(angle);
    double const s = std::sin(angle);
    return Vec3{c.x, co * c.y - s * c.z, s * c.y + co * c.z};
}

Vec3 rotY(Vec3 c, double angle) {
    double const co = std::cos(angle);
    double const s = std::sin(angle);
    return Vec3{co * c.x + s * c.z, c.y, -s * c.x + co * c.z};
}

Vec3 pitchRoll(Vec3 c, double pitch, double roll) {
    return rotY(rotX(c, roll), pitch);
}

Vec3 getC(double pitch, double roll) {
    return pitchRoll(Vec3{-87.0, 35.0, 0.0}, pitch, roll);
}

Vec3 getD(double pitch, double roll) {
    return pitchRoll(Vec3{-74.632, -35.0, 0.0}, pitch, roll);
}

Vec3 getE(double theta) {
    return add(rotY(Vec3{-65.0, 0.0, 0.0}, theta), Vec3{-22.0, 35.0, 317.837});
}

Vec3 getF(double theta) {
    return add(rotY(Vec3{-65.0, 0.0, 0.0}, theta), Vec3{-12.0, -34.5, 236.273});
}

template <typename Constraint>
double solveScalarNewton(Constraint constraint, double initialGuess, char const* message) {
    constexpr double h = 1.0e-6;
    constexpr double tolerance = 1.0e-10;
    double x = initialGuess;

    for (int i = 0; i < 30; ++i) {
        double const fx = constraint(x);
        if (std::abs(fx) < tolerance) {
            return x;
        }
        double const dfx = (constraint(x + h) - fx) / h;
        if (std::abs(dfx) < 1.0e-12) {
            throw std::runtime_error(message);
        }
        x -= fx / dfx;
    }

    if (std::abs(constraint(x)) < 1.0e-8) {
        return x;
    }
    throw std::runtime_error(message);
}

AnkleMotors solveAnkleMotorsFromJoints(double pitch,
                                       double roll,
                                       double initialMotorE,
                                       double initialMotorF,
                                       AnkleLinkageConfig const& config) {
    Vec3 const c = getC(pitch, roll);
    Vec3 const d = getD(pitch, roll);
    double const thetaE = solveScalarNewton(
        [&](double theta) {
            return distance(c, getE(theta)) - config.linkCEMm;
        },
        initialMotorE,
        "ankle E linkage solver failed");
    double const thetaF = solveScalarNewton(
        [&](double theta) {
            return distance(d, getF(theta)) - config.linkDFMm;
        },
        initialMotorF,
        "ankle F linkage solver failed");
    return AnkleMotors{thetaE, thetaF};
}

} // namespace

double solveKneeMotorFromJoint(double kneePitchRad,
                               double previousMotorRad,
                               KneeLinkageConfig const& config) {
    double const betaAbs = config.betaZeroRad + kneePitchRad;
    Vec2 const k{config.kneeArmMm * std::cos(betaAbs), config.kneeArmMm * std::sin(betaAbs)};
    Vec2 const m{config.motorXmm, config.motorZmm};
    double const dx = k.x - m.x;
    double const dz = k.z - m.z;
    double const d = std::hypot(dx, dz);
    requireReachable(d,
                     config.crankRadiusMm,
                     config.rodLengthMm,
                     "knee target is outside linkage reach");

    double const phi = std::atan2(dz, dx);
    double const alpha = checkedAcos((config.crankRadiusMm * config.crankRadiusMm + d * d -
                                      config.rodLengthMm * config.rodLengthMm) /
                                     (2.0 * config.crankRadiusMm * d));
    double const first = phi + alpha - config.thetaZeroRad;
    double const second = phi - alpha - config.thetaZeroRad;
    return chooseClosest(first, second, previousMotorRad);
}

double solveKneeJointFromMotor(double motorRad,
                               double previousKneePitchRad,
                               KneeLinkageConfig const& config) {
    double const thetaAbs = config.thetaZeroRad + motorRad;
    Vec2 const e{config.motorXmm + config.crankRadiusMm * std::cos(thetaAbs),
                 config.motorZmm + config.crankRadiusMm * std::sin(thetaAbs)};
    double const d = std::hypot(e.x, e.z);
    requireReachable(d, config.kneeArmMm, config.rodLengthMm, "knee motor is outside linkage reach");

    double const phi = std::atan2(e.z, e.x);
    double const alpha = checkedAcos((config.kneeArmMm * config.kneeArmMm + d * d -
                                      config.rodLengthMm * config.rodLengthMm) /
                                     (2.0 * config.kneeArmMm * d));
    double const first = phi + alpha - config.betaZeroRad;
    double const second = phi - alpha - config.betaZeroRad;
    return chooseClosest(first, second, previousKneePitchRad);
}

LegMotorTargets solveRightLegMotorsFromJoints(LegJointTargets const& joints,
                                              LegMotorTargets const& previousMotors,
                                              KneeLinkageConfig const& knee,
                                              AnkleLinkageConfig const& ankle) {
    LegMotorTargets motors;
    motors.hipPitchMotorRad = joints.hipPitchRad;
    motors.kneeMotorRad = solveKneeMotorFromJoint(joints.kneePitchRad, previousMotors.kneeMotorRad, knee);
    AnkleMotors const ankleMotors = solveAnkleMotorsFromJoints(joints.anklePitchRad,
                                                               joints.ankleRollRad,
                                                               previousMotors.ankleMotorERad,
                                                               previousMotors.ankleMotorFRad,
                                                               ankle);
    motors.ankleMotorERad = ankleMotors.e;
    motors.ankleMotorFRad = ankleMotors.f;
    return motors;
}

AnkleJointAngles solveAnkleJointsFromMotors(double motorERad,
                                            double motorFRad,
                                            double initialPitchRad,
                                            double initialRollRad,
                                            AnkleLinkageConfig const& config) {
    constexpr double h = 1.0e-5;
    constexpr double tolerance = 1.0e-10;
    double pitch = initialPitchRad;
    double roll = initialRollRad;

    auto residual = [&](double p, double r) {
        AnkleMotors const motors = solveAnkleMotorsFromJoints(p, r, motorERad, motorFRad, config);
        return AnkleMotors{motors.e - motorERad, motors.f - motorFRad};
    };

    for (int i = 0; i < 20; ++i) {
        AnkleMotors const f = residual(pitch, roll);
        if (std::hypot(f.e, f.f) < tolerance) {
            return AnkleJointAngles{pitch, roll};
        }

        AnkleMotors const fPitch = residual(pitch + h, roll);
        AnkleMotors const fRoll = residual(pitch, roll + h);
        double const j11 = (fPitch.e - f.e) / h;
        double const j21 = (fPitch.f - f.f) / h;
        double const j12 = (fRoll.e - f.e) / h;
        double const j22 = (fRoll.f - f.f) / h;
        double const det = j11 * j22 - j12 * j21;
        if (std::abs(det) < 1.0e-12) {
            throw std::runtime_error("ankle inverse linkage solver is singular");
        }

        double const dp = (-f.e * j22 + j12 * f.f) / det;
        double const dr = (j21 * f.e - j11 * f.f) / det;
        pitch += dp;
        roll += dr;
    }

    AnkleMotors const f = residual(pitch, roll);
    if (std::hypot(f.e, f.f) < 1.0e-8) {
        return AnkleJointAngles{pitch, roll};
    }
    throw std::runtime_error("ankle inverse linkage solver failed");
}

} // namespace RmdCanSdk
