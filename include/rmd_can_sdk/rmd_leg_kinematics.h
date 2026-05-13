#pragma once

namespace RmdCanSdk {

struct KneeLinkageConfig {
    double thetaZeroRad = 8.4 * 3.14159265358979323846 / 180.0;
    double betaZeroRad = 61.2 * 3.14159265358979323846 / 180.0;
    double crankRadiusMm = 50.0;
    double kneeArmMm = 50.0;
    double rodLengthMm = 215.0;
    double motorXmm = 0.0;
    double motorZmm = 250.0;
};

struct AnkleLinkageConfig {
    double linkCEMm = 317.837;
    double linkDFMm = 236.273;
};

struct LegJointTargets {
    double hipPitchRad = 0.0;
    double kneePitchRad = 0.0;
    double anklePitchRad = 0.0;
    double ankleRollRad = 0.0;
};

struct LegMotorTargets {
    double hipPitchMotorRad = 0.0;
    double kneeMotorRad = 0.0;
    double ankleMotorERad = 0.0;
    double ankleMotorFRad = 0.0;
};

struct AnkleJointAngles {
    double pitchRad = 0.0;
    double rollRad = 0.0;
};

struct KinematicsResult {
    bool valid = true;
    char const* error = "";
};

double solveKneeMotorFromJoint(double kneePitchRad,
                               double previousMotorRad,
                               KneeLinkageConfig const& config = {});

double solveKneeJointFromMotor(double motorRad,
                               double previousKneePitchRad,
                               KneeLinkageConfig const& config = {});

LegMotorTargets solveRightLegMotorsFromJoints(LegJointTargets const& joints,
                                              LegMotorTargets const& previousMotors = {},
                                              KneeLinkageConfig const& knee = {},
                                              AnkleLinkageConfig const& ankle = {});

AnkleJointAngles solveAnkleJointsFromMotors(double motorERad,
                                            double motorFRad,
                                            double initialPitchRad,
                                            double initialRollRad,
                                            AnkleLinkageConfig const& config = {});

} // namespace RmdCanSdk
