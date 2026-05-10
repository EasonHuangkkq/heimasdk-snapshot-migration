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
constexpr float DegToRad = Pi / 180.0f;
constexpr float RadToDeg = 180.0f / Pi;
constexpr int SettleMs = 700;
constexpr double SampleDtSoftLimitMs = 2.0;
constexpr double SampleDtP999LimitMs = 1.5;
constexpr double SampleDtHardLimitMs = 3.0;

struct Candidate {
    float kp = 0.0f;
    float kd = 0.0f;
    float commandGain = 1.0f;
    float commandAmplitudeDeg = 0.0f;
};

struct Result {
    Candidate candidate;
    int candidateIndex = 0;
    int samples = 0;
    int actualFailures = 0;
    double meanAbsErrorDeg = 0.0;
    double rmsErrorDeg = 0.0;
    double maxAbsErrorDeg = 0.0;
    double maxAbsVelocityRadS = 0.0;
    double maxAbsTorqueNm = 0.0;
    double sampleDtMaxMs = 0.0;
    double sampleDtP999Ms = 0.0;
    int sampleDtOver2Ms = 0;
    double actualMinDeg = 0.0;
    double actualMaxDeg = 0.0;
    std::string samplePath;
    std::string safetyAbortReason;
};

struct AuditDecision {
    std::string status;
    std::string reason;
    bool acceptable = false;
    double score = 0.0;
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

bool parseFloatList(std::string const& text, std::vector<float>& out) {
    out.clear();
    std::size_t begin = 0;
    while (begin <= text.size()) {
        std::size_t const comma = text.find(',', begin);
        std::string item = text.substr(begin, comma == std::string::npos ? std::string::npos : comma - begin);
        item.erase(std::remove_if(item.begin(), item.end(), [](unsigned char ch) { return std::isspace(ch); }), item.end());
        if (!item.empty()) {
            float value = 0.0f;
            if (!parseFloatStrict(item.c_str(), &value)) {
                return false;
            }
            out.push_back(value);
        }
        if (comma == std::string::npos) {
            break;
        }
        begin = comma + 1;
    }
    return !out.empty();
}

std::string defaultOutputDir() {
    std::filesystem::create_directories("logs");
    return "logs/rmd_mit_sine_autotune_" + timestamp();
}

void disableMit(DriverSDK::DriverSDK& sdk, std::vector<DriverSDK::motorTargetStruct>& targets) {
    for (auto& target : targets) {
        target.enabled = 0;
        target.tor = 0.0f;
    }
    sdk.setMotorTarget(targets);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void writeSampleHeader(std::ofstream& out) {
    out << "elapsed_ms,sample_ms,eval_target_deg,command_target_deg,actual_pos_deg,"
           "actual_vel_rad_s,actual_torque_nm,error_deg,status_word,error_code,actual_ok\n";
}

void settleAtZero(DriverSDK::DriverSDK& sdk,
                  std::vector<DriverSDK::motorTargetStruct>& targets,
                  std::chrono::steady_clock::duration samplePeriod,
                  std::size_t index) {
    targets[index].pos = 0.0f;
    targets[index].vel = 0.0f;
    targets[index].tor = 0.0f;
    auto const start = std::chrono::steady_clock::now();
    auto next = start;
    auto const deadline = start + std::chrono::milliseconds(SettleMs);
    while (!stopRequested.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline) {
        sdk.setMotorTarget(targets);
        next += samplePeriod;
        std::this_thread::sleep_until(next);
    }
}

int allowedSampleDtOverruns(int samples) {
    return std::max(1, samples / 10000);
}

bool sampleTimingAcceptable(Result const& result) {
    return result.sampleDtP999Ms <= SampleDtP999LimitMs
        && result.sampleDtMaxMs <= SampleDtHardLimitMs
        && result.sampleDtOver2Ms <= allowedSampleDtOverruns(result.samples);
}

double sampleTimingPenalty(Result const& result) {
    double const p999Penalty = std::max(0.0, result.sampleDtP999Ms - SampleDtP999LimitMs) * 20.0;
    double const maxPenalty = std::max(0.0, result.sampleDtMaxMs - SampleDtHardLimitMs) * 50.0;
    double const overrunPenalty =
        std::max(0, result.sampleDtOver2Ms - allowedSampleDtOverruns(result.samples)) * 0.5;
    return p999Penalty + maxPenalty + overrunPenalty;
}

double score(Result const& result) {
    if (result.samples <= 0 || result.actualFailures > 0) {
        return 10000.0 + 1000.0 * result.actualFailures;
    }
    Candidate const& c = result.candidate;
    return result.meanAbsErrorDeg
         + 0.20 * result.rmsErrorDeg
         + 0.08 * result.maxAbsErrorDeg
         + 0.035 * result.maxAbsTorqueNm
         + 0.004 * c.kp
         + 0.20 * c.kd
         + 0.30 * std::max(0.0f, c.commandGain - 1.0f)
         + sampleTimingPenalty(result);
}

bool acceptable(Result const& result, float maxMeanErrorDeg, float maxTorqueNm) {
    return result.samples > 0
        && result.actualFailures == 0
        && result.meanAbsErrorDeg <= maxMeanErrorDeg
        && result.maxAbsTorqueNm <= maxTorqueNm
        && sampleTimingAcceptable(result);
}

AuditDecision auditResult(Result const& result,
                          double bestScore,
                          bool haveBest,
                          float maxMeanErrorDeg,
                          float maxTorqueNm) {
    AuditDecision decision;
    decision.score = score(result);
    decision.acceptable = acceptable(result, maxMeanErrorDeg, maxTorqueNm);
    if (result.samples <= 0) {
        decision.status = "crash";
        decision.reason = "no_valid_samples";
        return decision;
    }
    if (result.actualFailures > 0) {
        decision.status = "crash";
        decision.reason = "feedback_failures";
        return decision;
    }
    if (!result.safetyAbortReason.empty()) {
        decision.status = "discard";
        decision.reason = result.safetyAbortReason;
        return decision;
    }
    if (!sampleTimingAcceptable(result)) {
        decision.status = "discard";
        decision.reason = "sample_timing_unstable";
        return decision;
    }
    if (result.maxAbsTorqueNm > maxTorqueNm) {
        decision.status = "discard";
        decision.reason = "torque_limit";
        return decision;
    }
    if (!haveBest || decision.score < bestScore) {
        decision.status = "keep";
        if (!haveBest) {
            decision.reason = decision.acceptable ? "baseline" : "baseline_unacceptable";
        } else {
            decision.reason = decision.acceptable ? "score_improved" : "score_improved_unacceptable";
        }
        return decision;
    }
    decision.status = "discard";
    decision.reason = decision.acceptable ? "score_not_improved" : "mean_error_limit";
    return decision;
}

Result runCandidate(DriverSDK::DriverSDK& sdk,
                    std::vector<DriverSDK::motorTargetStruct>& targets,
                    std::vector<DriverSDK::motorActualStruct>& actuals,
                    std::size_t index,
                    Candidate const& candidate,
                    int candidateIndex,
                    std::filesystem::path const& outputDir,
                    int durationSec,
                    std::chrono::steady_clock::duration samplePeriod,
                    float evalAmplitudeDeg,
                    float frequencyHz,
                    float maxTorqueNm) {
    Result result;
    result.candidate = candidate;
    result.candidateIndex = candidateIndex;

    std::ostringstream name;
    name << "candidate_" << std::setw(3) << std::setfill('0') << candidateIndex
         << "_kp" << std::setfill(' ') << std::setprecision(4) << candidate.kp
         << "_kd" << candidate.kd
         << "_gain" << candidate.commandGain
         << "_cmd" << candidate.commandAmplitudeDeg
         << "_samples.csv";
    std::filesystem::path samplePath = outputDir / name.str();
    result.samplePath = samplePath.string();
    std::ofstream samples(samplePath);
    writeSampleHeader(samples);

    settleAtZero(sdk, targets, samplePeriod, index);
    sdk.getMotorActual(actuals);
    auto lastActual = actuals[index];

    double const omega = 2.0 * static_cast<double>(Pi) * static_cast<double>(frequencyHz);
    float const evalAmplitudeRad = evalAmplitudeDeg * DegToRad;
    float const commandAmplitudeRad = candidate.commandAmplitudeDeg * DegToRad;
    auto const testStart = std::chrono::steady_clock::now();
    auto const phaseStart = testStart;
    auto const deadline = phaseStart + std::chrono::seconds(durationSec);
    auto nextSample = phaseStart + samplePeriod;
    auto lastSample = phaseStart;
    double sumAbsError = 0.0;
    double sumSquaredError = 0.0;
    std::vector<double> sampleDts;
    bool haveActual = false;

    while (!stopRequested.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_until(nextSample);
        auto const sampleTime = std::chrono::steady_clock::now();
        nextSample += samplePeriod;

        double const t = std::chrono::duration<double>(sampleTime - phaseStart).count();
        float const evalTargetRad = evalAmplitudeRad * static_cast<float>(std::sin(omega * t));
        float const commandTargetRad = commandAmplitudeRad * static_cast<float>(std::sin(omega * t));
        targets[index].pos = commandTargetRad;
        targets[index].vel = 0.0f;
        targets[index].tor = 0.0f;
        sdk.setMotorTarget(targets);

        bool const actualOk = sdk.getMotorActual(actuals) == 0;
        if (actualOk) {
            lastActual = actuals[index];
            double const actualDeg = static_cast<double>(lastActual.pos) * RadToDeg;
            double const evalTargetDeg = static_cast<double>(evalTargetRad) * RadToDeg;
            double const err = actualDeg - evalTargetDeg;
            sumAbsError += std::fabs(err);
            sumSquaredError += err * err;
            result.maxAbsErrorDeg = std::max(result.maxAbsErrorDeg, std::fabs(err));
            result.maxAbsVelocityRadS = std::max(result.maxAbsVelocityRadS, static_cast<double>(std::fabs(lastActual.vel)));
            result.maxAbsTorqueNm = std::max(result.maxAbsTorqueNm, static_cast<double>(std::fabs(lastActual.tor)));
            if (!haveActual) {
                result.actualMinDeg = actualDeg;
                result.actualMaxDeg = actualDeg;
                haveActual = true;
            } else {
                result.actualMinDeg = std::min(result.actualMinDeg, actualDeg);
                result.actualMaxDeg = std::max(result.actualMaxDeg, actualDeg);
            }
            ++result.samples;
            samples << std::fixed << std::setprecision(3)
                    << msBetween(testStart, sampleTime) << ','
                    << msBetween(phaseStart, sampleTime) << ','
                    << std::setprecision(6)
                    << evalTargetDeg << ','
                    << static_cast<double>(commandTargetRad) * RadToDeg << ','
                    << actualDeg << ','
                    << lastActual.vel << ','
                    << lastActual.tor << ','
                    << err << ','
                    << lastActual.statusWord << ','
                    << lastActual.errorCode << ",1\n";
            if (std::fabs(lastActual.tor) > maxTorqueNm) {
                result.safetyAbortReason = "runtime_torque_limit";
                break;
            }
        } else {
            ++result.actualFailures;
            samples << std::fixed << std::setprecision(3)
                    << msBetween(testStart, sampleTime) << ','
                    << msBetween(phaseStart, sampleTime)
                    << ",0,0,0,0,0,0,0,0,0\n";
        }

        double const dtMs = msBetween(lastSample, sampleTime);
        result.sampleDtMaxMs = std::max(result.sampleDtMaxMs, dtMs);
        if (dtMs > SampleDtSoftLimitMs) {
            ++result.sampleDtOver2Ms;
        }
        sampleDts.push_back(dtMs);
        lastSample = sampleTime;
    }

    if (result.samples > 0) {
        result.meanAbsErrorDeg = sumAbsError / result.samples;
        result.rmsErrorDeg = std::sqrt(sumSquaredError / result.samples);
    }
    if (!sampleDts.empty()) {
        std::sort(sampleDts.begin(), sampleDts.end());
        std::size_t const index = static_cast<std::size_t>(
            std::floor(0.999 * static_cast<double>(sampleDts.size() - 1)));
        result.sampleDtP999Ms = sampleDts[index];
    }
    return result;
}

void writeResultsHeader(std::ofstream& out) {
    out << "candidate_id\tstatus\taudit_reason\tacceptable\tscore\tkp\tkd\tcommand_gain\tcommand_amplitude_deg\tmean_abs_error_deg\t"
           "rms_error_deg\tmax_abs_error_deg\tactual_min_deg\tactual_max_deg\t"
           "max_abs_velocity_rad_s\tmax_abs_torque_nm\tsample_dt_max_ms\tsample_dt_p999_ms\tsample_dt_over_2ms\tsamples\t"
           "actual_failures\tdescription\tsamples_path\n";
}

void writeResult(std::ofstream& out, Result const& result, AuditDecision const& audit) {
    Candidate const& c = result.candidate;
    out << result.candidateIndex << '\t'
        << audit.status << '\t'
        << audit.reason << '\t'
        << (audit.acceptable ? 1 : 0) << '\t'
        << std::fixed << std::setprecision(6)
        << audit.score << '\t'
        << c.kp << '\t'
        << c.kd << '\t'
        << c.commandGain << '\t'
        << c.commandAmplitudeDeg << '\t'
        << result.meanAbsErrorDeg << '\t'
        << result.rmsErrorDeg << '\t'
        << result.maxAbsErrorDeg << '\t'
        << result.actualMinDeg << '\t'
        << result.actualMaxDeg << '\t'
        << result.maxAbsVelocityRadS << '\t'
        << result.maxAbsTorqueNm << '\t'
        << result.sampleDtMaxMs << '\t'
        << result.sampleDtP999Ms << '\t'
        << result.sampleDtOver2Ms << '\t'
        << result.samples << '\t'
        << result.actualFailures << '\t'
        << "kp=" << c.kp << " kd=" << c.kd
        << " command_gain=" << c.commandGain
        << " command_amplitude_deg=" << c.commandAmplitudeDeg << '\t'
        << result.samplePath << '\n';
    out.flush();
}

int runAuditSelfTest() {
    Result stable;
    stable.candidate = Candidate{200.0f, 0.2f, 0.9f, 18.0f};
    stable.samples = 120000;
    stable.meanAbsErrorDeg = 1.81;
    stable.rmsErrorDeg = 1.92;
    stable.maxAbsErrorDeg = 6.09;
    stable.maxAbsTorqueNm = 13.15;
    stable.sampleDtMaxMs = 2.085;
    stable.sampleDtP999Ms = 1.065;
    stable.sampleDtOver2Ms = 1;

    AuditDecision const stableAudit = auditResult(stable, 0.0, false, 3.0f, 18.0f);
    if (!stableAudit.acceptable || stableAudit.status != "keep") {
        std::cerr << "stable jitter case should pass audit, got status="
                  << stableAudit.status << " reason=" << stableAudit.reason << "\n";
        return 1;
    }

    Result unstable = stable;
    unstable.sampleDtMaxMs = 2.8;
    unstable.sampleDtP999Ms = 2.1;
    unstable.sampleDtOver2Ms = 200;
    AuditDecision const unstableAudit = auditResult(unstable, stableAudit.score, true, 3.0f, 18.0f);
    if (unstableAudit.acceptable || unstableAudit.reason != "sample_timing_unstable") {
        std::cerr << "unstable jitter case should fail audit, got acceptable="
                  << unstableAudit.acceptable << " reason=" << unstableAudit.reason << "\n";
        return 1;
    }

    return 0;
}

} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    if (argc == 2 && std::string(argv[1]) == "--audit-self-test") {
        return runAuditSelfTest();
    }

    std::string const configPath = argc > 1 ? argv[1] : "config/example_rmd_can.xml";
    int durationSec = 5;
    int sampleHz = 1000;
    float evalAmplitudeDeg = 20.0f;
    float frequencyHz = 1.0f;
    std::string kpText = "80,120,160,200";
    std::string kdText = "0.3,0.5";
    std::string gainText = "1.0,1.2,1.4,1.6";
    float maxCommandAmplitudeDeg = 35.0f;
    float maxMeanErrorDeg = 3.0f;
    float maxTorqueNm = 18.0f;
    std::string outputDir = defaultOutputDir();

    if (argc > 2 && !parseIntStrict(argv[2], &durationSec)) {
        std::cerr << "duration_s must be an integer\n";
        return 2;
    }
    if (argc > 3 && !parseIntStrict(argv[3], &sampleHz)) {
        std::cerr << "sample_hz must be an integer\n";
        return 2;
    }
    if (argc > 4 && !parseFloatStrict(argv[4], &evalAmplitudeDeg)) {
        std::cerr << "eval_amplitude_deg must be a number\n";
        return 2;
    }
    if (argc > 5 && !parseFloatStrict(argv[5], &frequencyHz)) {
        std::cerr << "frequency_hz must be a number\n";
        return 2;
    }
    if (argc > 6) {
        kpText = argv[6];
    }
    if (argc > 7) {
        kdText = argv[7];
    }
    if (argc > 8) {
        gainText = argv[8];
    }
    if (argc > 9 && !parseFloatStrict(argv[9], &maxCommandAmplitudeDeg)) {
        std::cerr << "max_command_amplitude_deg must be a number\n";
        return 2;
    }
    if (argc > 10 && !parseFloatStrict(argv[10], &maxMeanErrorDeg)) {
        std::cerr << "max_mean_error_deg must be a number\n";
        return 2;
    }
    if (argc > 11 && !parseFloatStrict(argv[11], &maxTorqueNm)) {
        std::cerr << "max_torque_nm must be a number\n";
        return 2;
    }
    if (argc > 12) {
        outputDir = argv[12];
    }

    if (durationSec <= 0 || durationSec > 120) {
        std::cerr << "duration_s must be in 1..120\n";
        return 2;
    }
    if (sampleHz <= 0 || sampleHz > 2000) {
        std::cerr << "sample_hz must be in 1..2000\n";
        return 2;
    }
    if (evalAmplitudeDeg <= 0.0f || evalAmplitudeDeg > 180.0f) {
        std::cerr << "eval_amplitude_deg must be in 0..180\n";
        return 2;
    }
    if (frequencyHz <= 0.0f || frequencyHz > 20.0f) {
        std::cerr << "frequency_hz must be in 0..20\n";
        return 2;
    }
    if (maxCommandAmplitudeDeg < evalAmplitudeDeg) {
        std::cerr << "max_command_amplitude_deg must be >= eval_amplitude_deg\n";
        return 2;
    }

    std::vector<float> kpValues;
    std::vector<float> kdValues;
    std::vector<float> gainValues;
    if (!parseFloatList(kpText, kpValues) || !parseFloatList(kdText, kdValues) || !parseFloatList(gainText, gainValues)) {
        std::cerr << "kp/kd/gain lists must be comma-separated numbers\n";
        return 2;
    }
    for (float kp : kpValues) {
        if (kp < 0.0f || kp > 500.0f) {
            std::cerr << "kp values must be in 0..500\n";
            return 2;
        }
    }
    for (float kd : kdValues) {
        if (kd < RmdCanSdk::RmdKdMin || kd > RmdCanSdk::RmdKdMax) {
            std::cerr << "kd values must be in " << RmdCanSdk::RmdKdMin << ".." << RmdCanSdk::RmdKdMax << "\n";
            return 2;
        }
    }
    for (float gain : gainValues) {
        if (gain <= 0.0f || gain > 3.0f) {
            std::cerr << "gain values must be in 0..3\n";
            return 2;
        }
    }

    std::vector<Candidate> candidates;
    for (float kp : kpValues) {
        for (float kd : kdValues) {
            for (float gain : gainValues) {
                float const commandAmplitudeDeg = evalAmplitudeDeg * gain;
                if (commandAmplitudeDeg <= maxCommandAmplitudeDeg) {
                    candidates.push_back(Candidate{kp, kd, gain, commandAmplitudeDeg});
                }
            }
        }
    }
    if (candidates.empty()) {
        std::cerr << "candidate plan is empty\n";
        return 2;
    }

    std::filesystem::create_directories(outputDir);
    std::filesystem::path resultsPath = std::filesystem::path(outputDir) / "results.tsv";
    std::ofstream results(resultsPath);
    writeResultsHeader(results);

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

    std::cout << "MIT sine autotune config=" << configPath
              << " can=" << masterIt->device
              << " motor_id=" << motor.motorId
              << " alias=" << motor.alias
              << " eval_amplitude_deg=" << evalAmplitudeDeg
              << " frequency_hz=" << frequencyHz
              << " candidates=" << candidates.size()
              << " output_dir=" << outputDir << "\n";

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
    targets[index].tor = 0.0f;
    targets[index].pos = 0.0f;
    auto const samplePeriod =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1.0 / sampleHz));

    std::vector<Result> allResults;
    int bestIndex = -1;
    double bestScore = 0.0;
    for (std::size_t i = 0; i < candidates.size() && !stopRequested.load(std::memory_order_acquire); ++i) {
        Candidate const& c = candidates[i];
        targets[index].kp = c.kp;
        targets[index].kd = c.kd;
        std::cout << "run " << (i + 1) << "/" << candidates.size()
                  << " kp=" << c.kp
                  << " kd=" << c.kd
                  << " command_gain=" << c.commandGain
                  << " command_amp=" << c.commandAmplitudeDeg << "deg\n";
        Result result = runCandidate(sdk, targets, actuals, index, c, static_cast<int>(i + 1),
                                     outputDir, durationSec, samplePeriod,
                                     evalAmplitudeDeg, frequencyHz, maxTorqueNm);
        AuditDecision const audit = auditResult(result, bestScore, bestIndex >= 0, maxMeanErrorDeg, maxTorqueNm);
        if (audit.status == "keep") {
            bestIndex = static_cast<int>(allResults.size());
            bestScore = audit.score;
        }
        writeResult(results, result, audit);
        std::cout << "  status=" << audit.status
                  << " reason=" << audit.reason
                  << " score=" << std::fixed << std::setprecision(3) << audit.score
                  << " mean_err=" << result.meanAbsErrorDeg
                  << "deg max_err=" << result.maxAbsErrorDeg
                  << "deg torque=" << result.maxAbsTorqueNm
                  << "Nm actual=[" << result.actualMinDeg << ", " << result.actualMaxDeg << "]deg"
                  << (audit.acceptable ? " ACCEPT" : " reject") << "\n";
        allResults.push_back(result);
    }

    targets[index].pos = 0.0f;
    targets[index].vel = 0.0f;
    targets[index].tor = 0.0f;
    sdk.setMotorTarget(targets);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    disableMit(sdk, targets);

    if (bestIndex >= 0) {
        Result const& best = allResults[static_cast<std::size_t>(bestIndex)];
        Candidate const& c = best.candidate;
        std::cout << "--- best ---\n"
                  << "kp=" << c.kp
                  << " kd=" << c.kd
                  << " command_gain=" << c.commandGain
                  << " command_amplitude_deg=" << c.commandAmplitudeDeg << "\n"
                  << "score=" << score(best)
                  << " mean_abs_error_deg=" << best.meanAbsErrorDeg
                  << " rms_error_deg=" << best.rmsErrorDeg
                  << " max_abs_error_deg=" << best.maxAbsErrorDeg << "\n"
                  << "max_abs_torque_nm=" << best.maxAbsTorqueNm
                  << " max_abs_velocity_rad_s=" << best.maxAbsVelocityRadS
                  << " results=" << resultsPath << "\n";
    }
    return stopRequested.load(std::memory_order_acquire) ? 1 : 0;
}
