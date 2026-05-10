#include "rmd_can_sdk/rmd_can_transport.h"
#include "rmd_can_sdk/rmd_protocol.h"

#include <algorithm>
#include <atomic>
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
#include <vector>

namespace {

std::atomic<bool> stopRequested{false};

constexpr float MotionPositionThresholdDeg = 1.0f;
constexpr float MotionVelocityThresholdDps = 5.0f;
constexpr float TargetToleranceDeg = 2.0f;
constexpr int SettledSpeedDps = 3;
constexpr int MaxA4SpeedDps = 100;
constexpr int MaxStatusSampleHz = 1000;

struct Status2 {
    int angleDeg = 0;
    int speedDps = 0;
    float iqA = 0.0f;
};

struct CommandReply {
    bool ok = false;
    double replyMs = -1.0;
    std::string raw;
};

void handleSignal(int) {
    stopRequested.store(true, std::memory_order_release);
}

double msBetween(std::chrono::steady_clock::time_point begin,
                 std::chrono::steady_clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - begin).count();
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

std::string hexFrame(RmdCanSdk::CanFrame const& frame) {
    std::ostringstream out;
    out << std::hex << std::uppercase << std::setfill('0');
    for (int i = 0; i < frame.length; ++i) {
        if (i > 0) {
            out << ' ';
        }
        out << std::setw(2) << static_cast<int>(frame.data[i]);
    }
    return out.str();
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
    return "logs/rmd_a4_response_" + timestamp();
}

bool requestReply(RmdCanSdk::SocketCanTransport& transport,
                  int motorId,
                  unsigned char const data[8],
                  unsigned char expectedCmd,
                  RmdCanSdk::CanFrame& out,
                  double* replyMs = nullptr,
                  std::chrono::steady_clock::time_point* commandTime = nullptr) {
    int const txId = RmdCanSdk::standardTxId(motorId);
    int const rxId = RmdCanSdk::standardRxId(motorId);
    auto const start = std::chrono::steady_clock::now();
    if (commandTime != nullptr) {
        *commandTime = start;
    }
    if (transport.send(txId, data, 8) < 0) {
        std::cerr << "send failed for cmd 0x" << std::hex << static_cast<int>(data[0]) << std::dec << "\n";
        return false;
    }

    for (int i = 0; i < 10 && !stopRequested.load(std::memory_order_acquire); ++i) {
        RmdCanSdk::CanFrame frame;
        int const ret = transport.receive(frame, 100);
        if (ret <= 0) {
            continue;
        }
        if (frame.id == rxId && frame.length == 8 && frame.data[0] == expectedCmd) {
            if (replyMs != nullptr) {
                *replyMs = msBetween(start, std::chrono::steady_clock::now());
            }
            out = frame;
            return true;
        }
    }
    return false;
}

bool readStatus1(RmdCanSdk::SocketCanTransport& transport, int motorId, RmdCanSdk::Status1Feedback& status) {
    unsigned char data[8] = {0x9A, 0, 0, 0, 0, 0, 0, 0};
    RmdCanSdk::CanFrame frame;
    if (!requestReply(transport, motorId, data, 0x9A, frame)) {
        return false;
    }
    std::vector<unsigned char> bytes(frame.data.begin(), frame.data.begin() + frame.length);
    status = RmdCanSdk::parseStatus1Reply(bytes);
    return status.valid;
}

bool readStatus2(RmdCanSdk::SocketCanTransport& transport, int motorId, Status2& status) {
    unsigned char data[8] = {0x9C, 0, 0, 0, 0, 0, 0, 0};
    RmdCanSdk::CanFrame frame;
    if (!requestReply(transport, motorId, data, 0x9C, frame)) {
        return false;
    }
    status.iqA = static_cast<float>(i16le(frame.data[2], frame.data[3])) * 0.01f;
    status.speedDps = i16le(frame.data[4], frame.data[5]);
    status.angleDeg = i16le(frame.data[6], frame.data[7]);
    return true;
}

bool sendMotorRun(RmdCanSdk::SocketCanTransport& transport, int motorId) {
    unsigned char data[8] = {0x88, 0, 0, 0, 0, 0, 0, 0};
    if (transport.send(RmdCanSdk::standardTxId(motorId), data, 8) < 0) {
        std::cerr << "send failed for motor run cmd 0x88\n";
        return false;
    }
    return true;
}

CommandReply commandA4(RmdCanSdk::SocketCanTransport& transport,
                       int motorId,
                       float targetDeg,
                       int maxSpeedDps,
                       std::chrono::steady_clock::time_point& commandTime) {
    unsigned char data[8] = {0xA4, 0, 0, 0, 0, 0, 0, 0};
    putU16(data, 2, static_cast<unsigned short>(maxSpeedDps));
    putI32(data, 4, static_cast<int>(std::lround(targetDeg * 100.0f)));

    RmdCanSdk::CanFrame frame;
    CommandReply reply;
    reply.ok = requestReply(transport, motorId, data, 0xA4, frame, &reply.replyMs, &commandTime);
    if (reply.ok) {
        reply.raw = hexFrame(frame);
    }
    return reply;
}

void writeSampleHeader(std::ofstream& out) {
    out << "trial,elapsed_ms,sample_ms,target_deg,start_angle_deg,angle_deg,speed_dps,iq_a,"
           "position_delta_deg,velocity_delta_dps,target_error_deg,motion_detected,target_reached,"
           "status2_ok\n";
}

void writeSummaryHeader(std::ofstream& out) {
    out << "trial,target_deg,start_angle_deg,start_speed_dps,a4_reply_ok,a4_reply_ms,a4_reply_raw,"
           "motion_detected,motion_start_lower_ms,motion_start_upper_ms,target_reached,"
           "target_reached_lower_ms,target_reached_upper_ms,final_angle_deg,final_speed_dps,"
           "final_error_deg,samples,status2_failures,timeout_ms\n";
}

} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    std::string const channel = argc > 1 ? argv[1] : "can0";
    int const motorId = argc > 2 ? std::atoi(argv[2]) : 14;
    int const durationSec = argc > 3 ? std::atoi(argv[3]) : 300;
    int const maxSpeedDps = argc > 4 ? std::atoi(argv[4]) : 10;
    float const targetA = argc > 5 ? std::atof(argv[5]) : 0.0f;
    float const targetB = argc > 6 ? std::atof(argv[6]) : 30.0f;
    int const sampleHz = argc > 7 ? std::atoi(argv[7]) : 1000;
    std::string const outputPrefix = argc > 8 ? argv[8] : defaultOutputPrefix();

    if (durationSec <= 0) {
        std::cerr << "duration_s must be > 0\n";
        return 2;
    }
    if (maxSpeedDps <= 0 || maxSpeedDps > MaxA4SpeedDps) {
        std::cerr << "refusing max_speed outside 1.." << MaxA4SpeedDps << " dps\n";
        return 2;
    }
    if (sampleHz <= 0 || sampleHz > MaxStatusSampleHz) {
        std::cerr << "sample_hz must be in 1.." << MaxStatusSampleHz
                  << " for request/reply 0x9C sampling\n";
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
    writeSummaryHeader(summary);

    RmdCanSdk::SocketCanTransport transport;
    if (transport.open(channel) != 0) {
        std::cerr << "failed to open " << channel << "\n";
        return 2;
    }

    RmdCanSdk::Status1Feedback status1;
    if (!readStatus1(transport, motorId, status1)) {
        std::cerr << "failed to read status1\n";
        return 1;
    }
    std::cout << "status1 temp=" << status1.temperatureC
              << " mos=" << status1.mosTemperatureC
              << " brake_release_cmd=" << (status1.brakeReleaseCommandActive ? "release" : "lock")
              << " voltage=" << status1.voltageV
              << " error=0x" << std::hex << status1.errorState << std::dec << "\n";
    if (status1.errorState != 0) {
        std::cerr << "refusing motion: motor error_state=0x" << std::hex << status1.errorState << std::dec << "\n";
        return 1;
    }
    if (!status1.brakeReleaseCommandActive) {
        std::cout << "brake release command state is lock; continuing because 0x9A DATA[3] is not a mechanical brake sensor\n";
    }
    if (!sendMotorRun(transport, motorId)) {
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    Status2 initial;
    if (!readStatus2(transport, motorId, initial)) {
        std::cerr << "failed to read initial status2\n";
        return 1;
    }

    auto const testStart = std::chrono::steady_clock::now();
    auto const testDeadline = testStart + std::chrono::seconds(durationSec);
    auto const samplePeriod =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1.0 / sampleHz));

    std::cout << "A4 response test can=" << channel
              << " motor_id=" << motorId
              << " duration_s=" << durationSec
              << " max_speed_dps=" << maxSpeedDps
              << " targets=[" << targetA << "," << targetB << "]"
              << " sample_hz=" << sampleHz << "\n";
    std::cout << "initial angle=" << initial.angleDeg << "deg speed=" << initial.speedDps
              << "dps iq=" << initial.iqA << "A\n";
    std::cout << "writing samples: " << samplesPath << "\n";
    std::cout << "writing summary: " << summaryPath << "\n";
    std::cout << "thresholds: motion pos_delta>=1deg or vel_delta>=5dps, target tolerance=2deg and |speed|<=3dps\n";

    Status2 lastStatus = initial;
    int trial = 0;
    while (!stopRequested.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < testDeadline) {
        ++trial;
        float const targetDeg = (trial % 2 == 1) ? targetA : targetB;
        Status2 startStatus = lastStatus;
        readStatus2(transport, motorId, startStatus);

        std::chrono::steady_clock::time_point commandTime;
        CommandReply const reply = commandA4(transport, motorId, targetDeg, maxSpeedDps, commandTime);
        std::cout << "trial=" << trial
                  << " target=" << targetDeg
                  << "deg start_angle=" << startStatus.angleDeg
                  << "deg A4_reply=" << (reply.ok ? "true" : "false")
                  << " reply_ms=" << reply.replyMs << "\n";

        double const distanceDeg = std::fabs(static_cast<double>(startStatus.angleDeg) - static_cast<double>(targetDeg));
        int const timeoutMs = std::max(5000, static_cast<int>((distanceDeg / maxSpeedDps) * 1000.0) + 5000);
        auto const trialDeadline = std::min(commandTime + std::chrono::milliseconds(timeoutMs), testDeadline);
        auto nextSample = commandTime + samplePeriod;
        auto lastSample = commandTime;

        bool motionDetected = false;
        bool targetReached = false;
        double motionLowerMs = -1.0;
        double motionUpperMs = -1.0;
        double targetLowerMs = -1.0;
        double targetUpperMs = -1.0;
        int samplesWritten = 0;
        int status2Failures = 0;
        Status2 finalStatus = startStatus;
        double finalErrorDeg = std::fabs(static_cast<double>(startStatus.angleDeg) - static_cast<double>(targetDeg));

        while (!stopRequested.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < trialDeadline) {
            std::this_thread::sleep_until(nextSample);
            auto const sampleTime = std::chrono::steady_clock::now();
            nextSample += samplePeriod;

            Status2 status;
            bool const statusOk = readStatus2(transport, motorId, status);
            if (!statusOk) {
                ++status2Failures;
                samples << trial << ','
                        << msBetween(testStart, sampleTime) << ','
                        << msBetween(commandTime, sampleTime) << ','
                        << targetDeg << ','
                        << startStatus.angleDeg << ",,,,,,0,0,0\n";
                lastSample = sampleTime;
                continue;
            }

            finalStatus = status;
            lastStatus = status;
            double const posDelta = std::fabs(static_cast<double>(status.angleDeg - startStatus.angleDeg));
            double const velDelta = std::fabs(static_cast<double>(status.speedDps - startStatus.speedDps));
            finalErrorDeg = std::fabs(static_cast<double>(status.angleDeg) - static_cast<double>(targetDeg));

            if (!motionDetected &&
                (posDelta >= MotionPositionThresholdDeg || velDelta >= MotionVelocityThresholdDps)) {
                motionDetected = true;
                motionLowerMs = msBetween(commandTime, lastSample);
                motionUpperMs = msBetween(commandTime, sampleTime);
                std::cout << "  motion_start_window_ms=[" << motionLowerMs << ", " << motionUpperMs << "]\n";
            }
            if (!targetReached && finalErrorDeg <= TargetToleranceDeg && std::abs(status.speedDps) <= SettledSpeedDps) {
                targetReached = true;
                targetLowerMs = msBetween(commandTime, lastSample);
                targetUpperMs = msBetween(commandTime, sampleTime);
                std::cout << "  target_reached_window_ms=[" << targetLowerMs << ", " << targetUpperMs << "]"
                          << " final_angle=" << status.angleDeg << "deg\n";
            }

            samples << trial << ','
                    << std::fixed << std::setprecision(3)
                    << msBetween(testStart, sampleTime) << ','
                    << msBetween(commandTime, sampleTime) << ','
                    << targetDeg << ','
                    << startStatus.angleDeg << ','
                    << status.angleDeg << ','
                    << status.speedDps << ','
                    << status.iqA << ','
                    << posDelta << ','
                    << velDelta << ','
                    << finalErrorDeg << ','
                    << (motionDetected ? 1 : 0) << ','
                    << (targetReached ? 1 : 0) << ",1\n";
            ++samplesWritten;
            if (samplesWritten % 50 == 0) {
                samples.flush();
            }

            lastSample = sampleTime;
            if (targetReached) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                break;
            }
        }

        summary << trial << ','
                << targetDeg << ','
                << startStatus.angleDeg << ','
                << startStatus.speedDps << ','
                << (reply.ok ? 1 : 0) << ','
                << std::fixed << std::setprecision(3)
                << reply.replyMs << ','
                << '"' << reply.raw << '"' << ','
                << (motionDetected ? 1 : 0) << ','
                << motionLowerMs << ','
                << motionUpperMs << ','
                << (targetReached ? 1 : 0) << ','
                << targetLowerMs << ','
                << targetUpperMs << ','
                << finalStatus.angleDeg << ','
                << finalStatus.speedDps << ','
                << finalErrorDeg << ','
                << samplesWritten << ','
                << status2Failures << ','
                << timeoutMs << "\n";
        summary.flush();
        samples.flush();

        if (!motionDetected) {
            std::cout << "  no visible motion before timeout_ms=" << timeoutMs
                      << " final_angle=" << finalStatus.angleDeg
                      << "deg final_speed=" << finalStatus.speedDps << "dps\n";
        }
    }

    std::cout << "finished. samples=" << samplesPath << " summary=" << summaryPath << "\n";
    return stopRequested.load(std::memory_order_acquire) ? 1 : 0;
}
