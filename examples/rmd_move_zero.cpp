#include "rmd_can_sdk/rmd_can_transport.h"
#include "rmd_can_sdk/rmd_protocol.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr int MaxA4SpeedDps = 100;

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
    int const txId = RmdCanSdk::standardTxId(motorId);
    int const rxId = RmdCanSdk::standardRxId(motorId);
    if (transport.send(txId, data, 8) < 0) {
        std::cerr << "send failed for cmd 0x" << std::hex << static_cast<int>(data[0]) << std::dec << "\n";
        return false;
    }
    for (int i = 0; i < 10; ++i) {
        RmdCanSdk::CanFrame frame;
        int ret = transport.receive(frame, 100);
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

bool readStatus1(RmdCanSdk::SocketCanTransport& transport, int motorId, RmdCanSdk::Status1Feedback& status) {
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

} // namespace

int main(int argc, char** argv) {
    std::string channel = argc > 1 ? argv[1] : "can0";
    int motorId = argc > 2 ? std::atoi(argv[2]) : 14;
    float targetDeg = argc > 3 ? std::atof(argv[3]) : 0.0f;
    int maxSpeedDps = argc > 4 ? std::atoi(argv[4]) : 10;
    if (maxSpeedDps <= 0 || maxSpeedDps > MaxA4SpeedDps) {
        std::cerr << "refusing max_speed outside 1.." << MaxA4SpeedDps << " dps\n";
        return 2;
    }

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
    if (status1.errorState != 0) {
        std::cerr << "refusing motion: motor error_state=0x" << std::hex << status1.errorState << std::dec << "\n";
        return 1;
    }
    if (!status1.brakeReleaseCommandActive) {
        std::cout << "brake release command state is lock; continuing because 0x9A DATA[3] is not a mechanical brake sensor\n";
    }

    int angleDeg = 0;
    int speedDps = 0;
    float iqA = 0.0f;
    if (!readStatus2(transport, motorId, angleDeg, speedDps, iqA)) {
        std::cerr << "failed to read status2\n";
        return 1;
    }

    if (!commandA4(transport, motorId, targetDeg, maxSpeedDps)) {
        std::cerr << "A4 command did not receive expected reply\n";
        return 1;
    }

    auto const deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
    while (std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if (!readStatus2(transport, motorId, angleDeg, speedDps, iqA)) {
            continue;
        }
        if (std::fabs(static_cast<float>(angleDeg) - targetDeg) <= 1.0f && std::abs(speedDps) <= 2) {
            std::cout << "target reached\n";
            return 0;
        }
    }
    std::cerr << "timeout before reaching target\n";
    return 1;
}
