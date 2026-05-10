#include "rmd_can_sdk/rmd_can_transport.h"
#include "rmd_can_sdk/rmd_protocol.h"

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

namespace {

void printFrame(char const* prefix, int id, unsigned char const* data, int length) {
    std::cout << prefix << " " << std::hex << std::uppercase << std::setw(3) << std::setfill('0') << id
              << " [" << std::dec << length << "]";
    for (int i = 0; i < length; ++i) {
        std::cout << " " << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                  << static_cast<int>(data[i]);
    }
    std::cout << std::dec << "\n";
}

bool sendRead(RmdCanSdk::SocketCanTransport& transport, int motorId, unsigned char cmd) {
    unsigned char data[8] = {cmd, 0, 0, 0, 0, 0, 0, 0};
    int const txId = RmdCanSdk::standardTxId(motorId);
    int const rxId = RmdCanSdk::standardRxId(motorId);
    if (transport.send(txId, data, 8) < 0) {
        std::cerr << "send failed for cmd 0x" << std::hex << static_cast<int>(cmd) << std::dec << "\n";
        return false;
    }
    printFrame("TX", txId, data, 8);
    for (int tries = 0; tries < 10; ++tries) {
        RmdCanSdk::CanFrame frame;
        int ret = transport.receive(frame, 100);
        if (ret <= 0) {
            continue;
        }
        printFrame("RX", frame.id, frame.data.data(), frame.length);
        if (frame.id == rxId && frame.length == 8 && frame.data[0] == cmd) {
            return true;
        }
    }
    std::cout << "timeout waiting for RX " << std::hex << std::uppercase << rxId
              << " cmd 0x" << static_cast<int>(cmd) << std::dec << "\n";
    return false;
}

} // namespace

int main(int argc, char** argv) {
    std::string channel = argc > 1 ? argv[1] : "can0";
    int motorId = argc > 2 ? std::atoi(argv[2]) : 14;
    RmdCanSdk::SocketCanTransport transport;
    if (transport.open(channel) != 0) {
        std::cerr << "failed to open " << channel << "\n";
        return 2;
    }
    bool ok = true;
    ok = sendRead(transport, motorId, 0x9A) && ok;
    ok = sendRead(transport, motorId, 0x9C) && ok;
    ok = sendRead(transport, motorId, 0x92) && ok;
    return ok ? 0 : 1;
}
