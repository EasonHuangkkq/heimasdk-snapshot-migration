#include "rmd_can_sdk/rs232_imu_backend.h"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

namespace {

void printUsage(char const* program) {
    std::cerr << "usage: " << program << " <device> <baudrate> [samples]\n";
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        printUsage(argv[0]);
        return 2;
    }

    char const* device = argv[1];
    int const baudrate = std::atoi(argv[2]);
    int samples = argc >= 4 ? std::atoi(argv[3]) : 200;
    if (baudrate <= 0 || samples <= 0) {
        printUsage(argv[0]);
        return 2;
    }

    RmdCanSdk::ImuSnapshotBuffer buffer;
    RmdCanSdk::Rs232ImuBackend backend(buffer);
    if (backend.start(device, baudrate, "YeSense") != 0) {
        std::cerr << "starting YeSense IMU failed: " << device << " baudrate " << baudrate << "\n";
        return 1;
    }

    for (int i = 0; i < samples; ++i) {
        DriverSDK::imuStruct imu;
        buffer.readInto(imu);
        std::cout << "rpy[rad]: " << imu.rpy[0] << " " << imu.rpy[1] << " " << imu.rpy[2]
                  << " gyr[rad/s]: " << imu.gyr[0] << " " << imu.gyr[1] << " " << imu.gyr[2]
                  << " acc: " << imu.acc[0] << " " << imu.acc[1] << " " << imu.acc[2] << "\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    backend.stop();
    return 0;
}
