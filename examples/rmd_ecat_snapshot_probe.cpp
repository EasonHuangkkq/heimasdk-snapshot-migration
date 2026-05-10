#include "rmd_can_sdk/heima_driver_sdk.h"
#include "rmd_can_sdk/rmd_can_config.h"

#include <chrono>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

int main(int argc, char** argv) {
    if (argc < 2 || argc > 4) {
        std::cerr << "usage: " << argv[0] << " <config.xml> [samples] [mode]\n";
        return 2;
    }

    char const* config = argv[1];
    int const samples = argc > 2 ? std::atoi(argv[2]) : 200;
    int const mode = argc > 3 ? std::atoi(argv[3]) : 8;
    if (samples <= 0) {
        std::cerr << "samples must be positive\n";
        return 2;
    }
    if (mode < -128 || mode > 127) {
        std::cerr << "mode must fit int8\n";
        return 2;
    }

    try {
        int const countFromConfig = RmdCanSdk::loadConfig(config).totalMotorCount;
        auto& sdk = DriverSDK::DriverSDK::instance();
        std::vector<char> modes(static_cast<std::size_t>(countFromConfig), static_cast<char>(mode));
        sdk.setMode(modes);
        sdk.init(config);

        int const count = sdk.getTotalMotorNr();
        std::vector<DriverSDK::motorActualStruct> actuals(static_cast<std::size_t>(count));
        int consecutiveReady = 0;
        int maxConsecutiveReady = 0;

        for (int sample = 0; sample < samples; ++sample) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            int const actualStatus = sdk.getMotorActual(actuals);

            std::cout << "sample " << sample;
            bool sampleReady = actualStatus == 0;
            for (int active : sdk.getActiveMotors()) {
                auto const& actual = actuals[static_cast<std::size_t>(active)];
                sampleReady = sampleReady && actual.statusWord != 0xffff && actual.errorCode == 0;
                std::cout << " | motor " << active + 1
                          << " pos=" << actual.pos
                          << " vel=" << actual.vel
                          << " tor=" << actual.tor
                          << " status=0x" << std::hex << std::setw(4) << std::setfill('0')
                          << actual.statusWord
                          << " err=0x" << std::setw(4) << actual.errorCode
                          << std::dec << std::setfill(' ');
            }
            std::cout << "\n";
            if (sampleReady) {
                ++consecutiveReady;
                if (consecutiveReady > maxConsecutiveReady) {
                    maxConsecutiveReady = consecutiveReady;
                }
            } else {
                consecutiveReady = 0;
            }
        }

        if (consecutiveReady < 10) {
            std::cerr << "EtherCAT feedback was not stable at the end; final consecutive ready samples="
                      << consecutiveReady << " max consecutive ready samples=" << maxConsecutiveReady << "\n";
            return 1;
        }
    } catch (std::exception const& ex) {
        std::cerr << "probe failed: " << ex.what() << "\n";
        return 1;
    }

    return 0;
}
