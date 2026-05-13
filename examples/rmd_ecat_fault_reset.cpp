#include "rmd_can_sdk/heima_driver_sdk.h"
#include "rmd_can_sdk/rmd_can_config.h"

#include <chrono>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <thread>
#include <vector>

int main(int argc, char** argv) {
    if (argc < 2 || argc > 4) {
        std::cerr << "usage: " << argv[0] << " <config.xml> [cycles] [mode]\n";
        return 2;
    }

    char const* configPath = argv[1];
    int const cycles = argc > 2 ? std::atoi(argv[2]) : 80;
    int const mode = argc > 3 ? std::atoi(argv[3]) : 8;
    if (cycles <= 0) {
        std::cerr << "cycles must be positive\n";
        return 2;
    }
    if (mode < -128 || mode > 127) {
        std::cerr << "mode must fit int8\n";
        return 2;
    }

    try {
        int const countFromConfig = RmdCanSdk::loadConfig(configPath).totalMotorCount;
        auto& sdk = DriverSDK::DriverSDK::instance();
        std::vector<char> modes(static_cast<std::size_t>(countFromConfig), static_cast<char>(mode));
        sdk.setMode(modes);
        sdk.init(configPath);

        int const count = sdk.getTotalMotorNr();
        std::vector<DriverSDK::motorTargetStruct> targets(static_cast<std::size_t>(count));
        std::vector<DriverSDK::motorActualStruct> actuals(static_cast<std::size_t>(count));

        for (int active : sdk.getActiveMotors()) {
            targets[static_cast<std::size_t>(active)].enabled = -1;
        }
        for (int i = 0; i < cycles; ++i) {
            sdk.setMotorTarget(targets);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        for (auto& target : targets) {
            target.enabled = 0;
        }
        for (int i = 0; i < 40; ++i) {
            sdk.setMotorTarget(targets);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        sdk.getMotorActual(actuals);
        std::cout << "fault reset sent";
        for (int active : sdk.getActiveMotors()) {
            auto const& actual = actuals[static_cast<std::size_t>(active)];
            std::cout << " | motor " << active + 1
                      << " status=0x" << std::hex << actual.statusWord
                      << " err=0x" << actual.errorCode << std::dec;
        }
        std::cout << "\n";
    } catch (std::exception const& ex) {
        std::cerr << "fault reset failed: " << ex.what() << "\n";
        return 1;
    }

    return 0;
}
