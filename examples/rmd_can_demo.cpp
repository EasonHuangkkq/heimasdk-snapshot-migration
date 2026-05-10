#include "rmd_can_sdk/heima_driver_sdk.h"

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

int main(int argc, char** argv) {
    char const* config = argc > 1 ? argv[1] : "config/example_rmd_can.xml";
    auto& sdk = DriverSDK::DriverSDK::instance();
    sdk.init(config);

    int const count = sdk.getTotalMotorNr();
    std::vector<DriverSDK::motorTargetStruct> targets(static_cast<std::size_t>(count));
    std::vector<DriverSDK::motorActualStruct> actuals(static_cast<std::size_t>(count));

    for (int active : sdk.getActiveMotors()) {
        targets[static_cast<std::size_t>(active)].enabled = 1;
        targets[static_cast<std::size_t>(active)].kp = 0.0f;
        targets[static_cast<std::size_t>(active)].kd = 0.0f;
    }
    sdk.setMotorTarget(targets);

    for (int i = 0; i < 100; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        sdk.getMotorActual(actuals);
        for (int active : sdk.getActiveMotors()) {
            auto const& actual = actuals[static_cast<std::size_t>(active)];
            std::cout << "motor " << active + 1
                      << " pos=" << actual.pos
                      << " vel=" << actual.vel
                      << " tor=" << actual.tor
                      << " status=0x" << std::hex << actual.statusWord << std::dec
                      << "\n";
        }
    }

    for (auto& target : targets) {
        target.enabled = 0;
    }
    sdk.setMotorTarget(targets);
    return 0;
}
