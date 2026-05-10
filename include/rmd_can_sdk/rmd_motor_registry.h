#pragma once

#include "rmd_can_sdk/rmd_types.h"

#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace RmdCanSdk {

struct RuntimeMotor {
    int alias = 0;
    std::size_t globalIndex = 0;
    std::size_t backendIndex = 0;
    std::size_t backendLocalIndex = 0;
    MotorBus bus = MotorBus::Can;
    int master = 0;
    int motorId = 0;
    int slaveId = 0;
    int ethercatSlave = 0;
    int domain = 0;
    std::string type;
    MotorParameters parameters;
};

struct BackendGroup {
    MotorBus bus = MotorBus::Can;
    int master = 0;
    std::vector<std::size_t> motorIndexes;
};

class MotorRegistry {
public:
    static MotorRegistry fromConfig(Config const& config);

    int totalMotorCount() const { return totalMotorCount_; }
    std::size_t backendCount() const { return backends_.size(); }
    std::vector<RuntimeMotor> const& motors() const { return motors_; }
    std::vector<BackendGroup> const& backends() const { return backends_; }
    RuntimeMotor const& motorByAlias(int alias) const;

private:
    int totalMotorCount_ = 0;
    std::vector<RuntimeMotor> motors_;
    std::vector<BackendGroup> backends_;
};

} // namespace RmdCanSdk
