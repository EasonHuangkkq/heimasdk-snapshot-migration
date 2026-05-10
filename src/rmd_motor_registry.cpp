#include "rmd_can_sdk/rmd_motor_registry.h"

#include <map>
#include <utility>

namespace RmdCanSdk {

MotorRegistry MotorRegistry::fromConfig(Config const& config) {
    MotorRegistry registry;
    registry.totalMotorCount_ = config.totalMotorCount;

    std::map<std::pair<MotorBus, int>, std::size_t> backendByKey;
    for (auto const& motor : config.motors) {
        if (motor.alias <= 0 || motor.alias > config.totalMotorCount) {
            throw std::runtime_error("motor alias outside total motor count");
        }

        std::pair<MotorBus, int> key{motor.bus, motor.master};
        auto backendIt = backendByKey.find(key);
        if (backendIt == backendByKey.end()) {
            BackendGroup group;
            group.bus = motor.bus;
            group.master = motor.master;
            registry.backends_.push_back(group);
            backendIt = backendByKey.emplace(key, registry.backends_.size() - 1).first;
        }

        RuntimeMotor runtime;
        runtime.alias = motor.alias;
        runtime.globalIndex = static_cast<std::size_t>(motor.alias - 1);
        runtime.backendIndex = backendIt->second;
        runtime.backendLocalIndex = registry.backends_[backendIt->second].motorIndexes.size();
        runtime.bus = motor.bus;
        runtime.master = motor.master;
        runtime.motorId = motor.motorId;
        runtime.slaveId = motor.slaveId;
        runtime.ethercatSlave = motor.ethercatSlave;
        runtime.domain = motor.domain;
        runtime.type = motor.type;
        runtime.parameters = motor.parameters;

        registry.backends_[backendIt->second].motorIndexes.push_back(registry.motors_.size());
        registry.motors_.push_back(runtime);
    }

    return registry;
}

RuntimeMotor const& MotorRegistry::motorByAlias(int alias) const {
    for (auto const& motor : motors_) {
        if (motor.alias == alias) {
            return motor;
        }
    }
    throw std::out_of_range("unknown motor alias");
}

} // namespace RmdCanSdk
