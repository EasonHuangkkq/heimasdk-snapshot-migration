#include "rmd_can_sdk/heima_driver_sdk.h"

#include "rmd_can_sdk/common.h"
#include "rmd_can_sdk/rmd_can_backend.h"
#include "rmd_can_sdk/rmd_can_config.h"
#include "rmd_can_sdk/rmd_ethercat_backend.h"
#include "rmd_can_sdk/rmd_ethercat_mt_device.h"
#include "rmd_can_sdk/rmd_ethercat_pdo.h"
#include "rmd_can_sdk/rmd_ethercat_target_frame.h"
#include "rmd_can_sdk/rmd_motor_backend.h"
#include "rmd_can_sdk/rmd_motor_registry.h"
#include "rmd_can_sdk/rmd_realtime_core.h"
#include "rmd_can_sdk/rmd_safety.h"
#include "rmd_can_sdk/rmd_types.h"
#include "rmd_can_sdk/rs232_imu_backend.h"

#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

namespace {

constexpr int Unsupported = std::numeric_limits<int>::max();

DriverSDK::motorActualStruct toPublicActual(RmdCanSdk::MotorActual const& actual,
                                            RmdCanSdk::MotorParameters const& params) {
    DriverSDK::motorActualStruct out;
    out.pos = params.polarity * (actual.pos - params.countBias);
    out.vel = params.polarity * actual.vel;
    out.tor = params.polarity * actual.tor;
    out.temp = actual.temp;
    out.driveTemp = actual.driveTemp;
    out.voltage = actual.voltage;
    out.statusWord = actual.statusWord;
    out.errorCode = actual.errorCode;
    return out;
}

DriverSDK::motorActualStruct copyPublicActual(RmdCanSdk::MotorActual const& actual) {
    DriverSDK::motorActualStruct out;
    out.pos = actual.pos;
    out.vel = actual.vel;
    out.tor = actual.tor;
    out.temp = actual.temp;
    out.driveTemp = actual.driveTemp;
    out.voltage = actual.voltage;
    out.statusWord = actual.statusWord;
    out.errorCode = actual.errorCode;
    return out;
}

RmdCanSdk::MotorTarget toCanTarget(DriverSDK::motorTargetStruct const& target,
                                  RmdCanSdk::MotorParameters const& params) {
    RmdCanSdk::MotorTarget out;
    float pos = RmdCanSdk::clamp(target.pos, params.minimumPosition, params.maximumPosition);
    float tor = RmdCanSdk::clamp(target.tor, -params.maximumTorque, params.maximumTorque);
    out.pos = params.polarity * pos + params.countBias;
    out.vel = params.polarity * target.vel;
    out.tor = params.polarity * tor;
    out.kp = target.kp;
    out.kd = target.kd;
    out.enabled = target.enabled;
    return out;
}

RmdCanSdk::MotorTarget toEthercatTarget(DriverSDK::motorTargetStruct const& target,
                                        RmdCanSdk::MotorParameters const& params,
                                        char mode,
                                        unsigned short maxCurrent) {
    RmdCanSdk::MotorTarget out;
    out.pos = RmdCanSdk::clamp(target.pos, params.minimumPosition, params.maximumPosition);
    out.vel = target.vel;
    out.tor = RmdCanSdk::clamp(target.tor, -params.maximumTorque, params.maximumTorque);
    out.kp = target.kp;
    out.kd = target.kd;
    out.mode = static_cast<int>(mode);
    out.enabled = target.enabled;
    out.maxCurrent = static_cast<int>(maxCurrent);
    return out;
}

} // namespace

namespace DriverSDK {

class DriverSDK::Impl {
    struct BackendRuntime {
        std::size_t backendIndex = 0;
        std::unique_ptr<RmdCanSdk::FrameBuffer<RmdCanSdk::MotorTargetFrame>> targetBuffer;
        std::unique_ptr<RmdCanSdk::FrameBuffer<RmdCanSdk::EthercatPackedTargetFrame>> ethercatTargetBuffer;
        std::unique_ptr<RmdCanSdk::FrameBuffer<RmdCanSdk::MotorActualFrame>> actualBuffer;
        std::unique_ptr<RmdCanSdk::MotorBackend> backend;
    };

public:
    ~Impl() {
        stop();
    }

    void init(char const* xmlFile) {
        stop();
        config_ = RmdCanSdk::loadConfig(xmlFile);
        registry_ = RmdCanSdk::MotorRegistry::fromConfig(config_);
        ensureOperatingModes();
        ensureMaxCurrents();

        if (config_.imu.enabled) {
            imuBackend_ = std::make_unique<RmdCanSdk::Rs232ImuBackend>(imuBuffer_);
            char const* type =
                config_.imu.normalizedType.empty() ? config_.imu.type.c_str() : config_.imu.normalizedType.c_str();
            if (imuBackend_->start(config_.imu.device.c_str(), config_.imu.baudrate, type) != 0) {
                stop();
                throw std::runtime_error("starting IMU backend failed");
            }
        }

        RmdCanSdk::MotorTargetFrame initialTargets;
        initialTargets.motorCount = static_cast<std::size_t>(config_.totalMotorCount);
        RmdCanSdk::EthercatPackedTargetFrame initialEthercatTargets;
        initialEthercatTargets.motorCount = static_cast<std::size_t>(config_.totalMotorCount);

        for (std::size_t i = 0; i < registry_.backends().size(); ++i) {
            auto const& group = registry_.backends()[i];
            RmdCanSdk::MotorActualFrame initialActuals;
            initialActuals.motorCount = static_cast<std::size_t>(config_.totalMotorCount);
            for (std::size_t registryIndex : group.motorIndexes) {
                auto const& motor = registry_.motors().at(registryIndex);
                initialActuals.valid.set(motor.globalIndex);
                initialActuals.actuals[motor.globalIndex].statusWord = 0x0000;
            }

            BackendRuntime runtime;
            runtime.backendIndex = i;
            runtime.actualBuffer =
                std::make_unique<RmdCanSdk::FrameBuffer<RmdCanSdk::MotorActualFrame>>(initialActuals);
            if (group.bus == RmdCanSdk::MotorBus::Can) {
                runtime.targetBuffer =
                    std::make_unique<RmdCanSdk::FrameBuffer<RmdCanSdk::MotorTargetFrame>>(initialTargets);
                runtime.backend = std::make_unique<RmdCanSdk::RmdCanBackend>(
                    config_, registry_, i, *runtime.targetBuffer, *runtime.actualBuffer);
            } else if (group.bus == RmdCanSdk::MotorBus::Ethercat) {
                runtime.ethercatTargetBuffer =
                    std::make_unique<RmdCanSdk::FrameBuffer<RmdCanSdk::EthercatPackedTargetFrame>>(
                        initialEthercatTargets);
                runtime.backend = std::make_unique<RmdCanSdk::RmdEthercatBackend>(
                    config_, registry_, i, *runtime.ethercatTargetBuffer, *runtime.actualBuffer, operatingModes_);
            }
            backendRuntimes_.push_back(std::move(runtime));
        }

        for (auto& runtime : backendRuntimes_) {
            if (runtime.backend->start() != 0) {
                stop();
                throw std::runtime_error("starting motor backend failed");
            }
        }
        initialized_ = true;
    }

    int totalMotorCount() const {
        return config_.totalMotorCount;
    }

    std::vector<int> activeMotors() const {
        std::vector<int> ret;
        for (auto const& motor : registry_.motors()) {
            ret.push_back(motor.alias - 1);
        }
        return ret;
    }

    int setTargets(std::vector<motorTargetStruct> const& data) {
        if (!initialized_ || static_cast<int>(data.size()) != config_.totalMotorCount) {
            return std::numeric_limits<int>::min();
        }
        RmdCanSdk::MotorTargetFrame frame;
        RmdCanSdk::EthercatPackedTargetFrame ethercatFrame;
        std::uint64_t const sequence = ++targetSequence_;
        RmdCanSdk::RealtimeClock::time_point const timestamp = RmdCanSdk::RealtimeClock::now();
        frame.sequence = sequence;
        frame.timestamp = timestamp;
        frame.motorCount = static_cast<std::size_t>(config_.totalMotorCount);
        ethercatFrame.sequence = sequence;
        ethercatFrame.timestamp = timestamp;
        ethercatFrame.motorCount = static_cast<std::size_t>(config_.totalMotorCount);
        for (auto const& motor : registry_.motors()) {
            std::size_t const index = motor.globalIndex;
            if (motor.bus == RmdCanSdk::MotorBus::Ethercat) {
                RmdCanSdk::MotorTarget target =
                    toEthercatTarget(data[index], motor.parameters, operatingModes_.at(index), maxCurrents_.at(index));
                RmdCanSdk::EthercatMtDeviceRxProfile const rxProfile =
                    RmdCanSdk::ethercatMtDeviceRxProfileForMode(operatingModes_.at(index));
                ethercatFrame.targets[index] =
                    RmdCanSdk::packEthercatTargetForRealtime(target, motor.parameters, rxProfile);
                ethercatFrame.valid.set(index);
            } else {
                auto converted = toCanTarget(data[index], motor.parameters);
                auto limited = RmdCanSdk::limitTarget(converted, motor.parameters);
                frame.targets[index] = limited.target;
                frame.valid.set(index);
            }
        }
        for (auto& runtime : backendRuntimes_) {
            if (runtime.targetBuffer != nullptr) {
                runtime.targetBuffer->publish(frame);
            }
            if (runtime.ethercatTargetBuffer != nullptr) {
                runtime.ethercatTargetBuffer->publish(ethercatFrame);
            }
        }
        return 0;
    }

    int getActuals(std::vector<motorActualStruct>& data) {
        if (!initialized_ || static_cast<int>(data.size()) != config_.totalMotorCount) {
            return std::numeric_limits<int>::min();
        }
        int staleCount = 0;
        for (auto& item : data) {
            item = motorActualStruct{};
        }
        for (auto const& runtime : backendRuntimes_) {
            RmdCanSdk::MotorActualFrame frame;
            runtime.actualBuffer->readInto(frame);
            auto const& group = registry_.backends().at(runtime.backendIndex);
            for (std::size_t registryIndex : group.motorIndexes) {
                auto const& motor = registry_.motors().at(registryIndex);
                std::size_t const index = motor.globalIndex;
                data[index] = motor.bus == RmdCanSdk::MotorBus::Ethercat
                                  ? copyPublicActual(frame.actuals[index])
                                  : toPublicActual(frame.actuals[index], motor.parameters);
                if (!frame.valid.test(index) || frame.stale.test(index)) {
                    staleCount++;
                }
            }
        }
        return staleCount == 0 ? 0 : -staleCount;
    }

    int getEncoderCounts(std::vector<int>& data) {
        if (!initialized_ || static_cast<int>(data.size()) != config_.totalMotorCount) {
            return std::numeric_limits<int>::min();
        }
        int staleCount = 0;
        std::fill(data.begin(), data.end(), 0);
        for (auto const& runtime : backendRuntimes_) {
            RmdCanSdk::MotorActualFrame frame;
            runtime.actualBuffer->readInto(frame);
            auto const& group = registry_.backends().at(runtime.backendIndex);
            for (std::size_t registryIndex : group.motorIndexes) {
                auto const& motor = registry_.motors().at(registryIndex);
                std::size_t const index = motor.globalIndex;
                data[index] = static_cast<int>(frame.actuals[index].encoderCount);
                if (!frame.valid.test(index) || frame.stale.test(index)) {
                    staleCount++;
                }
            }
        }
        return staleCount == 0 ? 0 : -staleCount;
    }

    int setCountBias(std::vector<int> const& countBias) {
        if (initialized_) {
            return -1;
        }
        if (static_cast<int>(countBias.size()) != config_.totalMotorCount) {
            return -1;
        }
        for (auto& motor : config_.motors) {
            motor.parameters.countBias = static_cast<float>(countBias[static_cast<std::size_t>(motor.alias - 1)]);
        }
        registry_ = RmdCanSdk::MotorRegistry::fromConfig(config_);
        return 0;
    }

    void getImu(imuStruct& data) {
        imuBuffer_.readInto(data);
    }

    int setMode(std::vector<char> const& modes) {
        if (initialized_) {
            return -1;
        }
        operatingModes_ = modes;
        return 0;
    }

    int setMaxCurrents(std::vector<unsigned short> const& maxCurrents) {
        if (initialized_) {
            return -1;
        }
        maxCurrents_ = maxCurrents;
        return 0;
    }

private:
    void ensureOperatingModes() {
        std::size_t const count = static_cast<std::size_t>(config_.totalMotorCount);
        if (operatingModes_.empty()) {
            operatingModes_.assign(count, static_cast<char>(8));
            return;
        }
        if (operatingModes_.size() != count) {
            throw std::runtime_error("configured motor mode count does not match total motor count");
        }
    }

    void ensureMaxCurrents() {
        std::size_t const count = static_cast<std::size_t>(config_.totalMotorCount);
        if (maxCurrents_.empty()) {
            maxCurrents_.assign(count, 1000);
            return;
        }
        if (maxCurrents_.size() != count) {
            throw std::runtime_error("configured max current count does not match total motor count");
        }
    }

    void stop() {
        for (auto& runtime : backendRuntimes_) {
            if (runtime.backend != nullptr) {
                runtime.backend->stop();
            }
        }
        backendRuntimes_.clear();
        if (imuBackend_ != nullptr) {
            imuBackend_->stop();
            imuBackend_.reset();
        }
        initialized_ = false;
    }

    RmdCanSdk::Config config_;
    RmdCanSdk::MotorRegistry registry_;
    RmdCanSdk::ImuSnapshotBuffer imuBuffer_;
    std::unique_ptr<RmdCanSdk::Rs232ImuBackend> imuBackend_;
    std::vector<BackendRuntime> backendRuntimes_;
    std::vector<char> operatingModes_;
    std::vector<unsigned short> maxCurrents_;
    std::uint64_t targetSequence_ = 0;
    bool initialized_ = false;
};

motorSDOClass::motorSDOClass(int i) : i(i) {}

motorREGClass::motorREGClass(int i) : i(i) {}

DriverSDK& DriverSDK::instance() {
    static DriverSDK instance;
    return instance;
}

DriverSDK::DriverSDK() : impl_(new Impl()) {}

DriverSDK::~DriverSDK() {
    delete impl_;
}

void DriverSDK::setCPU(unsigned short) {}

int DriverSDK::setCPUs(std::vector<unsigned short> const& cpus, std::string const& bus) {
    return bus == "CAN" && cpus.size() == 3 ? 0 : -1;
}

void DriverSDK::setMaxCurr(std::vector<unsigned short> const& maxCurr) {
    (void)impl_->setMaxCurrents(maxCurr);
}

int DriverSDK::setMode(std::vector<char> const& mode) {
    return impl_->setMode(mode);
}

void DriverSDK::init(char const* xmlFile) {
    try {
        impl_->init(xmlFile);
    } catch (std::exception const& e) {
        std::cerr << "rmd_can_sdk init failed: " << e.what() << "\n";
        throw;
    }
}

int DriverSDK::getLeftDigitNr() {
    return 0;
}

int DriverSDK::getRightDigitNr() {
    return 0;
}

int DriverSDK::getTotalMotorNr() {
    return impl_->totalMotorCount();
}

std::vector<int> DriverSDK::getActiveMotors() {
    return impl_->activeMotors();
}

int DriverSDK::setCntBias(std::vector<int> const& cntBias) {
    return impl_->setCountBias(cntBias);
}

int DriverSDK::fillSDO(motorSDOClass& data, char const*) {
    data.state = -1;
    return Unsupported;
}

void DriverSDK::getIMU(imuStruct& data) {
    impl_->getImu(data);
}

int DriverSDK::getSensor(std::vector<sensorStruct>& data) {
    for (auto& sensor : data) {
        sensor = sensorStruct{};
    }
    return Unsupported;
}

int DriverSDK::setDigitTarget(std::vector<digitTargetStruct> const&) {
    return Unsupported;
}

int DriverSDK::getDigitActual(std::vector<digitActualStruct>&) {
    return Unsupported;
}

int DriverSDK::setMotorTarget(std::vector<motorTargetStruct> const& data) {
    return impl_->setTargets(data);
}

int DriverSDK::getMotorActual(std::vector<motorActualStruct>& data) {
    return impl_->getActuals(data);
}

int DriverSDK::getEncoderCount(std::vector<int>& data) {
    return impl_->getEncoderCounts(data);
}

int DriverSDK::sendMotorSDORequest(motorSDOClass const&) {
    return Unsupported;
}

int DriverSDK::recvMotorSDOResponse(motorSDOClass& data) {
    data.state = -1;
    return Unsupported;
}

int DriverSDK::sendMotorREGRequest(motorREGClass const&) {
    return Unsupported;
}

int DriverSDK::recvMotorREGResponse(motorREGClass&) {
    return Unsupported;
}

int DriverSDK::calibrate(int) {
    return Unsupported;
}

void DriverSDK::advance() {}

std::string DriverSDK::version() {
    return "0.1.0-rmd-can";
}

} // namespace DriverSDK
