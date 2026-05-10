#include "rmd_can_sdk/rmd_can_config.h"

#include "rmd_can_sdk/rmd_ethercat_mt_device.h"

#include "tinyxml2.h"

#include <algorithm>
#include <cctype>
#include <map>
#include <stdexcept>

namespace RmdCanSdk {
namespace {

tinyxml2::XMLElement* requiredChild(tinyxml2::XMLElement* parent, char const* name) {
    if (parent == nullptr) {
        throw std::runtime_error(std::string("missing parent for ") + name);
    }
    tinyxml2::XMLElement* child = parent->FirstChildElement(name);
    if (child == nullptr) {
        throw std::runtime_error(std::string("missing XML element ") + name);
    }
    return child;
}

float requiredFloat(tinyxml2::XMLElement* parent, char const* name) {
    tinyxml2::XMLElement* child = requiredChild(parent, name);
    return child->FloatText();
}

bool startsWith(std::string const& value, char const* prefix) {
    std::string const p(prefix);
    return value.size() >= p.size() && value.compare(0, p.size(), p) == 0;
}

std::string normalizeUpper(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::toupper(ch));
    });
    return value;
}

MotorBus parseMotorBus(char const* bus) {
    if (bus == nullptr) {
        throw std::runtime_error("missing motor bus attribute");
    }
    std::string value = normalizeUpper(bus);
    if (value == "CAN") {
        return MotorBus::Can;
    }
    if (value == "ECAT" || value == "ETHERCAT") {
        return MotorBus::Ethercat;
    }
    throw std::runtime_error("unsupported motor bus attribute: " + value);
}

int firstIntAttribute(tinyxml2::XMLElement* element,
                      char const* first,
                      char const* second,
                      char const* third,
                      int fallback) {
    if (element->Attribute(first) != nullptr) {
        return element->IntAttribute(first);
    }
    if (second != nullptr && element->Attribute(second) != nullptr) {
        return element->IntAttribute(second);
    }
    if (third != nullptr && element->Attribute(third) != nullptr) {
        return element->IntAttribute(third);
    }
    return fallback;
}

MotorParameters readMotorParameters(tinyxml2::XMLElement* motor) {
    MotorParameters parameters;
    parameters.polarity = requiredFloat(motor, "Polarity");
    parameters.countBias = requiredFloat(motor, "CountBias");
    parameters.encoderResolution = requiredFloat(motor, "EncoderResolution");
    parameters.gearRatioTor = requiredFloat(motor, "GearRatioTor");
    parameters.gearRatioPosVel = requiredFloat(motor, "GearRatioPosVel");
    parameters.ratedCurrent = requiredFloat(motor, "RatedCurrent");
    parameters.torqueConstant = requiredFloat(motor, "TorqueConstant");
    parameters.ratedTorque = requiredFloat(motor, "RatedTorque");
    parameters.maximumTorque = requiredFloat(motor, "MaximumTorque");
    parameters.minimumPosition = requiredFloat(motor, "MinimumPosition");
    parameters.maximumPosition = requiredFloat(motor, "MaximumPosition");
    return parameters;
}

std::map<int, MotorParameters> readMotorParameterMap(tinyxml2::XMLElement* root, int& totalMotorCount) {
    std::map<int, MotorParameters> ret;
    totalMotorCount = 0;
    tinyxml2::XMLElement* motors = requiredChild(root, "Motors");
    tinyxml2::XMLElement* motor = motors->FirstChildElement("Motor");
    while (motor != nullptr) {
        int const alias = motor->IntAttribute("alias", 0);
        if (alias > 0) {
            ret[alias] = readMotorParameters(motor);
            if (alias > totalMotorCount) {
                totalMotorCount = alias;
            }
        }
        motor = motor->NextSiblingElement("Motor");
    }
    return ret;
}

void upsertMotor(Config& config, MotorConfig const& motor) {
    auto existing = std::find_if(config.motors.begin(), config.motors.end(), [&](MotorConfig const& item) {
        return item.alias == motor.alias;
    });
    if (existing == config.motors.end()) {
        config.motors.push_back(motor);
    } else {
        *existing = motor;
    }
}

void readExplicitMotorPlacements(tinyxml2::XMLElement* root,
                                 std::map<int, MotorParameters> const& motorParameters,
                                 Config& config) {
    tinyxml2::XMLElement* motors = root->FirstChildElement("Motors");
    if (motors == nullptr) {
        return;
    }

    tinyxml2::XMLElement* motorElement = motors->FirstChildElement("Motor");
    while (motorElement != nullptr) {
        char const* busAttr = motorElement->Attribute("bus");
        if (busAttr == nullptr) {
            motorElement = motorElement->NextSiblingElement("Motor");
            continue;
        }

        MotorConfig motor;
        motor.alias = motorElement->IntAttribute("alias", 0);
        motor.master = motorElement->IntAttribute("master", 0);
        motor.bus = parseMotorBus(busAttr);
        char const* typeAttr = motorElement->Attribute("type");
        motor.type = typeAttr == nullptr ? "" : typeAttr;
        motor.domain = motorElement->IntAttribute("domain", 0);

        auto params = motorParameters.find(motor.alias);
        if (motor.alias <= 0 || params == motorParameters.end()) {
            throw std::runtime_error("invalid explicit motor entry in config XML");
        }
        if (motor.bus == MotorBus::Can && !motor.type.empty() && !startsWith(motor.type, "RMD")) {
            motorElement = motorElement->NextSiblingElement("Motor");
            continue;
        }

        if (motor.bus == MotorBus::Can) {
            motor.motorId = firstIntAttribute(motorElement, "id", "motor_id", "slave_id", 0);
            motor.slaveId = firstIntAttribute(motorElement, "slave_id", "id", "motor_id", motor.motorId);
            if (motor.motorId <= 0) {
                throw std::runtime_error("explicit CAN motor entry is missing id");
            }
        } else {
            motor.ethercatSlave = firstIntAttribute(motorElement, "slave", "ethercat_slave", "slave_id", -1);
            motor.slaveId = motor.ethercatSlave;
            motor.motorId = firstIntAttribute(motorElement, "id", "motor_id", nullptr, motor.alias);
            if (motor.ethercatSlave < 0) {
                throw std::runtime_error("explicit EtherCAT motor entry is missing slave");
            }
        }

        motor.parameters = params->second;
        upsertMotor(config, motor);
        motorElement = motorElement->NextSiblingElement("Motor");
    }
}

void readEcatConfig(tinyxml2::XMLElement* root,
                    std::map<int, MotorParameters> const& motorParameters,
                    Config& config) {
    tinyxml2::XMLElement* ecat = root->FirstChildElement("ECAT");
    if (ecat == nullptr) {
        return;
    }

    tinyxml2::XMLElement* masters = ecat->FirstChildElement("Masters");
    if (masters != nullptr) {
        tinyxml2::XMLElement* master = masters->FirstChildElement("Master");
        while (master != nullptr) {
            config.periodNs = master->Int64Attribute("period", config.periodNs);
            config.ethercatDc = master->BoolAttribute("dc", config.ethercatDc);
            master = master->NextSiblingElement("Master");
        }
    }

    tinyxml2::XMLElement* domains = ecat->FirstChildElement("Domains");
    if (domains != nullptr) {
        tinyxml2::XMLElement* domain = domains->FirstChildElement("Domain");
        while (domain != nullptr) {
            int const order = domain->IntAttribute("order", -1);
            if (order >= 0) {
                std::size_t const index = static_cast<std::size_t>(order);
                if (config.ethercatDomainDivisions.size() <= index) {
                    config.ethercatDomainDivisions.resize(index + 1, 1);
                }
                config.ethercatDomainDivisions[index] = std::max(1, domain->IntAttribute("division", 1));
            }
            domain = domain->NextSiblingElement("Domain");
        }
    }

    tinyxml2::XMLElement* slaves = ecat->FirstChildElement("Slaves");
    if (slaves == nullptr) {
        return;
    }

    tinyxml2::XMLElement* slave = slaves->FirstChildElement("Slave");
    while (slave != nullptr) {
        if (slave->IntText(0) != 1) {
            slave = slave->NextSiblingElement("Slave");
            continue;
        }

        char const* typeAttr = slave->Attribute("type");
        std::string type = typeAttr == nullptr ? "" : typeAttr;
        if (!isEthercatMtDeviceType(type)) {
            slave = slave->NextSiblingElement("Slave");
            continue;
        }

        MotorConfig motor;
        motor.alias = slave->IntAttribute("alias", 0);
        motor.master = slave->IntAttribute("master", 0);
        motor.domain = slave->IntAttribute("domain", 0);
        motor.ethercatSlave = firstIntAttribute(slave, "slave", "ethercat_slave", "slave_id", -1);
        motor.slaveId = motor.ethercatSlave;
        motor.motorId = firstIntAttribute(slave, "id", "motor_id", nullptr, motor.alias);
        motor.bus = MotorBus::Ethercat;
        motor.type = type;

        auto params = motorParameters.find(motor.alias);
        if (motor.alias <= 0 || params == motorParameters.end()) {
            throw std::runtime_error("invalid MT_Device ECAT slave entry in config XML");
        }
        motor.parameters = params->second;
        upsertMotor(config, motor);
        slave = slave->NextSiblingElement("Slave");
    }
}

void readImuConfig(tinyxml2::XMLElement* root, Config& config) {
    tinyxml2::XMLElement* imu = root->FirstChildElement("IMU");
    if (imu == nullptr) {
        return;
    }

    config.imu.enabled = true;
    char const* device = imu->Attribute("device");
    char const* type = imu->Attribute("type");
    config.imu.device = device == nullptr ? "" : device;
    config.imu.baudrate = imu->IntAttribute("baudrate", 0);
    config.imu.type = type == nullptr ? "" : type;
    config.imu.normalizedType = normalizeUpper(config.imu.type);
}

} // namespace

Config loadConfig(std::string const& path) {
    tinyxml2::XMLDocument doc;
    auto const rc = doc.LoadFile(path.c_str());
    if (rc != tinyxml2::XML_SUCCESS) {
        throw std::runtime_error("loading config XML failed: " + path + ": " + doc.ErrorStr());
    }
    tinyxml2::XMLElement* root = doc.FirstChildElement("Config");
    if (root == nullptr) {
        throw std::runtime_error("invalid config XML: missing <Config>");
    }

    Config config;
    auto motorParameters = readMotorParameterMap(root, config.totalMotorCount);

    tinyxml2::XMLElement* can = root->FirstChildElement("CAN");
    if (can != nullptr) {
        tinyxml2::XMLElement* masters = can->FirstChildElement("Masters");
        if (masters != nullptr) {
            config.periodNs = masters->Int64Attribute("period", config.periodNs);
            tinyxml2::XMLElement* master = masters->FirstChildElement("Master");
            while (master != nullptr) {
                MasterConfig parsed;
                parsed.order = master->IntAttribute("order", 0);
                char const* device = master->Attribute("device");
                parsed.device = device == nullptr ? "" : device;
                parsed.baudrate = master->IntAttribute("baudrate", parsed.baudrate);
                parsed.canfd = master->BoolAttribute("canfd", parsed.canfd);
                parsed.dbaudrate = master->IntAttribute("dbaudrate", parsed.dbaudrate);
                parsed.division = master->IntAttribute("division", parsed.division);
                if (parsed.division <= 0) {
                    parsed.division = 1;
                }
                config.masters.push_back(parsed);
                master = master->NextSiblingElement("Master");
            }
        }

        tinyxml2::XMLElement* slaves = can->FirstChildElement("Slaves");
        if (slaves != nullptr) {
            tinyxml2::XMLElement* slave = slaves->FirstChildElement("Slave");
            while (slave != nullptr) {
                if (slave->IntText(0) != 1) {
                    slave = slave->NextSiblingElement("Slave");
                    continue;
                }
                char const* typeAttr = slave->Attribute("type");
                std::string type = typeAttr == nullptr ? "" : typeAttr;
                if (!startsWith(type, "RMD")) {
                    slave = slave->NextSiblingElement("Slave");
                    continue;
                }
                MotorConfig motor;
                motor.alias = slave->IntAttribute("alias", 0);
                motor.master = slave->IntAttribute("master", 0);
                motor.slaveId = slave->IntAttribute("slave_id", 0);
                motor.motorId = motor.slaveId;
                motor.ethercatSlave = 0;
                motor.bus = MotorBus::Can;
                motor.type = type;
                auto params = motorParameters.find(motor.alias);
                if (motor.alias <= 0 || motor.motorId <= 0 || params == motorParameters.end()) {
                    throw std::runtime_error("invalid RMD CAN slave entry in config XML");
                }
                motor.parameters = params->second;
                upsertMotor(config, motor);
                slave = slave->NextSiblingElement("Slave");
            }
        }
    }

    readEcatConfig(root, motorParameters, config);
    readExplicitMotorPlacements(root, motorParameters, config);
    readImuConfig(root, config);

    return config;
}

} // namespace RmdCanSdk
