#include "rmd_can_sdk/heima_driver_sdk.h"
#include "rmd_can_sdk/heima_ecat_types.h"
#include "rmd_can_sdk/rmd_bench_workflow.h"
#include "rmd_can_sdk/rmd_can_backend.h"
#include "rmd_can_sdk/rmd_can_config.h"
#include "rmd_can_sdk/rmd_ethercat_bindings.h"
#include "rmd_can_sdk/rmd_ethercat_mt_device.h"
#include "rmd_can_sdk/rmd_ethercat_pdo.h"
#include "rmd_can_sdk/rmd_ethercat_snapshot.h"
#include "rmd_can_sdk/rmd_ethercat_target_frame.h"
#include "rmd_can_sdk/rmd_motor_registry.h"
#include "rmd_can_sdk/rmd_motor_backend.h"
#include "rmd_can_sdk/rmd_motion_plan.h"
#include "rmd_can_sdk/rmd_protocol.h"
#include "rmd_can_sdk/rmd_realtime_core.h"
#include "rmd_can_sdk/rmd_safety.h"
#include "rmd_can_sdk/rmd_types.h"
#include "rmd_can_sdk/rs232_imu_backend.h"
#include "rmd_can_sdk/yesense_imu_decoder.h"

#include <atomic>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <cstddef>
#include <string>
#include <thread>
#include <vector>

namespace {

void require(bool condition, std::string const& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        std::exit(1);
    }
}

std::string writeTempConfig() {
    std::string const path = "/tmp/rmd_can_sdk_test_config.xml";
    std::ofstream out(path);
    out << R"(<?xml version="1.0" encoding="UTF-8"?>
<Config>
  <CAN>
    <Devices>
      <Device type="RMD-X12-P20-320">
        <MinP>-12.5</MinP><MaxP>12.5</MaxP>
        <MinV>-45</MinV><MaxV>45</MaxV>
        <MinKp>0</MinKp><MaxKp>500</MaxKp>
        <MinKd>0</MinKd><MaxKd>5</MaxKd>
        <MinT>-24</MinT><MaxT>24</MaxT>
      </Device>
    </Devices>
    <Categories>
      <Category name="driver">
        <Type master_ids="14" masters="0">RMD-X12-P20-320</Type>
        <Type master_ids="15" masters="0">RMD-X12-P20-320</Type>
      </Category>
    </Categories>
    <Masters period="1000000">
      <Master order="0" device="can0" canhal="false" baudrate="1000000" canfd="false" dbaudrate="5000000" division="1"/>
    </Masters>
    <Slaves>
      <Slave master="0" master_ids="14" slave_id="14" alias="1" type="RMD-X12-P20-320">1</Slave>
      <Slave master="0" master_ids="15" slave_id="15" alias="2" type="RMD-X12-P20-320">1</Slave>
      <Slave master="0" master_ids="16" slave_id="16" alias="3" type="Damiao DM-J4310">1</Slave>
    </Slaves>
  </CAN>
  <Motors>
    <Motor limb="0" motor="0" alias="1">
      <Polarity>1</Polarity><CountBias>0</CountBias><EncoderResolution>131072</EncoderResolution>
      <GearRatioTor>20</GearRatioTor><GearRatioPosVel>20</GearRatioPosVel>
      <RatedCurrent>42.42</RatedCurrent><TorqueConstant>2.81</TorqueConstant>
      <RatedTorque>85</RatedTorque><MaximumTorque>320</MaximumTorque>
      <MinimumPosition>-1</MinimumPosition><MaximumPosition>1</MaximumPosition>
    </Motor>
    <Motor limb="0" motor="1" alias="2">
      <Polarity>-1</Polarity><CountBias>0.25</CountBias><EncoderResolution>131072</EncoderResolution>
      <GearRatioTor>20</GearRatioTor><GearRatioPosVel>20</GearRatioPosVel>
      <RatedCurrent>42.42</RatedCurrent><TorqueConstant>2.81</TorqueConstant>
      <RatedTorque>85</RatedTorque><MaximumTorque>320</MaximumTorque>
      <MinimumPosition>-2</MinimumPosition><MaximumPosition>2</MaximumPosition>
    </Motor>
  </Motors>
</Config>
)";
    return path;
}

std::string writeTempMixedBusConfig() {
    std::string const path = "/tmp/rmd_can_sdk_test_mixed_bus_config.xml";
    std::ofstream out(path);
    out << R"(<?xml version="1.0" encoding="UTF-8"?>
<Config>
  <CAN>
    <Masters period="1000000">
      <Master order="0" device="can0" baudrate="1000000"/>
    </Masters>
    <Slaves>
      <Slave master="0" slave_id="14" alias="1" type="RMD-X12-P20-320">1</Slave>
    </Slaves>
  </CAN>
  <Motors>
    <Motor alias="1" bus="CAN" master="0" id="14" type="RMD-X12-P20-320">
      <Polarity>1</Polarity><CountBias>0</CountBias><EncoderResolution>131072</EncoderResolution>
      <GearRatioTor>20</GearRatioTor><GearRatioPosVel>20</GearRatioPosVel>
      <RatedCurrent>42.42</RatedCurrent><TorqueConstant>2.81</TorqueConstant>
      <RatedTorque>85</RatedTorque><MaximumTorque>320</MaximumTorque>
      <MinimumPosition>-1</MinimumPosition><MaximumPosition>1</MaximumPosition>
    </Motor>
    <Motor alias="2" bus="ECAT" master="0" slave="3" type="RMD-X12-P20-320">
      <Polarity>1</Polarity><CountBias>0</CountBias><EncoderResolution>131072</EncoderResolution>
      <GearRatioTor>20</GearRatioTor><GearRatioPosVel>20</GearRatioPosVel>
      <RatedCurrent>42.42</RatedCurrent><TorqueConstant>2.81</TorqueConstant>
      <RatedTorque>85</RatedTorque><MaximumTorque>320</MaximumTorque>
      <MinimumPosition>-2</MinimumPosition><MaximumPosition>2</MaximumPosition>
    </Motor>
  </Motors>
</Config>
)";
    return path;
}

std::string writeTempMtDeviceEthercatConfig() {
    std::string const path = "/tmp/rmd_can_sdk_test_mt_device_ecat_config.xml";
    std::ofstream out(path);
    out << R"(<?xml version="1.0" encoding="UTF-8"?>
<Config>
  <Motors>
    <Motor alias="1" bus="ECAT" master="0" slave="3" domain="1" type="MT-Device">
      <Polarity>1</Polarity><CountBias>0</CountBias><EncoderResolution>131072</EncoderResolution>
      <GearRatioTor>20</GearRatioTor><GearRatioPosVel>20</GearRatioPosVel>
      <RatedCurrent>42.42</RatedCurrent><TorqueConstant>2.81</TorqueConstant>
      <RatedTorque>85</RatedTorque><MaximumTorque>320</MaximumTorque>
      <MinimumPosition>-2</MinimumPosition><MaximumPosition>2</MaximumPosition>
    </Motor>
  </Motors>
</Config>
)";
    return path;
}

std::string writeTempOriginalStyleEthercatConfig() {
    std::string const path = "/tmp/rmd_can_sdk_test_original_style_ecat_config.xml";
    std::ofstream out(path);
    out << R"(<?xml version="1.0" encoding="UTF-8"?>
<Config>
  <ECAT>
    <Masters>
      <Master order="0" dc="true" period="1000000"/>
    </Masters>
    <Domains>
      <Domain master="0" order="0" division="1"/>
      <Domain master="0" order="1" division="1"/>
      <Domain master="0" order="2" division="1"/>
    </Domains>
    <Slaves>
      <Slave master="0" domain="2" alias="1" type="MT_Device">1</Slave>
    </Slaves>
  </ECAT>
  <Motors>
    <Motor alias="1">
      <Polarity>1</Polarity><CountBias>0</CountBias><EncoderResolution>131072</EncoderResolution>
      <GearRatioTor>20</GearRatioTor><GearRatioPosVel>20</GearRatioPosVel>
      <RatedCurrent>42.42</RatedCurrent><TorqueConstant>2.81</TorqueConstant>
      <RatedTorque>85</RatedTorque><MaximumTorque>320</MaximumTorque>
      <MinimumPosition>-2</MinimumPosition><MaximumPosition>2</MaximumPosition>
    </Motor>
  </Motors>
</Config>
)";
    return path;
}

std::string writeTempOrinSplitterEthercatConfig() {
    std::string const path = "/tmp/rmd_can_sdk_test_orin_splitter_ecat_config.xml";
    std::ofstream out(path);
    out << R"(<?xml version="1.0" encoding="UTF-8"?>
<Config>
  <ECAT>
    <Masters>
      <Master order="0" dc="true" period="1000000"/>
    </Masters>
    <Domains>
      <Domain master="0" order="0" division="1"/>
      <Domain master="0" order="1" division="1"/>
    </Domains>
    <Slaves>
      <Slave master="0" domain="0" slave="1" alias="1" type="MT_Device">1</Slave>
      <Slave master="0" domain="1" slave="14" alias="2" type="MT_Device">1</Slave>
    </Slaves>
  </ECAT>
  <Motors>
    <Motor alias="1">
      <Polarity>1</Polarity><CountBias>0</CountBias><EncoderResolution>131072</EncoderResolution>
      <GearRatioTor>20</GearRatioTor><GearRatioPosVel>20</GearRatioPosVel>
      <RatedCurrent>42.42</RatedCurrent><TorqueConstant>2.81</TorqueConstant>
      <RatedTorque>85</RatedTorque><MaximumTorque>320</MaximumTorque>
      <MinimumPosition>-2</MinimumPosition><MaximumPosition>2</MaximumPosition>
    </Motor>
    <Motor alias="2">
      <Polarity>1</Polarity><CountBias>0</CountBias><EncoderResolution>131072</EncoderResolution>
      <GearRatioTor>20</GearRatioTor><GearRatioPosVel>20</GearRatioPosVel>
      <RatedCurrent>42.42</RatedCurrent><TorqueConstant>2.81</TorqueConstant>
      <RatedTorque>85</RatedTorque><MaximumTorque>320</MaximumTorque>
      <MinimumPosition>-2</MinimumPosition><MaximumPosition>2</MaximumPosition>
    </Motor>
  </Motors>
</Config>
)";
    return path;
}

std::string writeTempImuConfig() {
    std::string const path = "/tmp/rmd_can_sdk_test_imu_config.xml";
    std::ofstream out(path);
    out << R"(<?xml version="1.0" encoding="UTF-8"?>
<Config>
  <CAN>
    <Masters period="1000000">
      <Master order="0" device="can0" baudrate="1000000"/>
    </Masters>
    <Slaves>
      <Slave master="0" slave_id="14" alias="1" type="RMD-X12-P20-320">1</Slave>
    </Slaves>
  </CAN>
  <Motors>
    <Motor alias="1">
      <Polarity>1</Polarity><CountBias>0</CountBias><EncoderResolution>131072</EncoderResolution>
      <GearRatioTor>20</GearRatioTor><GearRatioPosVel>20</GearRatioPosVel>
      <RatedCurrent>42.42</RatedCurrent><TorqueConstant>2.81</TorqueConstant>
      <RatedTorque>85</RatedTorque><MaximumTorque>320</MaximumTorque>
      <MinimumPosition>-1</MinimumPosition><MaximumPosition>1</MaximumPosition>
    </Motor>
  </Motors>
  <IMU device="/dev/ttyACM0" baudrate="921600" type="YeSense"/>
</Config>
)";
    return path;
}

std::string writeTempImuOnlyConfig() {
    std::string const path = "/tmp/rmd_can_sdk_test_imu_only_config.xml";
    std::ofstream out(path);
    out << R"(<?xml version="1.0" encoding="UTF-8"?>
<Config>
  <Motors>
  </Motors>
  <IMU device="/tmp/rmd_missing_yesense_imu" baudrate="921600" type="YeSense"/>
</Config>
)";
    return path;
}

std::string writeTempEmptyConfig() {
    std::string const path = "/tmp/rmd_can_sdk_test_empty_config.xml";
    std::ofstream out(path);
    out << R"(<?xml version="1.0" encoding="UTF-8"?>
<Config>
  <Motors>
  </Motors>
</Config>
)";
    return path;
}

void testMitCodec() {
    auto zero = RmdCanSdk::packMit(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    std::vector<unsigned char> expected{0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08, 0x00};
    require(std::vector<unsigned char>(zero.begin(), zero.end()) == expected, "MIT zero command bytes must match Python codec");

    auto reply = RmdCanSdk::parseMitReply({14, 0x7f, 0xff, 0x7f, 0xf7, 0xff, 0x00, 0x00}, 14);
    require(reply.valid, "MIT reply with matching motor id is valid");
    require(reply.motorId == 14, "MIT reply motor id parsed");
    require(std::fabs(reply.positionRad) < 0.001f, "MIT reply position near zero");
}

void testMitV44ScalingUsesDynamicTorqueAndKd50() {
    require(std::fabs(RmdCanSdk::RmdPMin + 12.566f) < 0.0001f, "V4.4 MIT p_min is -12.566rad");
    require(std::fabs(RmdCanSdk::RmdPMax - 12.566f) < 0.0001f, "V4.4 MIT p_max is 12.566rad");
    require(std::fabs(RmdCanSdk::RmdKdMax - 50.0f) < 0.0001f, "V4.4 MIT kd max is 50");

    auto command = RmdCanSdk::packMit(0.0f, 0.0f, 0.0f, 10.0f, 41.68498f, 100.0f);
    std::vector<unsigned char> expected{0x80, 0x00, 0x80, 0x00, 0x00, 0x33, 0x3B, 0x55};
    require(std::vector<unsigned char>(command.begin(), command.end()) == expected,
            "V4.4 MIT command scales kd over 0..50 and t_ff over +/-motor maximum torque");

    auto feedback = RmdCanSdk::parseMitReply({14, 0x80, 0x00, 0x80, 0x0B, 0x55, 0x00, 0x00}, 14, 100.0f);
    require(feedback.valid, "V4.4 MIT feedback with dynamic torque range is valid");
    require(std::fabs(feedback.feedbackTorqueNm - 41.68498f) < 0.05f,
            "V4.4 MIT feedback torque decodes over +/-motor maximum torque");
}

void testCanProtocolParsesFixedFramesWithoutVectorAllocation() {
    std::array<unsigned char, 8> mit{14, 0x80, 0x00, 0x80, 0x0B, 0x55, 0x00, 0x00};
    auto mitFeedback = RmdCanSdk::parseMitReply(mit, 8, 14, 100.0f);
    require(mitFeedback.valid, "fixed CAN frame MIT overload parses valid feedback");
    require(std::fabs(mitFeedback.feedbackTorqueNm - 41.68498f) < 0.05f,
            "fixed CAN frame MIT overload keeps dynamic torque scaling");

    std::array<unsigned char, 8> status2{0x9C, 0x20, 0x64, 0x00, 0x0C, 0x00, 0x4E, 0x00};
    auto status2Feedback = RmdCanSdk::parseStatus2Reply(status2, 8, 2.81f);
    require(status2Feedback.valid, "fixed CAN frame status2 overload parses valid feedback");
    require(std::fabs(status2Feedback.estimatedTorqueNm - 2.81f) < 0.0001f,
            "fixed CAN frame status2 overload preserves torque estimate");

    auto shortFeedback = RmdCanSdk::parseStatus2Reply(status2, 7, 2.81f);
    require(!shortFeedback.valid, "fixed CAN frame status2 overload rejects short frames");
}

void testStatus2TorqueEstimate() {
    auto feedback = RmdCanSdk::parseStatus2Reply({0x9C, 0x20, 0x64, 0x00, 0x0C, 0x00, 0x4E, 0x00}, 2.81f);
    require(feedback.valid, "0x9C status2 reply is valid");
    require(std::fabs(feedback.iqA - 1.0f) < 0.0001f, "0x9C iq decodes as 0.01A/LSB");
    require(std::fabs(feedback.estimatedTorqueNm - 2.81f) < 0.0001f, "actual torque estimate uses 0x9C iq * torque constant");
    require(feedback.speedDps == 12, "0x9C speed parses as dps");
    require(feedback.angleDeg == 78, "0x9C angle parses as output degrees");
}

void testMultiturnAngleParsesHundredthDegrees() {
    auto feedback = RmdCanSdk::parseMultiturnAngleReply({0x92, 0x00, 0x00, 0x00, 0x3C, 0xFF, 0xFF, 0xFF});
    require(feedback.valid, "0x92 multiturn angle reply is valid");
    require(feedback.rawAngleCentideg == -196, "0x92 multiturn angle parses signed little-endian centidegrees");
    require(std::fabs(feedback.angleDeg + 1.96f) < 0.0001f, "0x92 angle scales as 0.01deg/LSB");
    require(std::fabs(feedback.angleRad - (-1.96f * RmdCanSdk::Pi / 180.0f)) < 0.0001f,
            "0x92 angle also reports radians");
}

void testStatus1BrakeCommandAndErrorFlags() {
    auto feedback = RmdCanSdk::parseStatus1Reply({0x9A, 0x18, 0x21, 0x01, 0xB4, 0x02, 0x16, 0x00});
    require(feedback.valid, "0x9A status1 reply is valid");
    require(feedback.temperatureC == 24, "0x9A temperature parses as signed Celsius");
    require(feedback.mosTemperatureC == 33, "0x9A MOS temperature parses as signed Celsius");
    require(feedback.brakeReleaseCommandState == 1, "0x9A DATA[3] exposes RlyCtrlRslt command state");
    require(feedback.brakeReleaseCommandActive, "0x9A DATA[3] command state true means release command active");
    require(std::fabs(feedback.voltageV - 69.2f) < 0.0001f, "0x9A voltage parses as 0.1V/LSB");
    require(feedback.errorState == 0x0016, "0x9A error state parses little-endian");
    require((feedback.errorState & RmdCanSdk::Status1ErrorStall) != 0, "0x0002 means stall per PDF table");
    require((feedback.errorState & RmdCanSdk::Status1ErrorLowVoltage) != 0, "0x0004 means low voltage per PDF table");
    require((feedback.errorState & RmdCanSdk::Status1ErrorPhaseCurrentOverCurrent) != 0,
            "0x0010 means phase current over-current per PDF table");
}

void testObservationUsesMitPositionAndStatus2AuxiliaryData() {
    RmdCanSdk::MotorActual actual;
    RmdCanSdk::MitFeedback mit;
    mit.valid = true;
    mit.positionRad = 1.25f;
    mit.velocityRadS = -0.5f;
    mit.feedbackTorqueNm = 3.0f;

    RmdCanSdk::applyMitFeedback(actual, mit);
    require(std::fabs(actual.pos - 1.25f) < 0.0001f, "MIT feedback owns observation position");
    require(std::fabs(actual.vel + 0.5f) < 0.0001f, "MIT feedback owns observation velocity");
    require(actual.statusWord == 0x0237, "MIT feedback marks motor healthy");
    require(actual.errorCode == 0, "MIT feedback clears observation error");

    RmdCanSdk::Status2Feedback status2;
    status2.valid = true;
    status2.angleRad = -2.0f;
    status2.speedDps = 123;
    status2.estimatedTorqueNm = 4.5f;
    status2.temperatureC = 42;

    RmdCanSdk::applyStatus2Feedback(actual, status2);
    require(std::fabs(actual.pos - 1.25f) < 0.0001f, "0x9C must not overwrite MIT observation position");
    require(std::fabs(actual.vel + 0.5f) < 0.0001f, "0x9C must not overwrite MIT observation velocity");
    require(std::fabs(actual.tor - 4.5f) < 0.0001f, "0x9C updates auxiliary torque estimate");
    require(actual.temp == 42, "0x9C updates auxiliary temperature");
    require(actual.statusWord == 0x0237, "0x9C auxiliary data must not downgrade MIT health");
    require(actual.errorCode == 0, "0x9C auxiliary data must not set observation error");
}

void testConfigParser() {
    RmdCanSdk::Config config = RmdCanSdk::loadConfig(writeTempConfig());
    require(config.masters.size() == 1, "one CAN master parsed");
    require(config.masters[0].order == 0, "master order parsed");
    require(config.motors.size() == 2, "only RMD enabled slaves parsed");
    require(config.motors[0].alias == 1 && config.motors[0].motorId == 14, "first RMD alias/id parsed");
    require(config.motors[1].alias == 2 && config.motors[1].motorId == 15, "second RMD alias/id parsed");
    require(config.motors[1].parameters.polarity == -1.0f, "motor parameters parsed by alias");
    require(!config.imu.enabled, "missing IMU element leaves IMU disabled");
}

void testConfigParserReadsYesenseImu() {
    RmdCanSdk::Config config = RmdCanSdk::loadConfig(writeTempImuConfig());

    require(config.imu.enabled, "IMU element enables IMU config");
    require(config.imu.device == "/dev/ttyACM0", "IMU device parses");
    require(config.imu.baudrate == 921600, "IMU baudrate parses");
    require(config.imu.type == "YeSense", "IMU type preserves configured spelling");
    require(config.imu.normalizedType == "YESENSE", "IMU type normalizes for backend selection");
}

void testMotorRegistryNormalizesCanMotors() {
    RmdCanSdk::Config config = RmdCanSdk::loadConfig(writeTempConfig());
    RmdCanSdk::MotorRegistry registry = RmdCanSdk::MotorRegistry::fromConfig(config);

    require(registry.totalMotorCount() == 2, "registry keeps total configured motor count");
    require(registry.backendCount() == 1, "registry creates one CAN backend group");
    require(registry.motorByAlias(1).globalIndex == 0, "alias 1 maps to global index 0");
    require(registry.motorByAlias(2).globalIndex == 1, "alias 2 maps to global index 1");
    require(registry.motorByAlias(1).bus == RmdCanSdk::MotorBus::Can, "alias 1 is CAN");
    require(registry.motorByAlias(1).backendLocalIndex == 0, "first CAN motor local index is 0");
    require(registry.motorByAlias(2).backendLocalIndex == 1, "second CAN motor local index is 1");
}

void testMixedBusConfigCreatesIndependentBackendGroups() {
    RmdCanSdk::Config config = RmdCanSdk::loadConfig(writeTempMixedBusConfig());
    RmdCanSdk::MotorRegistry registry = RmdCanSdk::MotorRegistry::fromConfig(config);

    require(registry.totalMotorCount() == 2, "mixed registry keeps total configured motor count");
    require(registry.backendCount() == 2, "mixed registry creates one CAN and one EtherCAT backend group");
    require(registry.motorByAlias(1).bus == RmdCanSdk::MotorBus::Can, "alias 1 remains CAN");
    require(registry.motorByAlias(2).bus == RmdCanSdk::MotorBus::Ethercat, "alias 2 is EtherCAT");
    require(registry.motorByAlias(2).ethercatSlave == 3, "EtherCAT slave position parses");
    require(registry.motorByAlias(2).backendLocalIndex == 0, "first EtherCAT motor local index is 0");
}

void testEthercatBindingTableKeepsHeimaAliasSlaveDomainAndOffsets() {
    RmdCanSdk::Config config = RmdCanSdk::loadConfig(writeTempMtDeviceEthercatConfig());
    require(config.motors.size() == 1, "MT_Device EtherCAT motor is parsed");
    require(config.motors[0].bus == RmdCanSdk::MotorBus::Ethercat, "MT_Device motor remains EtherCAT");
    require(config.motors[0].type == "MT-Device", "MT_Device type is preserved for heima lookup");
    require(config.motors[0].ethercatSlave == 3, "MT_Device slave position parses");
    require(config.motors[0].domain == 1, "MT_Device domain parses");

    RmdCanSdk::MotorRegistry registry = RmdCanSdk::MotorRegistry::fromConfig(config);
    RmdCanSdk::RuntimeMotor const& motor = registry.motorByAlias(1);
    require(motor.domain == 1, "registry keeps EtherCAT domain");

    std::vector<RmdCanSdk::EthercatPdoBinding> bindings =
        RmdCanSdk::buildEthercatPdoBindings(registry, motor.backendIndex);
    require(bindings.size() == 1, "one EtherCAT PDO binding is built");
    require(bindings[0].globalIndex == 0, "binding keeps global alias index");
    require(bindings[0].backendLocalIndex == 0, "binding keeps backend-local index");
    require(bindings[0].alias == 1, "binding keeps alias");
    require(bindings[0].master == 0, "binding keeps master order");
    require(bindings[0].slave == 3, "binding keeps EtherCAT slave position");
    require(bindings[0].domain == 1, "binding keeps EtherCAT domain");
    require(bindings[0].type == "MT-Device", "binding keeps heima device type");
    require(bindings[0].rxOffset == -1 && bindings[0].txOffset == -1, "binding starts without registered offsets");

    RmdCanSdk::assignEthercatPdoOffsets(bindings, 0, 42, 84);
    require(bindings[0].rxOffset == 42, "binding records registered RxPDO offset");
    require(bindings[0].txOffset == 84, "binding records registered TxPDO offset");
}

void testOriginalStyleEthercatConfigCreatesMtDeviceMotor() {
    RmdCanSdk::Config config = RmdCanSdk::loadConfig(writeTempOriginalStyleEthercatConfig());
    require(config.periodNs == 1000000L, "original ECAT master period parses");
    require(config.ethercatDc, "original ECAT master dc flag parses");
    require(config.ethercatDomainDivisions.size() == 3, "original ECAT domain divisions parse");
    require(config.ethercatDomainDivisions[2] == 1, "original ECAT domain division value parses");
    require(config.motors.size() == 1, "original ECAT slave entry creates one motor");
    require(config.motors[0].bus == RmdCanSdk::MotorBus::Ethercat, "original ECAT slave creates EtherCAT motor");
    require(config.motors[0].type == "MT_Device", "original ECAT slave keeps MT_Device type");
    require(config.motors[0].master == 0, "original ECAT slave master parses");
    require(config.motors[0].domain == 2, "original ECAT slave domain parses");
    require(config.motors[0].ethercatSlave == -1, "original ECAT slave position remains unresolved before bus scan");

    RmdCanSdk::MotorRegistry registry = RmdCanSdk::MotorRegistry::fromConfig(config);
    require(registry.backendCount() == 1, "original ECAT config creates one EtherCAT backend group");
    RmdCanSdk::RuntimeMotor const& motor = registry.motorByAlias(1);
    require(motor.type == "MT_Device", "registry keeps original MT_Device spelling");
    require(motor.domain == 2, "registry keeps original ECAT domain");

    std::vector<RmdCanSdk::EthercatPdoBinding> bindings =
        RmdCanSdk::buildEthercatPdoBindings(registry, motor.backendIndex);
    require(bindings.size() == 1, "original ECAT config creates one binding");
    require(RmdCanSdk::resolveEthercatSlavePosition(bindings[0]) == 0,
            "unresolved original ECAT slave position falls back to alias order");
}

void testOrinSplitterEthercatConfigKeepsExplicitSlavePositions() {
    RmdCanSdk::Config config = RmdCanSdk::loadConfig(writeTempOrinSplitterEthercatConfig());
    require(config.motors.size() == 2, "Orin splitter ECAT config creates only MT_Device motors");
    require(config.motors[0].ethercatSlave == 1, "Orin splitter config skips branch device at slave position 0");
    require(config.motors[1].ethercatSlave == 14, "Orin splitter config skips branch device at slave position 13");

    RmdCanSdk::MotorRegistry registry = RmdCanSdk::MotorRegistry::fromConfig(config);
    RmdCanSdk::RuntimeMotor const& first = registry.motorByAlias(1);
    RmdCanSdk::RuntimeMotor const& waist = registry.motorByAlias(2);
    std::vector<RmdCanSdk::EthercatPdoBinding> bindings =
        RmdCanSdk::buildEthercatPdoBindings(registry, first.backendIndex);
    require(bindings.size() == 2, "Orin splitter ECAT config keeps both RMD EtherCAT bindings");
    require(RmdCanSdk::resolveEthercatSlavePosition(bindings[first.backendLocalIndex]) == 1,
            "first RMD binding resolves to slave position 1");
    require(RmdCanSdk::resolveEthercatSlavePosition(bindings[waist.backendLocalIndex]) == 14,
            "waist RMD binding resolves to slave position 14");
}

void testEthercatSnapshotUsesRegisteredPdoOffsets() {
    std::array<std::uint8_t, 128> domain{};

    RmdCanSdk::MotorParameters params;
    params.encoderResolution = 1000.0f;
    params.gearRatioPosVel = 2.0f;
    params.gearRatioTor = 10.0f;
    params.torqueConstant = 0.5f;
    params.maximumTorque = 10.0f;
    params.minimumPosition = -10.0f;
    params.maximumPosition = 10.0f;

    RmdCanSdk::EthercatPdoBinding binding;
    binding.globalIndex = 0;
    binding.backendLocalIndex = 0;
    binding.alias = 1;
    binding.rxOffset = 73;
    binding.txOffset = 17;
    binding.rxProfile = RmdCanSdk::EthercatMtDeviceRxProfile::Pvt;
    binding.txProfile = RmdCanSdk::EthercatMtDeviceTxProfile::Extended;
    binding.rxSize = RmdCanSdk::ethercatMtDeviceRxPdoSize(binding.rxProfile);
    binding.txSize = RmdCanSdk::ethercatMtDeviceTxPdoSize(binding.txProfile);
    binding.parameters = params;
    std::vector<RmdCanSdk::EthercatPdoBinding> bindings{binding};

    RmdCanSdk::EthercatTxPdoBytes tx{};
    tx.StatusWord = 0x0237;
    tx.ActualPosition = 1000;
    tx.ActualVelocity = 500;
    tx.ActualTorque = 2;
    tx.MotorTemperature = 35;
    tx.DriveTemperature = 40;
    tx.Voltage = 680;
    std::memcpy(domain.data() + binding.txOffset, &tx, sizeof(tx));

    RmdCanSdk::MotorActualFrame actuals;
    actuals.motorCount = 1;
    RmdCanSdk::readEthercatDomainSnapshot(domain.data(), domain.size(), bindings, actuals);
    require(actuals.valid.test(0), "snapshot reader marks registered motor valid");
    require(!actuals.stale.test(0), "snapshot reader clears stale for valid PDO");
    require(actuals.actuals[0].statusWord == 0x0237, "snapshot reader parses status from registered TxPDO offset");
    require(actuals.actuals[0].temp == 35, "snapshot reader parses temperature from registered TxPDO offset");

    RmdCanSdk::MotorTargetFrame targets;
    targets.motorCount = 1;
    targets.targets[0].pos = RmdCanSdk::Pi;
    targets.targets[0].vel = RmdCanSdk::Pi / 2.0f;
    targets.targets[0].tor = 5.0f;
    targets.targets[0].kp = 12.3f;
    targets.targets[0].kd = 4.6f;
    targets.targets[0].mode = 5;
    targets.targets[0].enabled = 1;
    targets.valid.set(0);
    RmdCanSdk::writeEthercatDomainSnapshot(domain.data(), domain.size(), bindings, targets, actuals);

    RmdCanSdk::EthercatRxPdoBytes rx;
    std::memcpy(&rx, domain.data() + binding.rxOffset, sizeof(rx));
    require(rx.ControlWord == 0x000f, "snapshot writer writes enable command at registered RxPDO offset");
    require(rx.TargetPosition == 500, "snapshot writer packs heima position at registered RxPDO offset");
    require(rx.TargetVelocity == 250, "snapshot writer packs heima velocity at registered RxPDO offset");
    require(rx.TargetTorque == 10000, "snapshot writer packs heima torque at registered RxPDO offset");
    require(rx.PvtKd == 4600, "snapshot writer packs heima gains at registered RxPDO offset");

    tx.StatusWord = 0x0033;
    tx.ActualPosition = 1234;
    std::memcpy(domain.data() + binding.txOffset, &tx, sizeof(tx));
    RmdCanSdk::readEthercatDomainSnapshot(domain.data(), domain.size(), bindings, actuals);
    RmdCanSdk::writeEthercatDomainSnapshot(domain.data(), domain.size(), bindings, targets, actuals);
    std::memcpy(&rx, domain.data() + binding.rxOffset, sizeof(rx));
    require(rx.ControlWord == 0x000f, "snapshot writer follows heima switch-on status enable command");
    require(rx.TargetPosition == 500,
            "snapshot writer final target position follows setMotorTarget second pass like original heimaSDK");

    targets.targets[0].enabled = 0;
    RmdCanSdk::writeEthercatDomainSnapshot(domain.data(), domain.size(), bindings, targets, actuals);
    std::memcpy(&rx, domain.data() + binding.rxOffset, sizeof(rx));
    require(rx.ControlWord == 0x0006, "snapshot writer sends shutdown when target disabled");
    require(rx.TargetPosition == 500, "snapshot writer keeps disabled target values like original heimaSDK");

    targets.targets[0].enabled = -1;
    RmdCanSdk::writeEthercatDomainSnapshot(domain.data(), domain.size(), bindings, targets, actuals);
    std::memcpy(&rx, domain.data() + binding.rxOffset, sizeof(rx));
    require(rx.ControlWord == 0x0086, "snapshot writer sends heima fault-reset controlword for enabled -1");

    binding.rxProfile = RmdCanSdk::EthercatMtDeviceRxProfile::Standard;
    binding.rxSize = RmdCanSdk::ethercatMtDeviceRxPdoSize(binding.rxProfile);
    bindings[0] = binding;
    targets.targets[0].enabled = 1;
    targets.targets[0].mode = 10;
    targets.targets[0].tor = 5.0f;
    targets.targets[0].maxCurrent = 321;
    RmdCanSdk::writeEthercatDomainSnapshot(domain.data(), domain.size(), bindings, targets, actuals);
    RmdCanSdk::HeimaStandardRxData standardRx;
    std::memcpy(&standardRx, domain.data() + binding.rxOffset, sizeof(standardRx));
    require(standardRx.ControlWord == 0x000f, "snapshot writer supports standard RxPDO controlword");
    require(standardRx.MaxTorque == 321, "snapshot writer writes setMaxCurr into standard RxPDO MaxTorque");
}

void testEthercatSnapshotWritesPrepackedTargetsAndRealtimeControlWord() {
    require(sizeof(RmdCanSdk::EthercatPackedTarget) <= 32,
            "prepacked EtherCAT target keeps one compact max-sized RxPDO byte block per motor");

    std::array<std::uint8_t, 128> domain{};

    RmdCanSdk::MotorParameters params;
    params.encoderResolution = 1000.0f;
    params.ratedCurrent = 10.0f;
    params.torqueConstant = 2.0f;
    params.maximumTorque = 10.0f;
    params.minimumPosition = -10.0f;
    params.maximumPosition = 10.0f;

    RmdCanSdk::EthercatPdoBinding binding;
    binding.globalIndex = 0;
    binding.rxOffset = 16;
    binding.rxProfile = RmdCanSdk::EthercatMtDeviceRxProfile::Pvt;
    binding.rxSize = RmdCanSdk::ethercatMtDeviceRxPdoSize(binding.rxProfile);
    binding.parameters = params;
    std::vector<RmdCanSdk::EthercatPdoBinding> bindings{binding};

    RmdCanSdk::MotorTarget target;
    target.pos = RmdCanSdk::Pi;
    target.vel = RmdCanSdk::Pi / 2.0f;
    target.tor = 1.0f;
    target.kp = 0.123f;
    target.kd = 0.456f;
    target.mode = 5;
    target.enabled = 1;

    RmdCanSdk::EthercatPackedTargetFrame targets;
    targets.motorCount = 1;
    targets.targets[0] =
        RmdCanSdk::packEthercatTargetForRealtime(target, params, RmdCanSdk::EthercatMtDeviceRxProfile::Pvt);
    targets.valid.set(0);

    RmdCanSdk::MotorActualFrame actuals;
    actuals.motorCount = 1;
    actuals.actuals[0].statusWord = 0x0031;
    actuals.valid.set(0);

    RmdCanSdk::writeEthercatDomainSnapshot(domain.data(), domain.size(), bindings, targets, actuals);

    RmdCanSdk::HeimaPvtRxData rx{};
    std::memcpy(&rx, domain.data() + binding.rxOffset, sizeof(rx));
    require(rx.ControlWord == 0x0007,
            "prepacked snapshot writer applies realtime ControlWord from latest StatusWord");
    require(rx.TargetPosition == 500, "prepacked snapshot writer keeps preconverted target position");
    require(rx.TargetVelocity == 250, "prepacked snapshot writer keeps preconverted target velocity");
    require(rx.TargetTorque == 50, "prepacked snapshot writer keeps preconverted target torque");
    require(rx.PvtKp == 123, "prepacked snapshot writer keeps preconverted PVT kp");
    require(rx.PvtKd == 456, "prepacked snapshot writer keeps preconverted PVT kd");

    binding.rxProfile = RmdCanSdk::EthercatMtDeviceRxProfile::Standard;
    binding.rxSize = RmdCanSdk::ethercatMtDeviceRxPdoSize(binding.rxProfile);
    bindings[0] = binding;
    target.mode = 10;
    target.tor = 10.0f;
    target.maxCurrent = 120;
    targets.targets[0] =
        RmdCanSdk::packEthercatTargetForRealtime(target, params, RmdCanSdk::EthercatMtDeviceRxProfile::Standard);
    actuals.actuals[0].statusWord = 0x0033;

    RmdCanSdk::writeEthercatDomainSnapshot(domain.data(), domain.size(), bindings, targets, actuals);

    RmdCanSdk::HeimaStandardRxData standardRx{};
    std::memcpy(&standardRx, domain.data() + binding.rxOffset, sizeof(standardRx));
    require(standardRx.ControlWord == 0x000f,
            "prepacked standard snapshot writer applies realtime enable-operation ControlWord");
    require(standardRx.TargetTorque == 120, "prepacked standard snapshot writer keeps clamped CST torque");
    require(standardRx.MaxTorque == 120, "prepacked standard snapshot writer keeps MaxTorque");
}

void testMtDevicePdoSpecMatchesOriginalHeimaConfig() {
    RmdCanSdk::EthercatMtDevicePdoSpec const& spec = RmdCanSdk::ethercatMtDevicePdoSpec();

    require(spec.vendorId == 0x00202008u, "MT_Device vendor id matches heima config.xml");
    require(spec.productCode == 0x00000000u, "MT_Device product code matches heima config.xml");
    require(spec.rxAssignmentIndex == 0x1c12, "MT_Device RxPDO assignment object matches heima ecat.cpp");
    require(spec.txAssignmentIndex == 0x1c13, "MT_Device TxPDO assignment object matches heima ecat.cpp");

    auto const& standardRx = RmdCanSdk::ethercatMtDeviceRxPdoSpec(RmdCanSdk::EthercatMtDeviceRxProfile::Standard);
    require(standardRx.pdoIndex == 0x1600, "MT_Device standard RxPDO remaps to 0x1600");
    std::array<std::uint16_t, 7> const standardRxIndexes{0x6040, 0x607a, 0x60ff, 0x6071,
                                                         0x6072, 0x6060, 0x2ffd};
    std::array<std::uint8_t, 7> const standardRxBits{16, 32, 32, 16, 16, 8, 8};
    for (std::size_t i = 0; i < standardRxIndexes.size(); ++i) {
        require(standardRx.entries[i].index == standardRxIndexes[i],
                "MT_Device standard RxPDO entry order matches MT-Device_260224.xml");
        require(standardRx.entries[i].subindex == 0, "MT_Device standard RxPDO subindex is 0");
        require(standardRx.entries[i].bitLength == standardRxBits[i], "MT_Device standard RxPDO bit length matches XML");
    }

    auto const& pvtRx = RmdCanSdk::ethercatMtDeviceRxPdoSpec(RmdCanSdk::EthercatMtDeviceRxProfile::Pvt);
    require(pvtRx.pdoIndex == 0x1601, "MT_Device PVT RxPDO remaps to 0x1601");
    std::array<std::uint16_t, 8> const pvtRxIndexes{0x6040, 0x607a, 0x60ff, 0x6071,
                                                    0x2000, 0x2001, 0x6060, 0x2ffd};
    std::array<std::uint8_t, 8> const pvtRxBits{16, 32, 32, 16, 32, 32, 8, 8};
    for (std::size_t i = 0; i < pvtRxIndexes.size(); ++i) {
        require(pvtRx.entries[i].index == pvtRxIndexes[i], "MT_Device PVT RxPDO entry order matches XML");
        require(pvtRx.entries[i].subindex == 0, "MT_Device PVT RxPDO subindex is 0");
        require(pvtRx.entries[i].bitLength == pvtRxBits[i], "MT_Device PVT RxPDO bit length matches XML");
    }

    auto const& tx = RmdCanSdk::ethercatMtDeviceTxPdoSpec(RmdCanSdk::EthercatMtDeviceTxProfile::Extended);
    require(tx.pdoIndex == 0x1a02, "MT_Device extended TxPDO remaps to 0x1A02");
    std::array<std::uint16_t, 10> const txIndexes{0x6041, 0x6064, 0x606c, 0x6077, 0x603f,
                                                   0x2009, 0x200c, 0x200a, 0x6061, 0x2ffe};
    std::array<std::uint8_t, 10> const txBits{16, 32, 32, 16, 16, 16, 16, 16, 8, 8};
    for (std::size_t i = 0; i < txIndexes.size(); ++i) {
        require(tx.entries[i].index == txIndexes[i], "MT_Device TxPDO entry order matches heima config.xml");
        require(tx.entries[i].subindex == 0, "MT_Device TxPDO subindex is 0");
        require(tx.entries[i].bitLength == txBits[i], "MT_Device TxPDO bit length matches heima config.xml");
    }

    require(RmdCanSdk::ethercatMtDeviceRxProfileForMode(5) == RmdCanSdk::EthercatMtDeviceRxProfile::Pvt,
            "MT_Device mode 5 selects PVT RxPDO profile");
    require(RmdCanSdk::ethercatMtDeviceRxProfileForMode(8) == RmdCanSdk::EthercatMtDeviceRxProfile::Standard,
            "MT_Device mode 8 selects standard RxPDO profile");
    require(RmdCanSdk::ethercatMtDeviceRxProfileForMode(9) == RmdCanSdk::EthercatMtDeviceRxProfile::Standard,
            "MT_Device mode 9 selects standard RxPDO profile");
    require(RmdCanSdk::ethercatMtDeviceRxProfileForMode(10) == RmdCanSdk::EthercatMtDeviceRxProfile::Standard,
            "MT_Device mode 10 selects standard RxPDO profile");

    require(RmdCanSdk::isEthercatMtDeviceType("MT_Device"), "MT_Device type accepts original underscore spelling");
    require(RmdCanSdk::isEthercatMtDeviceType("MT-Device"), "MT_Device type accepts standalone config hyphen spelling");
}

void testSafetySupervisorClampsTargetsAndMarksStaleActuals() {
    RmdCanSdk::MotorParameters params;
    params.minimumPosition = -1.0f;
    params.maximumPosition = 1.0f;
    params.maximumTorque = 10.0f;

    RmdCanSdk::MotorTarget target;
    target.pos = 2.5f;
    target.tor = -12.0f;
    target.kp = 600.0f;
    target.kd = 60.0f;
    target.enabled = 1;

    auto limited = RmdCanSdk::limitTarget(target, params);
    require(limited.clamped, "out-of-range target reports clamped");
    require(std::fabs(limited.target.pos - 1.0f) < 0.0001f, "position clamps to maximumPosition");
    require(std::fabs(limited.target.tor + 10.0f) < 0.0001f, "torque clamps to maximumTorque");
    require(std::fabs(limited.target.kp - RmdCanSdk::RmdKpMax) < 0.0001f, "kp clamps to protocol max");
    require(std::fabs(limited.target.kd - RmdCanSdk::RmdKdMax) < 0.0001f, "kd clamps to protocol max");

    RmdCanSdk::MotorActual actual;
    actual.statusWord = 0x0237;
    actual.errorCode = 0;
    RmdCanSdk::markActualStale(actual);
    require(actual.statusWord == 0xffff, "stale actual statusWord is public fault");
    require(actual.errorCode == RmdCanSdk::ErrorCodeFeedbackTimeout, "stale actual errorCode is timeout");
}

void testBenchWorkflowHelpersUseSharedMotorSemantics() {
    RmdCanSdk::Config config = RmdCanSdk::loadConfig(writeTempConfig());

    RmdCanSdk::MotorParameters const* first = RmdCanSdk::findParamsForAlias(config, 1);
    require(first != nullptr, "bench workflow finds motor parameters by alias");
    require(std::fabs(first->maximumPosition - 1.0f) < 0.0001f,
            "bench workflow returns the matching motor parameter block");
    require(RmdCanSdk::findParamsForAlias(config, 99) == nullptr,
            "bench workflow reports missing motor parameters");

    require(!RmdCanSdk::operationEnabled(0x0031), "bench workflow rejects switched-on-disabled status");
    require(RmdCanSdk::operationEnabled(0x0037), "bench workflow accepts operation-enabled status");
    require(RmdCanSdk::operationEnabled(0x0237), "bench workflow masks vendor/status high bits");
}

class FakeBackend final : public RmdCanSdk::MotorBackend {
public:
    int start() override {
        status_.running = true;
        return 0;
    }

    void stop() override {
        status_.running = false;
    }

    RmdCanSdk::BackendStatus status() const override {
        return status_;
    }

private:
    RmdCanSdk::BackendStatus status_;
};

void testMotorBackendInterface() {
    FakeBackend backend;
    require(backend.start() == 0, "backend start returns success");
    require(backend.status().running, "backend reports running after start");
    backend.stop();
    require(!backend.status().running, "backend reports stopped after stop");
}

void testAtomicBackendStatusPublishesDeterministicMetrics() {
    RmdCanSdk::AtomicBackendStatus status;
    status.setRunning(true);
    status.recordCycle(900, 1000);
    status.recordCycle(1200, 1000);
    status.recordRxTimeout();
    status.recordWcIncomplete();
    status.recordStaleFrame();
    status.setFault(RmdCanSdk::ErrorCodeFeedbackTimeout);

    RmdCanSdk::BackendStatus snapshot = status.snapshot();
    require(snapshot.running, "atomic backend status reports running");
    require(snapshot.degraded, "atomic backend status reports degraded after fault");
    require(snapshot.errorCode == RmdCanSdk::ErrorCodeFeedbackTimeout, "atomic backend status publishes error code");
    require(snapshot.cycleCount == 2, "atomic backend status counts cycles");
    require(snapshot.deadlineMissCount == 1, "atomic backend status counts deadline misses");
    require(snapshot.lastCycleNs == 1200, "atomic backend status publishes last cycle duration");
    require(snapshot.maxCycleNs == 1200, "atomic backend status keeps max cycle duration");
    require(snapshot.rxTimeoutCount == 1, "atomic backend status counts CAN RX timeouts");
    require(snapshot.wcIncompleteCount == 1, "atomic backend status counts EtherCAT incomplete WKC domains");
    require(snapshot.staleFrameCount == 1, "atomic backend status counts stale published frames");
}

void testRmdCanBackendConstructsWithoutOpeningSocket() {
    RmdCanSdk::Config config = RmdCanSdk::loadConfig(writeTempConfig());
    RmdCanSdk::MotorRegistry registry = RmdCanSdk::MotorRegistry::fromConfig(config);
    RmdCanSdk::FrameBuffer<RmdCanSdk::MotorTargetFrame> targets;
    RmdCanSdk::FrameBuffer<RmdCanSdk::MotorActualFrame> actuals;
    RmdCanSdk::RmdCanBackend backend(config, registry, 0, targets, actuals);
    require(!backend.status().running, "CAN backend is stopped before start");
}

void testEthercatPdoCodecParsesPackedTxData() {
    RmdCanSdk::EthercatTxPdoBytes bytes;
    bytes.StatusWord = 0x0237;
    bytes.ActualPosition = 600;
    bytes.ActualVelocity = 250;
    bytes.ActualTorque = 50;
    bytes.ErrorCode = 0;
    bytes.MotorTemperature = 35;
    bytes.DriveTemperature = 40;
    bytes.Voltage = 680;

    RmdCanSdk::MotorParameters params;
    params.polarity = -1.0f;
    params.countBias = 100.0f;
    params.encoderResolution = 1000.0f;
    params.gearRatioPosVel = 20.0f;
    params.gearRatioTor = 20.0f;
    params.ratedCurrent = 10.0f;
    params.torqueConstant = 2.0f;

    auto actual = RmdCanSdk::parseEthercatTxPdo(bytes, params);
    require(std::fabs(actual.pos + RmdCanSdk::Pi) < 0.0001f,
            "heima EtherCAT TxPDO position treats countBias as counts and ignores GearRatioPosVel");
    require(std::fabs(actual.vel + RmdCanSdk::Pi / 2.0f) < 0.0001f,
            "heima EtherCAT TxPDO velocity ignores GearRatioPosVel");
    require(std::fabs(actual.tor + 1.0f) < 0.0001f,
            "heima EtherCAT TxPDO torque decodes per-mille rated current torque");
    require(actual.encoderCount == 600, "heima EtherCAT TxPDO keeps raw ActualPosition for getEncoderCount");
    require(actual.statusWord == 0x0237, "EtherCAT TxPDO statusWord parses");
    require(actual.errorCode == 0, "EtherCAT TxPDO errorCode parses");
    require(actual.temp == 35, "EtherCAT TxPDO motor temperature parses");
    require(actual.driveTemp == 40, "EtherCAT TxPDO drive temperature parses");
    require(actual.voltage == 680, "EtherCAT TxPDO voltage parses");
}

void testEthercatPdoCodecPacksHeimaRxData() {
    RmdCanSdk::MotorParameters params;
    params.encoderResolution = 1000.0f;
    params.gearRatioPosVel = 20.0f;
    params.gearRatioTor = 20.0f;
    params.countBias = 100.0f;
    params.polarity = -1.0f;
    params.ratedCurrent = 10.0f;
    params.torqueConstant = 2.0f;
    params.maximumTorque = 10.0f;
    params.minimumPosition = -10.0f;
    params.maximumPosition = 10.0f;

    RmdCanSdk::MotorTarget target;
    target.pos = RmdCanSdk::Pi;
    target.vel = RmdCanSdk::Pi / 2.0f;
    target.tor = 5.0f;
    target.kp = 12.3f;
    target.kd = 4.6f;
    target.enabled = 1;
    target.mode = 8;

    RmdCanSdk::EthercatRxPdoBytes rx = RmdCanSdk::packEthercatRxPdo(target, params);

    require(rx.ControlWord == 0x000f, "enabled heima RxPDO uses enable-operation controlword");
    require(rx.TargetPosition == -400,
            "heima RxPDO target position treats countBias as counts and ignores GearRatioPosVel");
    require(rx.TargetVelocity == -250, "heima RxPDO target velocity ignores GearRatioPosVel");
    require(rx.TargetTorque == 0, "heima mode 8 RxPDO zeros feedforward torque like original SDK");
    require(rx.PvtKp == 0, "heima mode 8 RxPDO zeros PVT kp like original SDK");
    require(rx.PvtKd == 0, "heima mode 8 RxPDO zeros PVT kd like original SDK");
    require(rx.Mode == 8, "heima RxPDO mode follows target mode");
    require(rx.Undefined == 0, "heima RxPDO padding byte remains zero");

    target.mode = 5;
    params.polarity = 1.0f;
    params.countBias = 0.0f;
    target.tor = 1.0f;
    target.kp = 0.123f;
    target.kd = 0.456f;
    rx = RmdCanSdk::packEthercatRxPdo(target, params);
    require(rx.TargetTorque == 50, "heima mode 5 RxPDO packs torque as per-mille rated current torque");
    require(rx.PvtKp == 123, "heima mode 5 RxPDO scales kp by 1000");
    require(rx.PvtKd == 456, "heima mode 5 RxPDO scales kd by 1000");
    require(rx.Mode == 5, "heima mode 5 RxPDO writes requested mode");

    target.enabled = 0;
    rx = RmdCanSdk::packEthercatRxPdo(target, params);
    require(rx.ControlWord == 0x0006, "disabled heima RxPDO uses shutdown controlword");
    require(rx.TargetPosition == 500,
            "disabled heima RxPDO still packs target position like original setMotorTarget second pass");

    target.enabled = -1;
    rx = RmdCanSdk::packEthercatRxPdo(target, params);
    require(rx.ControlWord == 0x0006, "raw pack uses shutdown until snapshot applies fault-reset controlword");

    target.enabled = 1;
    target.mode = 10;
    target.tor = 10.0f;
    target.maxCurrent = 120;
    RmdCanSdk::HeimaStandardRxData standardRx = RmdCanSdk::packEthercatStandardRxPdo(target, params);
    require(standardRx.TargetTorque == 120, "heima CST standard RxPDO clamps target torque raw value by setMaxCurr");
    require(standardRx.MaxTorque == 120, "heima CST standard RxPDO writes setMaxCurr to 0x6072 MaxTorque");

    params.polarity = -1.0f;
    standardRx = RmdCanSdk::packEthercatStandardRxPdo(target, params);
    require(standardRx.TargetTorque == -120,
            "heima CST standard RxPDO clamps negative polarity target torque by setMaxCurr");
    params.polarity = 1.0f;

    require(RmdCanSdk::ethercatMtDeviceControlWord(1, 0x0031) == 0x0007,
            "heima CiA402 state machine sends switch-on command for status 0x31");
    require(RmdCanSdk::ethercatMtDeviceControlWord(1, 0x0033) == 0x000f,
            "heima CiA402 state machine sends enable-operation command for status 0x33");
    require(RmdCanSdk::ethercatMtDeviceControlWord(1, 0x0037) == 0x000f,
            "heima CiA402 state machine keeps enable-operation command for status 0x37");
    require(RmdCanSdk::ethercatMtDeviceControlWord(1, 0xffff) == 0x0006,
            "heima CiA402 state machine falls back to shutdown for unknown status");
    require(RmdCanSdk::ethercatMtDeviceControlWord(-1, 0x0037) == 0x0086,
            "heima CiA402 state machine sends fault-reset controlword for enabled -1");
}

void testHeimaEcatPdoLayoutMatchesOriginal() {
    require(sizeof(RmdCanSdk::HeimaStandardRxData) == 16, "heima MT_Device standard RxPDO packs to 16 bytes");
    require(offsetof(RmdCanSdk::HeimaStandardRxData, ControlWord) == 0, "standard RxPDO ControlWord offset matches XML");
    require(offsetof(RmdCanSdk::HeimaStandardRxData, TargetPosition) == 2,
            "standard RxPDO TargetPosition offset matches XML");
    require(offsetof(RmdCanSdk::HeimaStandardRxData, TargetVelocity) == 6,
            "standard RxPDO TargetVelocity offset matches XML");
    require(offsetof(RmdCanSdk::HeimaStandardRxData, TargetTorque) == 10,
            "standard RxPDO TargetTorque offset matches XML");
    require(offsetof(RmdCanSdk::HeimaStandardRxData, MaxTorque) == 12,
            "standard RxPDO MaxTorque offset matches XML");
    require(offsetof(RmdCanSdk::HeimaStandardRxData, Mode) == 14, "standard RxPDO Mode offset matches XML");

    require(sizeof(RmdCanSdk::HeimaPvtRxData) == 22, "heima MT_Device PVT RxPDO remains packed to 22 bytes");
    require(offsetof(RmdCanSdk::HeimaPvtRxData, ControlWord) == 0, "PVT RxPDO ControlWord offset matches heima");
    require(offsetof(RmdCanSdk::HeimaPvtRxData, TargetPosition) == 2, "PVT RxPDO TargetPosition offset matches heima");
    require(offsetof(RmdCanSdk::HeimaPvtRxData, TargetVelocity) == 6, "PVT RxPDO TargetVelocity offset matches heima");
    require(offsetof(RmdCanSdk::HeimaPvtRxData, TargetTorque) == 10, "PVT RxPDO TargetTorque offset matches heima");
    require(offsetof(RmdCanSdk::HeimaPvtRxData, PvtKp) == 12, "PVT RxPDO PvtKp offset matches heima");
    require(offsetof(RmdCanSdk::HeimaPvtRxData, PvtKd) == 16, "PVT RxPDO PvtKd offset matches heima");
    require(offsetof(RmdCanSdk::HeimaPvtRxData, Mode) == 20, "PVT RxPDO Mode offset matches heima");

    require(sizeof(RmdCanSdk::HeimaDriverTxData) == 22, "heima MT_Device TxPDO remains packed to 22 bytes");
    require(offsetof(RmdCanSdk::HeimaDriverTxData, StatusWord) == 0, "TxPDO StatusWord offset matches heima");
    require(offsetof(RmdCanSdk::HeimaDriverTxData, ActualPosition) == 2, "TxPDO ActualPosition offset matches heima");
    require(offsetof(RmdCanSdk::HeimaDriverTxData, ActualVelocity) == 6, "TxPDO ActualVelocity offset matches heima");
    require(offsetof(RmdCanSdk::HeimaDriverTxData, ActualTorque) == 10, "TxPDO ActualTorque offset matches heima");
    require(offsetof(RmdCanSdk::HeimaDriverTxData, ErrorCode) == 12, "TxPDO ErrorCode offset matches heima");
    require(offsetof(RmdCanSdk::HeimaDriverTxData, MotorTemperature) == 14,
            "TxPDO MotorTemperature offset matches heima");
    require(offsetof(RmdCanSdk::HeimaDriverTxData, DriveTemperature) == 16,
            "TxPDO DriveTemperature offset matches heima");
    require(offsetof(RmdCanSdk::HeimaDriverTxData, Voltage) == 18, "TxPDO Voltage offset matches heima");
    require(offsetof(RmdCanSdk::HeimaDriverTxData, ModeDisplay) == 20, "TxPDO ModeDisplay offset matches heima");
}

void testHeimaHalfFloatConversionMatchesOriginal() {
    require(RmdCanSdk::heimaSingle2Half(0.0f) == 0x0000, "heima half conversion encodes +0");
    require(RmdCanSdk::heimaSingle2Half(-0.0f) == 0x8000, "heima half conversion encodes -0");
    require(RmdCanSdk::heimaSingle2Half(1.0f) == 0x3c00, "heima half conversion encodes 1.0");
    require(RmdCanSdk::heimaSingle2Half(-2.0f) == 0xc000, "heima half conversion encodes -2.0");
    require(std::fabs(RmdCanSdk::heimaHalf2Single(0x3c00) - 1.0f) < 0.0001f,
            "heima half conversion decodes 1.0");
    require(std::fabs(RmdCanSdk::heimaHalf2Single(0xc000) + 2.0f) < 0.0001f,
            "heima half conversion decodes -2.0");
}

void testDriverSdkReportsUninitializedBeforeInit() {
    DriverSDK::DriverSDK& sdk = DriverSDK::DriverSDK::instance();
    std::vector<DriverSDK::motorTargetStruct> targets(1);
    std::vector<DriverSDK::motorActualStruct> actuals(1);
    require(sdk.setMotorTarget(targets) == std::numeric_limits<int>::min(),
            "setMotorTarget rejects calls before successful init");
    require(sdk.getMotorActual(actuals) == std::numeric_limits<int>::min(),
            "getMotorActual rejects calls before successful init");
}

void testDriverSdkRejectsSetModeAfterInit() {
    DriverSDK::DriverSDK& sdk = DriverSDK::DriverSDK::instance();
    sdk.init(writeTempEmptyConfig().c_str());
    std::vector<char> modes;
    require(sdk.setMode(modes) == -1, "setMode is rejected after init so realtime mode table is frozen");
}

void testDriverSdkFailsWhenConfiguredImuCannotStart() {
    DriverSDK::DriverSDK& sdk = DriverSDK::DriverSDK::instance();
    bool threw = false;
    try {
        sdk.init(writeTempImuOnlyConfig().c_str());
    } catch (...) {
        threw = true;
    }
    require(threw, "DriverSDK init fails when configured YeSense IMU cannot start");
}

void testImuSnapshotBufferReturnsWholeSample() {
    RmdCanSdk::ImuSnapshotBuffer buffer;
    DriverSDK::imuStruct sample;
    sample.rpy[0] = 1.0f;
    sample.rpy[1] = 1.0f;
    sample.rpy[2] = 1.0f;
    buffer.publish(sample);

    DriverSDK::imuStruct out;
    buffer.readInto(out);
    require(out.rpy[0] == 1.0f && out.rpy[1] == 1.0f && out.rpy[2] == 1.0f,
            "IMU snapshot returns a whole published sample");
}

void testRs232ImuBackendAcceptsYesenseType() {
    RmdCanSdk::ImuSnapshotBuffer buffer;
    RmdCanSdk::Rs232ImuBackend backend(buffer);

    require(RmdCanSdk::isSupportedImuType("YeSense"), "RS232 IMU backend accepts YeSense type");
    require(RmdCanSdk::isSupportedImuType("YESENSE"), "RS232 IMU backend accepts normalized YeSense type");
    require(RmdCanSdk::isSupportedImuType("HiPNUC"), "RS232 IMU backend still accepts HiPNUC type");
    require(RmdCanSdk::isSupportedImuType("Xsens"), "RS232 IMU backend still accepts Xsens type");
    require(!RmdCanSdk::isSupportedImuType("not-an-imu"), "RS232 IMU backend rejects unsupported IMU type");
    require(backend.start("/tmp/rmd_missing_yesense_imu", 921600, "YeSense") == -1,
            "RS232 IMU backend rejects missing YeSense serial device");
    require(backend.start("/tmp/rmd_missing_yesense_imu", 921600, "not-an-imu") == -1,
            "RS232 IMU backend rejects unsupported IMU type");
}

void testYesenseOutputConvertsToImuStruct() {
    yesense::yis_out_data_t sample{};
    sample.content.valid_flg = 1;
    sample.content.euler = 1;
    sample.content.gyro = 1;
    sample.content.acc = 1;
    sample.euler.roll = 10.0f;
    sample.euler.pitch = -20.0f;
    sample.euler.yaw = 30.0f;
    sample.gyro.x = 40.0f;
    sample.gyro.y = -50.0f;
    sample.gyro.z = 60.0f;
    sample.acc.x = 1.25f;
    sample.acc.y = -2.5f;
    sample.acc.z = 9.81f;

    DriverSDK::imuStruct imu = RmdCanSdk::yesenseOutputToImu(sample);

    require(std::fabs(imu.rpy[0] - 10.0f * RmdCanSdk::Pi / 180.0f) < 0.0001f,
            "YeSense roll converts deg to rad");
    require(std::fabs(imu.rpy[1] - (-20.0f * RmdCanSdk::Pi / 180.0f)) < 0.0001f,
            "YeSense pitch converts deg to rad");
    require(std::fabs(imu.rpy[2] - 30.0f * RmdCanSdk::Pi / 180.0f) < 0.0001f,
            "YeSense yaw converts deg to rad");
    require(std::fabs(imu.gyr[0] - 40.0f * RmdCanSdk::Pi / 180.0f) < 0.0001f,
            "YeSense gyro x converts deg/s to rad/s");
    require(std::fabs(imu.gyr[1] - (-50.0f * RmdCanSdk::Pi / 180.0f)) < 0.0001f,
            "YeSense gyro y converts deg/s to rad/s");
    require(std::fabs(imu.gyr[2] - 60.0f * RmdCanSdk::Pi / 180.0f) < 0.0001f,
            "YeSense gyro z converts deg/s to rad/s");
    require(std::fabs(imu.acc[0] - 1.25f) < 0.0001f, "YeSense acc x is preserved");
    require(std::fabs(imu.acc[1] + 2.5f) < 0.0001f, "YeSense acc y is preserved");
    require(std::fabs(imu.acc[2] - 9.81f) < 0.0001f, "YeSense acc z is preserved");
}

void testMaskTracker() {
    RmdCanSdk::MaskTracker mask;
    mask.setExpectedFromSlaveIds({14, 15});
    require(!mask.markReceived(14), "first feedback must not advance complete mask");
    require(mask.markReceived(15), "second feedback completes mask");
    require(mask.currentMask() == 0, "mask resets after completion");
    require(!mask.markReceived(16), "unexpected slave id ignored");
}

void testFeedbackFrameTrackerTimeout() {
    using Tracker = RmdCanSdk::FeedbackFrameTracker;
    Tracker tracker;
    tracker.setExpectedFromSlaveIds({14, 15});
    auto const start = Tracker::Clock::time_point{};
    auto const timeout = std::chrono::milliseconds(20);

    require(!tracker.markReceived(14, start), "first feedback starts partial frame");
    require(tracker.isReceived(14), "received slave is tracked before publish");
    require(!tracker.isReceived(15), "missing slave remains unreceived");
    require(!tracker.shouldPublishTimeout(start + std::chrono::milliseconds(19), timeout),
            "partial frame does not publish before timeout");
    require(tracker.shouldPublishTimeout(start + timeout, timeout),
            "partial frame publishes when timeout expires");
    tracker.reset();
    require(!tracker.isReceived(14), "reset clears received mask after partial publish");
}

void testMitSineTargetsStayInsideConfiguredPositionLimits() {
    constexpr float DegToRad = RmdCanSdk::Pi / 180.0f;
    RmdCanSdk::MotorParameters motor3;
    motor3.minimumPosition = -0.7f;
    motor3.maximumPosition = 1.2f;
    RmdCanSdk::MotorParameters motor4;
    motor4.minimumPosition = -1.5f;
    motor4.maximumPosition = 0.05f;

    RmdCanSdk::SineTargetSpec motor3Spec;
    motor3Spec.centerRad = 0.0f;
    motor3Spec.amplitudeRad = 10.0f * DegToRad;
    auto motor3Check = RmdCanSdk::checkSineTargetWithinLimits(motor3Spec, motor3, 2.0f * DegToRad);
    require(motor3Check.valid, "motor 3 +/-10deg sine target fits inside configured limits with margin");

    RmdCanSdk::SineTargetSpec motor4Spec;
    motor4Spec.centerRad = -75.0f * DegToRad;
    motor4Spec.amplitudeRad = 8.0f * DegToRad;
    auto motor4Check = RmdCanSdk::checkSineTargetWithinLimits(motor4Spec, motor4, 2.0f * DegToRad);
    require(motor4Check.valid, "motor 4 -75deg center +/-8deg sine target keeps margin from lower limit");
    require(std::fabs(motor4Check.minTargetRad - (-83.0f * DegToRad)) < 0.0001f,
            "motor 4 sine minimum is reported");
    require(std::fabs(motor4Check.maxTargetRad - (-67.0f * DegToRad)) < 0.0001f,
            "motor 4 sine maximum is reported");

    RmdCanSdk::SineTargetSpec unsafeMotor4Spec;
    unsafeMotor4Spec.centerRad = -85.0f * DegToRad;
    unsafeMotor4Spec.amplitudeRad = 8.0f * DegToRad;
    auto unsafeCheck = RmdCanSdk::checkSineTargetWithinLimits(unsafeMotor4Spec, motor4, 2.0f * DegToRad);
    require(!unsafeCheck.valid, "unsafe motor 4 sine target below MinimumPosition is rejected");

    auto safeActual = RmdCanSdk::checkAngleWithinLimits(-75.0f * DegToRad, motor4, 2.0f * DegToRad);
    require(safeActual.valid, "standard 0x92 angle inside configured limits passes hard monitor");
    auto unsafeActual = RmdCanSdk::checkAngleWithinLimits(-84.5f * DegToRad, motor4, 2.0f * DegToRad);
    require(!unsafeActual.valid, "standard 0x92 angle too close to lower limit fails hard monitor");

    RmdCanSdk::SineTargetSpec startsAtUpperEnd;
    startsAtUpperEnd.centerRad = -25.0f * DegToRad;
    startsAtUpperEnd.amplitudeRad = 25.0f * DegToRad;
    startsAtUpperEnd.phaseRad = 90.0f * DegToRad;
    require(std::fabs(RmdCanSdk::sineTargetAt(startsAtUpperEnd, 0.0f, 0.2f) - 0.0f) < 0.0001f,
            "phase 90deg starts a sine target at center plus amplitude");
}

void testFrameBufferSnapshotsAreWholeFrames() {
    using Frame = RmdCanSdk::MotorActualFrame;
    RmdCanSdk::FrameBuffer<Frame> buffer;
    std::atomic<bool> done{false};

    std::thread producer([&]() {
        for (std::uint64_t seq = 1; seq <= 20000; ++seq) {
            Frame frame;
            frame.sequence = seq;
            frame.motorCount = 3;
            for (std::size_t i = 0; i < frame.motorCount; ++i) {
                frame.actuals[i].pos = static_cast<float>(seq);
                frame.actuals[i].vel = static_cast<float>(seq);
                frame.actuals[i].tor = static_cast<float>(seq);
                frame.valid.set(i);
            }
            buffer.publish(frame);
        }
        done.store(true, std::memory_order_release);
    });

    Frame snapshot;
    while (!done.load(std::memory_order_acquire)) {
        buffer.readInto(snapshot);
        for (std::size_t i = 0; i < snapshot.motorCount; ++i) {
            require(snapshot.actuals[i].pos == snapshot.actuals[i].vel &&
                        snapshot.actuals[i].vel == snapshot.actuals[i].tor,
                    "FrameBuffer consumer must observe whole MotorActualFrame values");
        }
    }
    producer.join();
}

void testFrameBufferSequenceNeverMovesBackward() {
    using Frame = RmdCanSdk::MotorActualFrame;
    RmdCanSdk::FrameBuffer<Frame> buffer;
    std::atomic<bool> done{false};

    std::thread producer([&]() {
        for (std::uint64_t seq = 1; seq <= 100000; ++seq) {
            Frame frame;
            frame.sequence = seq;
            frame.motorCount = 1;
            frame.actuals[0].pos = static_cast<float>(seq);
            frame.valid.set(0);
            buffer.publish(frame);
            if ((seq % 7) == 0) {
                std::this_thread::yield();
            }
        }
        done.store(true, std::memory_order_release);
    });

    Frame snapshot;
    std::uint64_t lastSeen = 0;
    while (!done.load(std::memory_order_acquire)) {
        buffer.readInto(snapshot);
        require(snapshot.sequence >= lastSeen, "FrameBuffer must not rotate an older published frame back to reader");
        lastSeen = snapshot.sequence;
    }
    producer.join();
}

void testTripleBufferSnapshotsAreWholeFrames() {
    RmdCanSdk::TripleBuffer<std::vector<int>> buffer(std::vector<int>{0, 0, 0});
    std::atomic<bool> done{false};
    std::thread producer([&]() {
        for (int i = 1; i <= 20000; ++i) {
            buffer.publish(std::vector<int>{i, i, i});
        }
        done.store(true, std::memory_order_release);
    });

    std::vector<int> snapshot{0, 0, 0};
    while (!done.load(std::memory_order_acquire)) {
        buffer.readInto(snapshot);
        require(snapshot[0] == snapshot[1] && snapshot[1] == snapshot[2],
                "triple buffer consumer must observe a whole published frame");
    }
    producer.join();
}

void testTripleBufferSequenceNeverMovesBackward() {
    RmdCanSdk::TripleBuffer<std::vector<int>> buffer(std::vector<int>{0, 0});
    std::atomic<bool> done{false};
    std::thread producer([&]() {
        for (int i = 1; i <= 100000; ++i) {
            buffer.publish(std::vector<int>{i, i});
            if ((i % 7) == 0) {
                std::this_thread::yield();
            }
        }
        done.store(true, std::memory_order_release);
    });

    std::vector<int> snapshot{0, 0};
    int lastSeen = 0;
    while (!done.load(std::memory_order_acquire)) {
        buffer.readInto(snapshot);
        require(snapshot[0] >= lastSeen, "TripleBuffer must not rotate an older published frame back to reader");
        require(snapshot[0] == snapshot[1], "TripleBuffer monotonic test must still observe whole frames");
        lastSeen = snapshot[0];
    }
    producer.join();
}

void testPublicApiCompiles() {
    DriverSDK::motorTargetStruct target;
    target.pos = 0.1f;
    target.enabled = 1;
    DriverSDK::motorActualStruct actual;
    (void)actual;
    DriverSDK::DriverSDK& sdk = DriverSDK::DriverSDK::instance();
    require(!sdk.version().empty(), "public DriverSDK API is available");
}

void testUnsupportedCompatibilityApisReturnExplicitUnsupported() {
    DriverSDK::DriverSDK& sdk = DriverSDK::DriverSDK::instance();
    DriverSDK::motorSDOClass sdo(0);
    DriverSDK::motorREGClass reg(0);
    require(sdk.fillSDO(sdo, "TargetPosition") == std::numeric_limits<int>::max(), "fillSDO reports unsupported");
    require(sdo.state == -1, "unsupported SDO marks state as error");
    require(sdk.sendMotorSDORequest(sdo) == std::numeric_limits<int>::max(), "sendMotorSDORequest reports unsupported");
    require(sdk.recvMotorSDOResponse(sdo) == std::numeric_limits<int>::max(), "recvMotorSDOResponse reports unsupported");
    require(sdk.sendMotorREGRequest(reg) == std::numeric_limits<int>::max(), "sendMotorREGRequest reports unsupported");
    require(sdk.recvMotorREGResponse(reg) == std::numeric_limits<int>::max(), "recvMotorREGResponse reports unsupported");
    require(sdk.calibrate(0) == std::numeric_limits<int>::max(), "calibrate reports unsupported");
}

} // namespace

int main() {
    testMitCodec();
    testMitV44ScalingUsesDynamicTorqueAndKd50();
    testCanProtocolParsesFixedFramesWithoutVectorAllocation();
    testStatus2TorqueEstimate();
    testMultiturnAngleParsesHundredthDegrees();
    testStatus1BrakeCommandAndErrorFlags();
    testObservationUsesMitPositionAndStatus2AuxiliaryData();
    testConfigParser();
    testConfigParserReadsYesenseImu();
    testMotorRegistryNormalizesCanMotors();
    testMixedBusConfigCreatesIndependentBackendGroups();
    testEthercatBindingTableKeepsHeimaAliasSlaveDomainAndOffsets();
    testOriginalStyleEthercatConfigCreatesMtDeviceMotor();
    testOrinSplitterEthercatConfigKeepsExplicitSlavePositions();
    testEthercatSnapshotUsesRegisteredPdoOffsets();
    testEthercatSnapshotWritesPrepackedTargetsAndRealtimeControlWord();
    testMtDevicePdoSpecMatchesOriginalHeimaConfig();
    testSafetySupervisorClampsTargetsAndMarksStaleActuals();
    testMotorBackendInterface();
    testAtomicBackendStatusPublishesDeterministicMetrics();
    testRmdCanBackendConstructsWithoutOpeningSocket();
    testEthercatPdoCodecParsesPackedTxData();
    testEthercatPdoCodecPacksHeimaRxData();
    testHeimaEcatPdoLayoutMatchesOriginal();
    testHeimaHalfFloatConversionMatchesOriginal();
    testBenchWorkflowHelpersUseSharedMotorSemantics();
    testDriverSdkReportsUninitializedBeforeInit();
    testDriverSdkRejectsSetModeAfterInit();
    testDriverSdkFailsWhenConfiguredImuCannotStart();
    testImuSnapshotBufferReturnsWholeSample();
    testRs232ImuBackendAcceptsYesenseType();
    testYesenseOutputConvertsToImuStruct();
    testMaskTracker();
    testFeedbackFrameTrackerTimeout();
    testMitSineTargetsStayInsideConfiguredPositionLimits();
    testFrameBufferSnapshotsAreWholeFrames();
    testFrameBufferSequenceNeverMovesBackward();
    testTripleBufferSnapshotsAreWholeFrames();
    testTripleBufferSequenceNeverMovesBackward();
    testPublicApiCompiles();
    testUnsupportedCompatibilityApisReturnExplicitUnsupported();
    std::cout << "rmd_can_sdk_tests passed\n";
    return 0;
}
