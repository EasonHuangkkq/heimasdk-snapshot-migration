#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace DriverSDK {

struct imuStruct {
    float rpy[3]{};
    float gyr[3]{};
    float acc[3]{};
};

struct sensorStruct {
    float F[3]{};
    float M[3]{};
    unsigned int statusCode = 0xffff;
};

struct digitTargetStruct {
    unsigned short pos = 0;
};

struct digitActualStruct {
    unsigned short pos = 0;
};

struct motorTargetStruct {
    float pos = 0.0f;
    float vel = 0.0f;
    float tor = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
    int enabled = 0;
};

struct motorActualStruct {
    float pos = 0.0f;
    float vel = 0.0f;
    float tor = 0.0f;
    short temp = 0;
    short driveTemp = 0;
    unsigned short voltage = 0;
    unsigned short statusWord = 65535;
    unsigned short errorCode = 0;
};

struct backendStatusStruct {
    bool running = false;
    bool degraded = false;
    int errorCode = 0;
    std::uint64_t cycleCount = 0;
    std::uint64_t deadlineMissCount = 0;
    std::uint64_t lastCycleNs = 0;
    std::uint64_t maxCycleNs = 0;
    std::uint64_t lateWakeupCount = 0;
    std::uint64_t lastWakeupLatencyNs = 0;
    std::uint64_t maxWakeupLatencyNs = 0;
    std::uint64_t staleFrameCount = 0;
    std::uint64_t rxTimeoutCount = 0;
    std::uint64_t wcIncompleteCount = 0;
};

class motorSDOClass {
public:
    explicit motorSDOClass(int i);
    long value = 0;
    int i = 0;
    short state = 3;
    unsigned short index = 0;
    unsigned char subindex = 0;
    unsigned char signed_ = 0;
    unsigned char bitLength = 0;
    unsigned char operation = 0;
};

class motorREGClass {
public:
    explicit motorREGClass(int i);
    long value = 0;
    int i = 0;
};

unsigned short single2half(float value);
float half2single(unsigned short value);

class DriverSDK {
public:
    static DriverSDK& instance();

    void setCPU(unsigned short cpu);
    int setCPUs(std::vector<unsigned short> const& cpus, std::string const& bus);
    void setMaxCurr(std::vector<unsigned short> const& maxCurr);
    int setMode(std::vector<char> const& mode);
    void init(char const* xmlFile);
    int getLeftDigitNr();
    int getRightDigitNr();
    int getTotalMotorNr();
    std::vector<int> getActiveMotors();
    int setCntBias(std::vector<int> const& cntBias);
    int fillSDO(motorSDOClass& data, char const* object);
    void getIMU(imuStruct& data);
    int getSensor(std::vector<sensorStruct>& data);
    int setDigitTarget(std::vector<digitTargetStruct> const& data);
    int getDigitActual(std::vector<digitActualStruct>& data);
    int setMotorTarget(std::vector<motorTargetStruct> const& data);
    int getMotorActual(std::vector<motorActualStruct>& data);
    int getBackendStatus(std::vector<backendStatusStruct>& data);
    int getEncoderCount(std::vector<int>& data);
    int sendMotorSDORequest(motorSDOClass const& data);
    int recvMotorSDOResponse(motorSDOClass& data);
    int sendMotorREGRequest(motorREGClass const& data);
    int recvMotorREGResponse(motorREGClass& data);
    int calibrate(int i);
    void advance();
    std::string version();

private:
    class Impl;
    Impl* impl_;

    DriverSDK();
    ~DriverSDK();
    DriverSDK(DriverSDK const&) = delete;
    DriverSDK& operator=(DriverSDK const&) = delete;
};

} // namespace DriverSDK
