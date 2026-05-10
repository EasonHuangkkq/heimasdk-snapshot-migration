# Unified RMD Realtime SDK Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a stable realtime SDK where the same public `DriverSDK` motor API controls RMD motors over CAN and EtherCAT, with RS232 IMU kept independent and all cross-thread motor data exchanged as complete snapshots.

**Architecture:** Add a small realtime core around whole-frame target and actual buffers, then isolate transport-specific code in `RmdCanBackend` and `RmdEthercatBackend`. Default builds keep EtherCAT disabled so the current SocketCAN SDK remains buildable on machines without IgH EtherCAT headers; target builds enable EtherCAT with `-DRMD_CAN_SDK_ENABLE_ECAT=ON`.

**Tech Stack:** C++20, CMake, SocketCAN, optional IgH EtherCAT (`ecrt.h`), existing tinyxml2 config parser, CTest.

---

## File Structure

Create these focused files:

- `include/rmd_can_sdk/rmd_realtime_core.h`: whole-frame metadata, fixed-capacity motor frames, and SPSC `FrameBuffer`.
- `include/rmd_can_sdk/rmd_motor_registry.h`: normalized runtime mapping from aliases to backend-local motor indexes.
- `src/rmd_motor_registry.cpp`: config-to-registry normalization.
- `include/rmd_can_sdk/rmd_safety.h`: target limiting and stale feedback policy.
- `src/rmd_safety.cpp`: safety policy implementation.
- `include/rmd_can_sdk/rmd_motor_backend.h`: backend interface and status structs.
- `include/rmd_can_sdk/rmd_can_backend.h`: CAN backend public class.
- `src/rmd_can_backend.cpp`: moved SocketCAN RMD runtime from current `src/rmd_driver_sdk.cpp`.
- `include/rmd_can_sdk/rmd_ethercat_pdo.h`: pure byte-level EtherCAT PDO pack/parse helpers.
- `src/rmd_ethercat_pdo.cpp`: PDO codec implementation.
- `include/rmd_can_sdk/rmd_ethercat_backend.h`: EtherCAT backend public class.
- `src/rmd_ethercat_backend_stub.cpp`: compile-safe disabled backend.
- `src/rmd_ethercat_backend.cpp`: IgH EtherCAT enabled backend.
- `include/rmd_can_sdk/rs232_imu_backend.h`: RS232 IMU snapshot backend interface.
- `src/rs232_imu_backend.cpp`: serial IMU backend implementation.
- `scripts/run_mixed_bus_validation.sh`: target-side hardware validation entrypoint.

Modify these existing files:

- `include/rmd_can_sdk/rmd_types.h`: add bus enum and non-breaking validity fields.
- `include/rmd_can_sdk/rmd_can_config.h`: expose normalized config helpers if needed by registry.
- `src/rmd_can_config.cpp`: parse explicit `bus`/`slave` attributes while preserving existing CAN config behavior.
- `src/rmd_driver_sdk.cpp`: shrink to public API facade and realtime core composition.
- `tests/rmd_can_sdk_tests.cpp`: add deterministic unit tests for snapshots, registry, safety, and PDO codecs.
- `CMakeLists.txt`: add new source files and optional EtherCAT build flag.

---

### Task 1: Add Whole-Frame Realtime Core

**Files:**
- Create: `include/rmd_can_sdk/rmd_realtime_core.h`
- Modify: `tests/rmd_can_sdk_tests.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write failing tests for whole-frame snapshots**

Add these includes near the top of `tests/rmd_can_sdk_tests.cpp`:

```cpp
#include "rmd_can_sdk/rmd_realtime_core.h"
```

Add this test function before `testTripleBufferSnapshotsAreWholeFrames()`:

```cpp
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
```

Call it from `main()` before `testTripleBufferSnapshotsAreWholeFrames()`:

```cpp
    testFrameBufferSnapshotsAreWholeFrames();
```

- [ ] **Step 2: Run the test and verify it fails to compile**

Run:

```bash
cmake --build build
```

Expected: compilation fails because `rmd_can_sdk/rmd_realtime_core.h` does not exist.

- [ ] **Step 3: Add the realtime core header**

Create `include/rmd_can_sdk/rmd_realtime_core.h`:

```cpp
#pragma once

#include "rmd_can_sdk/rmd_types.h"

#include <array>
#include <atomic>
#include <bitset>
#include <chrono>
#include <cstddef>
#include <cstdint>

namespace RmdCanSdk {

constexpr std::size_t MaxRealtimeMotors = 64;

using RealtimeClock = std::chrono::steady_clock;

struct FrameMetadata {
    std::uint64_t sequence = 0;
    RealtimeClock::time_point timestamp{};
    std::size_t motorCount = 0;
};

struct MotorTargetFrame : FrameMetadata {
    std::array<MotorTarget, MaxRealtimeMotors> targets{};
    std::bitset<MaxRealtimeMotors> valid{};
};

struct MotorActualFrame : FrameMetadata {
    std::array<MotorActual, MaxRealtimeMotors> actuals{};
    std::bitset<MaxRealtimeMotors> valid{};
    std::bitset<MaxRealtimeMotors> stale{};
};

template <typename Frame>
class FrameBuffer {
public:
    explicit FrameBuffer(Frame initial = Frame{}) {
        buffers_[0] = initial;
        buffers_[1] = initial;
        buffers_[2] = initial;
    }

    void publish(Frame const& frame) {
        buffers_[writing_] = frame;
        writing_ = latest_.exchange(writing_, std::memory_order_acq_rel);
        dirty_.store(true, std::memory_order_release);
    }

    Frame read() const {
        Frame out;
        readInto(out);
        return out;
    }

    void readInto(Frame& out) const {
        if (dirty_.exchange(false, std::memory_order_acq_rel)) {
            reading_ = latest_.exchange(reading_, std::memory_order_acq_rel);
        }
        out = buffers_[reading_];
    }

private:
    std::array<Frame, 3> buffers_{};
    mutable std::atomic<int> latest_{0};
    mutable std::atomic<bool> dirty_{false};
    int writing_ = 1;
    mutable int reading_ = 2;
};

} // namespace RmdCanSdk
```

- [ ] **Step 4: Run unit tests**

Run:

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Expected: `rmd_can_sdk_tests passed`.

- [ ] **Step 5: Commit**

```bash
git add include/rmd_can_sdk/rmd_realtime_core.h tests/rmd_can_sdk_tests.cpp CMakeLists.txt
git commit -m "Add whole-frame realtime buffers"
```

---

### Task 2: Add Runtime Motor Registry

**Files:**
- Create: `include/rmd_can_sdk/rmd_motor_registry.h`
- Create: `src/rmd_motor_registry.cpp`
- Modify: `include/rmd_can_sdk/rmd_types.h`
- Modify: `src/rmd_can_config.cpp`
- Modify: `tests/rmd_can_sdk_tests.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write failing registry tests**

Add include:

```cpp
#include "rmd_can_sdk/rmd_motor_registry.h"
```

Add this test after `testConfigParser()`:

```cpp
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
```

Call it from `main()` after `testConfigParser()`:

```cpp
    testMotorRegistryNormalizesCanMotors();
```

- [ ] **Step 2: Run the test and verify it fails to compile**

Run:

```bash
cmake --build build
```

Expected: compilation fails because `rmd_motor_registry.h` does not exist.

- [ ] **Step 3: Add bus fields to motor config**

Modify `include/rmd_can_sdk/rmd_types.h` near `MotorConfig`:

```cpp
enum class MotorBus {
    Can,
    Ethercat,
};

struct MotorConfig {
    int alias = 0;
    int master = 0;
    int motorId = 0;
    int slaveId = 0;
    int ethercatSlave = 0;
    MotorBus bus = MotorBus::Can;
    std::string type;
    MotorParameters parameters;
};
```

Update `src/rmd_can_config.cpp` inside CAN slave parsing:

```cpp
        motor.bus = MotorBus::Can;
        motor.ethercatSlave = 0;
```

- [ ] **Step 4: Add registry header**

Create `include/rmd_can_sdk/rmd_motor_registry.h`:

```cpp
#pragma once

#include "rmd_can_sdk/rmd_types.h"

#include <cstddef>
#include <stdexcept>
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
```

- [ ] **Step 5: Add registry implementation**

Create `src/rmd_motor_registry.cpp`:

```cpp
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
```

- [ ] **Step 6: Add source to CMake**

Modify `CMakeLists.txt` library source list:

```cmake
    src/rmd_motor_registry.cpp
```

- [ ] **Step 7: Run tests**

Run:

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Expected: all tests pass.

- [ ] **Step 8: Commit**

```bash
git add include/rmd_can_sdk/rmd_types.h include/rmd_can_sdk/rmd_motor_registry.h src/rmd_motor_registry.cpp src/rmd_can_config.cpp tests/rmd_can_sdk_tests.cpp CMakeLists.txt
git commit -m "Add runtime motor registry"
```

---

### Task 3: Add Safety Supervisor

**Files:**
- Create: `include/rmd_can_sdk/rmd_safety.h`
- Create: `src/rmd_safety.cpp`
- Modify: `tests/rmd_can_sdk_tests.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write failing safety tests**

Add include:

```cpp
#include "rmd_can_sdk/rmd_safety.h"
```

Add this test after `testMotorRegistryNormalizesCanMotors()`:

```cpp
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
```

Call it from `main()` after `testMotorRegistryNormalizesCanMotors()`:

```cpp
    testSafetySupervisorClampsTargetsAndMarksStaleActuals();
```

- [ ] **Step 2: Run the test and verify it fails to compile**

Run:

```bash
cmake --build build
```

Expected: compilation fails because `rmd_safety.h` does not exist.

- [ ] **Step 3: Add safety header**

Create `include/rmd_can_sdk/rmd_safety.h`:

```cpp
#pragma once

#include "rmd_can_sdk/rmd_protocol.h"
#include "rmd_can_sdk/rmd_types.h"

namespace RmdCanSdk {

constexpr unsigned short ErrorCodeFeedbackTimeout = 1;
constexpr unsigned short ErrorCodeBackendFault = 2;
constexpr unsigned short ErrorCodeTargetLimited = 3;

struct LimitedTarget {
    MotorTarget target;
    bool clamped = false;
};

LimitedTarget limitTarget(MotorTarget const& target, MotorParameters const& params);
void markActualStale(MotorActual& actual);
void markActualBackendFault(MotorActual& actual);

} // namespace RmdCanSdk
```

- [ ] **Step 4: Add safety implementation**

Create `src/rmd_safety.cpp`:

```cpp
#include "rmd_can_sdk/rmd_safety.h"

#include "rmd_can_sdk/common.h"

namespace RmdCanSdk {
namespace {

float clampAndTrack(float value, float min, float max, bool& clamped) {
    float const out = clamp(value, min, max);
    if (out != value) {
        clamped = true;
    }
    return out;
}

} // namespace

LimitedTarget limitTarget(MotorTarget const& target, MotorParameters const& params) {
    LimitedTarget result;
    result.target = target;
    result.target.pos = clampAndTrack(target.pos, params.minimumPosition, params.maximumPosition, result.clamped);
    result.target.tor = clampAndTrack(target.tor, -params.maximumTorque, params.maximumTorque, result.clamped);
    result.target.kp = clampAndTrack(target.kp, RmdKpMin, RmdKpMax, result.clamped);
    result.target.kd = clampAndTrack(target.kd, RmdKdMin, RmdKdMax, result.clamped);
    return result;
}

void markActualStale(MotorActual& actual) {
    actual.statusWord = 0xffff;
    actual.errorCode = ErrorCodeFeedbackTimeout;
}

void markActualBackendFault(MotorActual& actual) {
    actual.statusWord = 0xffff;
    actual.errorCode = ErrorCodeBackendFault;
}

} // namespace RmdCanSdk
```

- [ ] **Step 5: Add source to CMake and run tests**

Modify `CMakeLists.txt`:

```cmake
    src/rmd_safety.cpp
```

Run:

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Expected: all tests pass.

- [ ] **Step 6: Commit**

```bash
git add include/rmd_can_sdk/rmd_safety.h src/rmd_safety.cpp tests/rmd_can_sdk_tests.cpp CMakeLists.txt
git commit -m "Add motor safety policy"
```

---

### Task 4: Add Backend Interface

**Files:**
- Create: `include/rmd_can_sdk/rmd_motor_backend.h`
- Modify: `tests/rmd_can_sdk_tests.cpp`

- [ ] **Step 1: Write failing interface compile test**

Add include:

```cpp
#include "rmd_can_sdk/rmd_motor_backend.h"
```

Add this test after `testSafetySupervisorClampsTargetsAndMarksStaleActuals()`:

```cpp
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
```

Call it from `main()`:

```cpp
    testMotorBackendInterface();
```

- [ ] **Step 2: Run the test and verify it fails to compile**

Run:

```bash
cmake --build build
```

Expected: compilation fails because `rmd_motor_backend.h` does not exist.

- [ ] **Step 3: Add backend interface**

Create `include/rmd_can_sdk/rmd_motor_backend.h`:

```cpp
#pragma once

namespace RmdCanSdk {

struct BackendStatus {
    bool running = false;
    bool degraded = false;
    int errorCode = 0;
};

class MotorBackend {
public:
    virtual ~MotorBackend() = default;
    virtual int start() = 0;
    virtual void stop() = 0;
    virtual BackendStatus status() const = 0;
};

} // namespace RmdCanSdk
```

- [ ] **Step 4: Run tests**

Run:

```bash
cmake --build build
ctest --test-dir build --output-on-failure
```

Expected: all tests pass.

- [ ] **Step 5: Commit**

```bash
git add include/rmd_can_sdk/rmd_motor_backend.h tests/rmd_can_sdk_tests.cpp
git commit -m "Add motor backend interface"
```

---

### Task 5: Extract RMD CAN Backend

**Files:**
- Create: `include/rmd_can_sdk/rmd_can_backend.h`
- Create: `src/rmd_can_backend.cpp`
- Modify: `src/rmd_driver_sdk.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/rmd_can_sdk_tests.cpp`

- [ ] **Step 1: Add a CAN backend compile test**

Add include:

```cpp
#include "rmd_can_sdk/rmd_can_backend.h"
```

Add this test after `testMotorBackendInterface()`:

```cpp
void testRmdCanBackendConstructsWithoutOpeningSocket() {
    RmdCanSdk::Config config = RmdCanSdk::loadConfig(writeTempConfig());
    RmdCanSdk::MotorRegistry registry = RmdCanSdk::MotorRegistry::fromConfig(config);
    RmdCanSdk::FrameBuffer<RmdCanSdk::MotorTargetFrame> targets;
    RmdCanSdk::FrameBuffer<RmdCanSdk::MotorActualFrame> actuals;
    RmdCanSdk::RmdCanBackend backend(config, registry, 0, targets, actuals);
    require(!backend.status().running, "CAN backend is stopped before start");
}
```

Call it from `main()`:

```cpp
    testRmdCanBackendConstructsWithoutOpeningSocket();
```

- [ ] **Step 2: Run the test and verify it fails to compile**

Run:

```bash
cmake --build build
```

Expected: compilation fails because `rmd_can_backend.h` does not exist.

- [ ] **Step 3: Add CAN backend header**

Create `include/rmd_can_sdk/rmd_can_backend.h`:

```cpp
#pragma once

#include "rmd_can_sdk/rmd_can_config.h"
#include "rmd_can_sdk/rmd_can_transport.h"
#include "rmd_can_sdk/rmd_motor_backend.h"
#include "rmd_can_sdk/rmd_motor_registry.h"
#include "rmd_can_sdk/rmd_realtime_core.h"

#include <atomic>
#include <thread>
#include <vector>

namespace RmdCanSdk {

class RmdCanBackend final : public MotorBackend {
public:
    RmdCanBackend(Config const& config,
                  MotorRegistry const& registry,
                  std::size_t backendIndex,
                  FrameBuffer<MotorTargetFrame>& targetBuffer,
                  FrameBuffer<MotorActualFrame>& actualBuffer);
    ~RmdCanBackend() override;

    int start() override;
    void stop() override;
    BackendStatus status() const override;

private:
    void txLoop();
    void rxLoop();
    void publishActuals();
    void markMissingStale();

    Config const& config_;
    MotorRegistry const& registry_;
    std::size_t backendIndex_ = 0;
    FrameBuffer<MotorTargetFrame>& targetBuffer_;
    FrameBuffer<MotorActualFrame>& actualBuffer_;
    BackendStatus status_;
    SocketCanTransport transport_;
    FeedbackFrameTracker mitFrame_;
    std::vector<std::size_t> motorIndexes_;
    MotorActualFrame workingActuals_;
    std::atomic<bool> stop_{true};
    std::thread txThread_;
    std::thread rxThread_;
};

} // namespace RmdCanSdk
```

- [ ] **Step 4: Move CAN runtime into implementation**

Create `src/rmd_can_backend.cpp` by moving the CAN-specific runtime logic from `src/rmd_driver_sdk.cpp`:

```cpp
#include "rmd_can_sdk/rmd_can_backend.h"

#include "rmd_can_sdk/rmd_protocol.h"
#include "rmd_can_sdk/rmd_safety.h"

#include <algorithm>
#include <chrono>

namespace RmdCanSdk {

RmdCanBackend::RmdCanBackend(Config const& config,
                             MotorRegistry const& registry,
                             std::size_t backendIndex,
                             FrameBuffer<MotorTargetFrame>& targetBuffer,
                             FrameBuffer<MotorActualFrame>& actualBuffer)
    : config_(config),
      registry_(registry),
      backendIndex_(backendIndex),
      targetBuffer_(targetBuffer),
      actualBuffer_(actualBuffer) {
    auto const& group = registry_.backends().at(backendIndex_);
    motorIndexes_ = group.motorIndexes;
    workingActuals_.motorCount = static_cast<std::size_t>(config_.totalMotorCount);

    std::vector<int> slaveIds;
    for (std::size_t registryIndex : motorIndexes_) {
        auto const& motor = registry_.motors().at(registryIndex);
        slaveIds.push_back(motor.slaveId);
        workingActuals_.valid.set(motor.globalIndex);
    }
    mitFrame_.setExpectedFromSlaveIds(slaveIds);
}

RmdCanBackend::~RmdCanBackend() {
    stop();
}

int RmdCanBackend::start() {
    if (status_.running) {
        return 0;
    }
    auto const& group = registry_.backends().at(backendIndex_);
    auto masterIt = std::find_if(config_.masters.begin(), config_.masters.end(), [&](MasterConfig const& master) {
        return master.order == group.master;
    });
    if (masterIt == config_.masters.end()) {
        status_.degraded = true;
        status_.errorCode = ErrorCodeBackendFault;
        return -1;
    }
    if (transport_.open(masterIt->device) != 0) {
        status_.degraded = true;
        status_.errorCode = ErrorCodeBackendFault;
        return -1;
    }
    stop_.store(false, std::memory_order_release);
    status_.running = true;
    txThread_ = std::thread(&RmdCanBackend::txLoop, this);
    rxThread_ = std::thread(&RmdCanBackend::rxLoop, this);
    return 0;
}

void RmdCanBackend::stop() {
    stop_.store(true, std::memory_order_release);
    if (txThread_.joinable()) {
        txThread_.join();
    }
    if (rxThread_.joinable()) {
        rxThread_.join();
    }
    transport_.close();
    status_.running = false;
}

BackendStatus RmdCanBackend::status() const {
    return status_;
}

void RmdCanBackend::publishActuals() {
    workingActuals_.sequence++;
    workingActuals_.timestamp = RealtimeClock::now();
    actualBuffer_.publish(workingActuals_);
}

void RmdCanBackend::markMissingStale() {
    for (std::size_t registryIndex : motorIndexes_) {
        auto const& motor = registry_.motors().at(registryIndex);
        if (mitFrame_.isReceived(motor.slaveId)) {
            continue;
        }
        markActualStale(workingActuals_.actuals[motor.globalIndex]);
        workingActuals_.stale.set(motor.globalIndex);
    }
}

void RmdCanBackend::txLoop() {
    auto const& group = registry_.backends().at(backendIndex_);
    auto masterIt = std::find_if(config_.masters.begin(), config_.masters.end(), [&](MasterConfig const& master) {
        return master.order == group.master;
    });
    std::chrono::nanoseconds const period(config_.periodNs * std::max(1, masterIt->division));
    MotorTargetFrame targets;
    std::uint64_t tick = 0;
    while (!stop_.load(std::memory_order_acquire)) {
        auto const started = std::chrono::steady_clock::now();
        targetBuffer_.readInto(targets);
        for (std::size_t registryIndex : motorIndexes_) {
            auto const& motor = registry_.motors().at(registryIndex);
            MotorTarget target = targets.targets[motor.globalIndex];
            if (target.enabled != 1) {
                target = MotorTarget{};
            }
            auto limited = limitTarget(target, motor.parameters);
            auto data = packMit(limited.target.pos, limited.target.vel, limited.target.kp, limited.target.kd,
                                limited.target.tor, motor.parameters.maximumTorque);
            if (transport_.send(mitTxId(motor.motorId), data.data(), static_cast<int>(data.size())) < 0) {
                status_.degraded = true;
                status_.errorCode = ErrorCodeBackendFault;
            }
        }
        if (tick % 20 == 0) {
            unsigned char status2[8] = {0x9C, 0, 0, 0, 0, 0, 0, 0};
            for (std::size_t registryIndex : motorIndexes_) {
                auto const& motor = registry_.motors().at(registryIndex);
                transport_.send(standardTxId(motor.motorId), status2, 8);
            }
        }
        tick++;
        std::this_thread::sleep_until(started + period);
    }
}

void RmdCanBackend::rxLoop() {
    while (!stop_.load(std::memory_order_acquire)) {
        CanFrame frame;
        int const received = transport_.receive(frame, 50);
        if (received <= 0) {
            markMissingStale();
            publishActuals();
            mitFrame_.reset();
            continue;
        }

        for (std::size_t registryIndex : motorIndexes_) {
            auto const& motor = registry_.motors().at(registryIndex);
            std::vector<unsigned char> data(frame.data.begin(), frame.data.begin() + frame.length);
            if (frame.id == mitRxId(motor.motorId)) {
                auto feedback = parseMitReply(data, motor.motorId, motor.parameters.maximumTorque);
                if (feedback.valid) {
                    applyMitFeedback(workingActuals_.actuals[motor.globalIndex], feedback);
                    workingActuals_.valid.set(motor.globalIndex);
                    workingActuals_.stale.reset(motor.globalIndex);
                    if (mitFrame_.markReceived(motor.slaveId, FeedbackFrameTracker::Clock::now())) {
                        publishActuals();
                    }
                }
            } else if (frame.id == standardRxId(motor.motorId)) {
                auto feedback = parseStatus2Reply(data, motor.parameters.torqueConstant);
                if (feedback.valid) {
                    applyStatus2Feedback(workingActuals_.actuals[motor.globalIndex], feedback);
                }
            }
        }
    }
}

} // namespace RmdCanSdk
```

- [ ] **Step 5: Add source to CMake**

Modify `CMakeLists.txt`:

```cmake
    src/rmd_can_backend.cpp
```

- [ ] **Step 6: Run tests**

Run:

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Expected: all tests pass.

- [ ] **Step 7: Commit**

```bash
git add include/rmd_can_sdk/rmd_can_backend.h src/rmd_can_backend.cpp tests/rmd_can_sdk_tests.cpp CMakeLists.txt
git commit -m "Extract RMD CAN backend"
```

---

### Task 6: Convert DriverSDK Facade To Realtime Core

**Files:**
- Modify: `src/rmd_driver_sdk.cpp`
- Modify: `tests/rmd_can_sdk_tests.cpp`

- [ ] **Step 1: Add a no-hardware facade behavior test**

Add this test after `testRmdCanBackendConstructsWithoutOpeningSocket()`:

```cpp
void testDriverSdkReportsUninitializedBeforeInit() {
    DriverSDK::DriverSDK& sdk = DriverSDK::DriverSDK::instance();
    std::vector<DriverSDK::motorTargetStruct> targets(1);
    std::vector<DriverSDK::motorActualStruct> actuals(1);
    require(sdk.setMotorTarget(targets) == std::numeric_limits<int>::min(),
            "setMotorTarget rejects calls before successful init");
    require(sdk.getMotorActual(actuals) == std::numeric_limits<int>::min(),
            "getMotorActual rejects calls before successful init");
}
```

Call it from `main()` before tests that use initialized configs:

```cpp
    testDriverSdkReportsUninitializedBeforeInit();
```

- [ ] **Step 2: Run the test**

Run:

```bash
cmake --build build
ctest --test-dir build --output-on-failure
```

Expected: this test may fail if current singleton state accepts calls without init. Keep the failing behavior visible before changing the facade.

- [ ] **Step 3: Replace `Impl` internals with core-owned buffers and backends**

In `src/rmd_driver_sdk.cpp`, keep public conversion helpers `toPublicActual()` and `toInternalTarget()`. Replace the current `Impl` data members with:

```cpp
    RmdCanSdk::Config config_;
    RmdCanSdk::MotorRegistry registry_;
    RmdCanSdk::FrameBuffer<RmdCanSdk::MotorTargetFrame> targetBuffer_;
    RmdCanSdk::FrameBuffer<RmdCanSdk::MotorActualFrame> actualBuffer_;
    std::vector<std::unique_ptr<RmdCanSdk::MotorBackend>> backends_;
    bool initialized_ = false;
```

Add includes:

```cpp
#include "rmd_can_sdk/rmd_can_backend.h"
#include "rmd_can_sdk/rmd_motor_backend.h"
#include "rmd_can_sdk/rmd_motor_registry.h"
#include "rmd_can_sdk/rmd_realtime_core.h"
#include "rmd_can_sdk/rmd_safety.h"
```

- [ ] **Step 4: Implement `Impl::init()` using registry and backend groups**

Use this structure:

```cpp
    void init(char const* xmlFile) {
        stop();
        config_ = RmdCanSdk::loadConfig(xmlFile);
        registry_ = RmdCanSdk::MotorRegistry::fromConfig(config_);

        RmdCanSdk::MotorTargetFrame initialTargets;
        RmdCanSdk::MotorActualFrame initialActuals;
        initialTargets.motorCount = static_cast<std::size_t>(config_.totalMotorCount);
        initialActuals.motorCount = static_cast<std::size_t>(config_.totalMotorCount);
        for (auto const& motor : registry_.motors()) {
            initialActuals.valid.set(motor.globalIndex);
            initialActuals.actuals[motor.globalIndex].statusWord = 0x0000;
        }
        targetBuffer_.publish(initialTargets);
        actualBuffer_.publish(initialActuals);

        for (std::size_t i = 0; i < registry_.backends().size(); ++i) {
            auto const& group = registry_.backends()[i];
            if (group.bus == RmdCanSdk::MotorBus::Can) {
                backends_.push_back(std::make_unique<RmdCanSdk::RmdCanBackend>(
                    config_, registry_, i, targetBuffer_, actualBuffer_));
            }
        }

        for (auto& backend : backends_) {
            if (backend->start() != 0) {
                stop();
                throw std::runtime_error("starting motor backend failed");
            }
        }
        initialized_ = true;
    }
```

- [ ] **Step 5: Implement `setTargets()` through target frame publish**

Use this structure:

```cpp
    int setTargets(std::vector<motorTargetStruct> const& data) {
        if (!initialized_ || static_cast<int>(data.size()) != config_.totalMotorCount) {
            return std::numeric_limits<int>::min();
        }
        RmdCanSdk::MotorTargetFrame frame;
        frame.sequence++;
        frame.timestamp = RmdCanSdk::RealtimeClock::now();
        frame.motorCount = static_cast<std::size_t>(config_.totalMotorCount);
        for (auto const& motor : registry_.motors()) {
            std::size_t const index = motor.globalIndex;
            auto converted = toInternalTarget(data[index], motor.parameters);
            auto limited = RmdCanSdk::limitTarget(converted, motor.parameters);
            frame.targets[index] = limited.target;
            frame.valid.set(index);
        }
        targetBuffer_.publish(frame);
        return 0;
    }
```

- [ ] **Step 6: Implement `getActuals()` through actual frame read**

Use this structure:

```cpp
    int getActuals(std::vector<motorActualStruct>& data) {
        if (!initialized_ || static_cast<int>(data.size()) != config_.totalMotorCount) {
            return std::numeric_limits<int>::min();
        }
        RmdCanSdk::MotorActualFrame frame;
        actualBuffer_.readInto(frame);
        for (auto& item : data) {
            item = motorActualStruct{};
        }
        for (auto const& motor : registry_.motors()) {
            std::size_t const index = motor.globalIndex;
            data[index] = toPublicActual(frame.actuals[index], motor.parameters);
        }
        return 0;
    }
```

- [ ] **Step 7: Implement `stop()`**

Use this structure:

```cpp
    void stop() {
        for (auto& backend : backends_) {
            if (backend != nullptr) {
                backend->stop();
            }
        }
        backends_.clear();
        initialized_ = false;
    }
```

- [ ] **Step 8: Run tests**

Run:

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Expected: all tests pass. Hardware-opening tests must still only construct backends, not call `init()` with real SocketCAN unless the interface exists.

- [ ] **Step 9: Commit**

```bash
git add src/rmd_driver_sdk.cpp tests/rmd_can_sdk_tests.cpp
git commit -m "Route DriverSDK through realtime frames"
```

---

### Task 7: Add EtherCAT Build Gate And PDO Codec

**Files:**
- Create: `include/rmd_can_sdk/rmd_ethercat_pdo.h`
- Create: `src/rmd_ethercat_pdo.cpp`
- Create: `include/rmd_can_sdk/rmd_ethercat_backend.h`
- Create: `src/rmd_ethercat_backend_stub.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/rmd_can_sdk_tests.cpp`

- [ ] **Step 1: Write failing PDO codec test**

Add include:

```cpp
#include "rmd_can_sdk/rmd_ethercat_pdo.h"
```

Add this test after `testRmdCanBackendConstructsWithoutOpeningSocket()`:

```cpp
void testEthercatPdoCodecParsesPackedTxData() {
    RmdCanSdk::EthercatTxPdoBytes bytes;
    bytes.statusWord = 0x0237;
    bytes.actualPosition = 1000;
    bytes.actualVelocity = -200;
    bytes.actualTorque = 123;
    bytes.errorCode = 0;
    bytes.motorTemperature = 35;
    bytes.driveTemperature = 40;
    bytes.voltage = 680;

    RmdCanSdk::MotorParameters params;
    params.polarity = 1.0f;
    params.countBias = 0.0f;
    params.encoderResolution = 1000.0f;
    params.gearRatioPosVel = 1.0f;
    params.gearRatioTor = 1.0f;
    params.torqueConstant = 1.0f;

    auto actual = RmdCanSdk::parseEthercatTxPdo(bytes, params);
    require(actual.statusWord == 0x0237, "EtherCAT TxPDO statusWord parses");
    require(actual.errorCode == 0, "EtherCAT TxPDO errorCode parses");
    require(actual.temp == 35, "EtherCAT TxPDO motor temperature parses");
    require(actual.driveTemp == 40, "EtherCAT TxPDO drive temperature parses");
    require(actual.voltage == 680, "EtherCAT TxPDO voltage parses");
}
```

Call it from `main()`:

```cpp
    testEthercatPdoCodecParsesPackedTxData();
```

- [ ] **Step 2: Run the test and verify it fails to compile**

Run:

```bash
cmake --build build
```

Expected: compilation fails because `rmd_ethercat_pdo.h` does not exist.

- [ ] **Step 3: Add PDO codec header**

Create `include/rmd_can_sdk/rmd_ethercat_pdo.h`:

```cpp
#pragma once

#include "rmd_can_sdk/rmd_types.h"

#include <cstdint>

namespace RmdCanSdk {

struct EthercatRxPdoBytes {
    std::uint16_t controlWord = 0;
    std::int32_t targetPosition = 0;
    std::int32_t targetVelocity = 0;
    std::int16_t targetTorque = 0;
    std::int32_t pvtKp = 0;
    std::int32_t pvtKd = 0;
    std::int8_t mode = 0;
    std::int8_t enabled = 0;
};

struct EthercatTxPdoBytes {
    std::uint16_t statusWord = 0;
    std::int32_t actualPosition = 0;
    std::int32_t actualVelocity = 0;
    std::int16_t actualTorque = 0;
    std::uint16_t errorCode = 0;
    std::int16_t motorTemperature = 0;
    std::int16_t driveTemperature = 0;
    std::uint16_t voltage = 0;
    std::int8_t modeDisplay = 0;
    std::int8_t reserved = 0;
};

EthercatRxPdoBytes packEthercatRxPdo(MotorTarget const& target, MotorParameters const& params);
MotorActual parseEthercatTxPdo(EthercatTxPdoBytes const& tx, MotorParameters const& params);

} // namespace RmdCanSdk
```

- [ ] **Step 4: Add PDO codec implementation**

Create `src/rmd_ethercat_pdo.cpp`:

```cpp
#include "rmd_can_sdk/rmd_ethercat_pdo.h"

#include "rmd_can_sdk/common.h"

#include <cmath>

namespace RmdCanSdk {
namespace {

std::int32_t radToCounts(float rad, MotorParameters const& params) {
    float const revolutions = rad / (2.0f * Pi);
    return static_cast<std::int32_t>(std::lround(revolutions * params.encoderResolution * params.gearRatioPosVel));
}

float countsToRad(std::int32_t counts, MotorParameters const& params) {
    float const denominator = params.encoderResolution * params.gearRatioPosVel;
    if (denominator == 0.0f) {
        return 0.0f;
    }
    return static_cast<float>(counts) / denominator * 2.0f * Pi;
}

std::int16_t torqueToRaw(float torque, MotorParameters const& params) {
    float const scale = params.torqueConstant * params.gearRatioTor;
    if (scale == 0.0f) {
        return 0;
    }
    return static_cast<std::int16_t>(std::lround(torque / scale));
}

float rawToTorque(std::int16_t raw, MotorParameters const& params) {
    return static_cast<float>(raw) * params.torqueConstant * params.gearRatioTor;
}

} // namespace

EthercatRxPdoBytes packEthercatRxPdo(MotorTarget const& target, MotorParameters const& params) {
    EthercatRxPdoBytes rx;
    rx.controlWord = target.enabled == 1 ? 0x000f : 0x0006;
    rx.targetPosition = radToCounts(target.pos, params);
    rx.targetVelocity = radToCounts(target.vel, params);
    rx.targetTorque = torqueToRaw(target.tor, params);
    rx.pvtKp = static_cast<std::int32_t>(std::lround(target.kp));
    rx.pvtKd = static_cast<std::int32_t>(std::lround(target.kd));
    rx.mode = 8;
    rx.enabled = static_cast<std::int8_t>(target.enabled);
    return rx;
}

MotorActual parseEthercatTxPdo(EthercatTxPdoBytes const& tx, MotorParameters const& params) {
    MotorActual actual;
    actual.pos = countsToRad(tx.actualPosition, params);
    actual.vel = countsToRad(tx.actualVelocity, params);
    actual.tor = rawToTorque(tx.actualTorque, params);
    actual.temp = tx.motorTemperature;
    actual.driveTemp = tx.driveTemperature;
    actual.voltage = tx.voltage;
    actual.statusWord = tx.statusWord;
    actual.errorCode = tx.errorCode;
    return actual;
}

} // namespace RmdCanSdk
```

- [ ] **Step 5: Add EtherCAT backend header and disabled stub**

Create `include/rmd_can_sdk/rmd_ethercat_backend.h`:

```cpp
#pragma once

#include "rmd_can_sdk/rmd_motor_backend.h"
#include "rmd_can_sdk/rmd_motor_registry.h"
#include "rmd_can_sdk/rmd_realtime_core.h"

namespace RmdCanSdk {

class RmdEthercatBackend final : public MotorBackend {
public:
    RmdEthercatBackend(Config const& config,
                       MotorRegistry const& registry,
                       std::size_t backendIndex,
                       FrameBuffer<MotorTargetFrame>& targetBuffer,
                       FrameBuffer<MotorActualFrame>& actualBuffer);
    ~RmdEthercatBackend() override;

    int start() override;
    void stop() override;
    BackendStatus status() const override;

private:
    BackendStatus status_;
};

} // namespace RmdCanSdk
```

Create `src/rmd_ethercat_backend_stub.cpp`:

```cpp
#include "rmd_can_sdk/rmd_ethercat_backend.h"

#include "rmd_can_sdk/rmd_safety.h"

namespace RmdCanSdk {

RmdEthercatBackend::RmdEthercatBackend(Config const&,
                                       MotorRegistry const&,
                                       std::size_t,
                                       FrameBuffer<MotorTargetFrame>&,
                                       FrameBuffer<MotorActualFrame>&) {}

RmdEthercatBackend::~RmdEthercatBackend() = default;

int RmdEthercatBackend::start() {
    status_.running = false;
    status_.degraded = true;
    status_.errorCode = ErrorCodeBackendFault;
    return -1;
}

void RmdEthercatBackend::stop() {
    status_.running = false;
}

BackendStatus RmdEthercatBackend::status() const {
    return status_;
}

} // namespace RmdCanSdk
```

- [ ] **Step 6: Add optional EtherCAT CMake gate**

Modify `CMakeLists.txt`:

```cmake
option(RMD_CAN_SDK_ENABLE_ECAT "Build IgH EtherCAT backend" OFF)

set(RMD_CAN_SDK_SOURCES
    src/common.cpp
    src/rmd_can_config.cpp
    src/rmd_can_transport.cpp
    src/rmd_driver_sdk.cpp
    src/rmd_motion_plan.cpp
    src/rmd_protocol.cpp
    src/rmd_types.cpp
    src/rmd_motor_registry.cpp
    src/rmd_safety.cpp
    src/rmd_can_backend.cpp
    src/rmd_ethercat_pdo.cpp
    third_party/tinyxml2.cpp
)

if(RMD_CAN_SDK_ENABLE_ECAT)
    list(APPEND RMD_CAN_SDK_SOURCES src/rmd_ethercat_backend.cpp)
else()
    list(APPEND RMD_CAN_SDK_SOURCES src/rmd_ethercat_backend_stub.cpp)
endif()

add_library(rmd_can_sdk STATIC ${RMD_CAN_SDK_SOURCES})
```

If `add_library(rmd_can_sdk STATIC` already exists with an inline source list, replace that source list with the `RMD_CAN_SDK_SOURCES` variable created in this step.

- [ ] **Step 7: Run tests**

Run:

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Expected: all tests pass with EtherCAT disabled.

- [ ] **Step 8: Commit**

```bash
git add include/rmd_can_sdk/rmd_ethercat_pdo.h src/rmd_ethercat_pdo.cpp include/rmd_can_sdk/rmd_ethercat_backend.h src/rmd_ethercat_backend_stub.cpp tests/rmd_can_sdk_tests.cpp CMakeLists.txt
git commit -m "Add EtherCAT build gate and PDO codec"
```

---

### Task 8: Implement EtherCAT Backend On Target

**Files:**
- Create: `src/rmd_ethercat_backend.cpp`
- Modify: `src/rmd_driver_sdk.cpp`
- Modify: `CMakeLists.txt`
- Test manually on EtherCAT target

- [ ] **Step 1: Add enabled backend source**

Create `src/rmd_ethercat_backend.cpp` with this structure:

```cpp
#include "rmd_can_sdk/rmd_ethercat_backend.h"

#include "rmd_can_sdk/rmd_ethercat_pdo.h"
#include "rmd_can_sdk/rmd_safety.h"

#include <ecrt.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

namespace RmdCanSdk {

class RmdEthercatRuntime {
public:
    Config const& config;
    MotorRegistry const& registry;
    std::size_t backendIndex;
    FrameBuffer<MotorTargetFrame>& targetBuffer;
    FrameBuffer<MotorActualFrame>& actualBuffer;
    BackendStatus status;
    std::atomic<bool> stop{true};
    std::thread thread;
    MotorActualFrame workingActuals;
    MotorTargetFrame targets;
    ec_master_t* master = nullptr;
    ec_domain_t* domain = nullptr;
    std::uint8_t* domainData = nullptr;
    std::vector<std::size_t> motorIndexes;

    RmdEthercatRuntime(Config const& c,
                       MotorRegistry const& r,
                       std::size_t b,
                       FrameBuffer<MotorTargetFrame>& tb,
                       FrameBuffer<MotorActualFrame>& ab)
        : config(c), registry(r), backendIndex(b), targetBuffer(tb), actualBuffer(ab) {}
};

namespace {

void realtimeLoop(RmdEthercatRuntime* rt) {
    while (!rt->stop.load(std::memory_order_acquire)) {
        ecrt_master_receive(rt->master);
        ecrt_domain_process(rt->domain);

        rt->workingActuals.sequence++;
        rt->workingActuals.timestamp = RealtimeClock::now();
        rt->workingActuals.motorCount = static_cast<std::size_t>(rt->config.totalMotorCount);

        for (std::size_t registryIndex : rt->motorIndexes) {
            auto const& motor = rt->registry.motors().at(registryIndex);
            std::size_t const txOffset = motor.backendLocalIndex * sizeof(EthercatTxPdoBytes);
            auto const* tx = reinterpret_cast<EthercatTxPdoBytes const*>(rt->domainData + txOffset);
            rt->workingActuals.actuals[motor.globalIndex] = parseEthercatTxPdo(*tx, motor.parameters);
            rt->workingActuals.valid.set(motor.globalIndex);
            rt->workingActuals.stale.reset(motor.globalIndex);
        }

        rt->targetBuffer.readInto(rt->targets);

        for (std::size_t registryIndex : rt->motorIndexes) {
            auto const& motor = rt->registry.motors().at(registryIndex);
            std::size_t const rxBase = rt->motorIndexes.size() * sizeof(EthercatTxPdoBytes);
            std::size_t const rxOffset = rxBase + motor.backendLocalIndex * sizeof(EthercatRxPdoBytes);
            auto* rx = reinterpret_cast<EthercatRxPdoBytes*>(rt->domainData + rxOffset);
            MotorTarget target = rt->targets.targets[motor.globalIndex];
            if (target.enabled != 1) {
                target = MotorTarget{};
            }
            *rx = packEthercatRxPdo(limitTarget(target, motor.parameters).target, motor.parameters);
        }

        rt->actualBuffer.publish(rt->workingActuals);
        ecrt_domain_queue(rt->domain);
        ecrt_master_send(rt->master);
    }
}

} // namespace

RmdEthercatBackend::RmdEthercatBackend(Config const& config,
                                       MotorRegistry const& registry,
                                       std::size_t backendIndex,
                                       FrameBuffer<MotorTargetFrame>& targetBuffer,
                                       FrameBuffer<MotorActualFrame>& actualBuffer)
    : runtime_(new RmdEthercatRuntime(config, registry, backendIndex, targetBuffer, actualBuffer)) {}

RmdEthercatBackend::~RmdEthercatBackend() {
    stop();
    delete runtime_;
}

int RmdEthercatBackend::start() {
    runtime_->motorIndexes = runtime_->registry.backends().at(runtime_->backendIndex).motorIndexes;
    runtime_->master = ecrt_request_master(runtime_->registry.backends().at(runtime_->backendIndex).master);
    if (runtime_->master == nullptr) {
        runtime_->status.degraded = true;
        runtime_->status.errorCode = ErrorCodeBackendFault;
        return -1;
    }
    runtime_->domain = ecrt_master_create_domain(runtime_->master);
    if (runtime_->domain == nullptr) {
        runtime_->status.degraded = true;
        runtime_->status.errorCode = ErrorCodeBackendFault;
        return -1;
    }
    if (ecrt_master_activate(runtime_->master) != 0) {
        runtime_->status.degraded = true;
        runtime_->status.errorCode = ErrorCodeBackendFault;
        return -1;
    }
    runtime_->domainData = ecrt_domain_data(runtime_->domain);
    if (runtime_->domainData == nullptr) {
        runtime_->status.degraded = true;
        runtime_->status.errorCode = ErrorCodeBackendFault;
        return -1;
    }
    runtime_->stop.store(false, std::memory_order_release);
    runtime_->status.running = true;
    runtime_->thread = std::thread(realtimeLoop, runtime_);
    return 0;
}

void RmdEthercatBackend::stop() {
    if (runtime_ == nullptr) {
        return;
    }
    runtime_->stop.store(true, std::memory_order_release);
    if (runtime_->thread.joinable()) {
        runtime_->thread.join();
    }
    runtime_->status.running = false;
}

BackendStatus RmdEthercatBackend::status() const {
    return runtime_ == nullptr ? BackendStatus{} : runtime_->status;
}

} // namespace RmdCanSdk
```

Also update `include/rmd_can_sdk/rmd_ethercat_backend.h` private section:

```cpp
    class RmdEthercatRuntime* runtime_ = nullptr;
```

- [ ] **Step 2: Link EtherCAT when enabled**

Modify `CMakeLists.txt`:

```cmake
if(RMD_CAN_SDK_ENABLE_ECAT)
    target_compile_definitions(rmd_can_sdk PUBLIC RMD_CAN_SDK_ENABLE_ECAT=1)
    target_link_libraries(rmd_can_sdk PUBLIC ethercat)
endif()
```

- [ ] **Step 3: Add EtherCAT backend construction to DriverSDK**

In `src/rmd_driver_sdk.cpp`, include:

```cpp
#include "rmd_can_sdk/rmd_ethercat_backend.h"
```

In backend group creation, add:

```cpp
            } else if (group.bus == RmdCanSdk::MotorBus::Ethercat) {
                backends_.push_back(std::make_unique<RmdCanSdk::RmdEthercatBackend>(
                    config_, registry_, i, targetBuffer_, actualBuffer_));
```

- [ ] **Step 4: Build on EtherCAT target**

Run on the EtherCAT target:

```bash
cmake -S . -B build-ecat -DRMD_CAN_SDK_ENABLE_ECAT=ON
cmake --build build-ecat
```

Expected: build succeeds and links `rmd_can_sdk` against `libethercat`.

- [ ] **Step 5: Run non-hardware tests on target**

Run:

```bash
ctest --test-dir build-ecat --output-on-failure
```

Expected: all unit tests pass.

- [ ] **Step 6: Commit**

```bash
git add include/rmd_can_sdk/rmd_ethercat_backend.h src/rmd_ethercat_backend.cpp src/rmd_driver_sdk.cpp CMakeLists.txt
git commit -m "Add EtherCAT motor backend"
```

---

### Task 9: Add RS232 IMU Snapshot Backend

**Files:**
- Create: `include/rmd_can_sdk/rs232_imu_backend.h`
- Create: `src/rs232_imu_backend.cpp`
- Modify: `src/rmd_driver_sdk.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/rmd_can_sdk_tests.cpp`

- [ ] **Step 1: Write IMU snapshot test**

Add include:

```cpp
#include "rmd_can_sdk/rs232_imu_backend.h"
```

Add this test:

```cpp
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
```

Call it from `main()`:

```cpp
    testImuSnapshotBufferReturnsWholeSample();
```

- [ ] **Step 2: Add IMU backend header**

Create `include/rmd_can_sdk/rs232_imu_backend.h`:

```cpp
#pragma once

#include "rmd_can_sdk/heima_driver_sdk.h"
#include "rmd_can_sdk/rmd_realtime_core.h"

namespace RmdCanSdk {

using ImuSnapshotBuffer = FrameBuffer<DriverSDK::imuStruct>;

class Rs232ImuBackend {
public:
    explicit Rs232ImuBackend(ImuSnapshotBuffer& buffer);
    ~Rs232ImuBackend();

    int start(char const* device, int baudrate);
    void stop();
    bool running() const { return running_; }

private:
    ImuSnapshotBuffer& buffer_;
    bool running_ = false;
};

} // namespace RmdCanSdk
```

- [ ] **Step 3: Add non-blocking backend implementation shell**

Create `src/rs232_imu_backend.cpp`:

```cpp
#include "rmd_can_sdk/rs232_imu_backend.h"

namespace RmdCanSdk {

Rs232ImuBackend::Rs232ImuBackend(ImuSnapshotBuffer& buffer) : buffer_(buffer) {}

Rs232ImuBackend::~Rs232ImuBackend() {
    stop();
}

int Rs232ImuBackend::start(char const*, int) {
    running_ = true;
    DriverSDK::imuStruct zero;
    buffer_.publish(zero);
    return 0;
}

void Rs232ImuBackend::stop() {
    running_ = false;
}

} // namespace RmdCanSdk
```

This task creates the stable snapshot boundary used by `DriverSDK::getIMU()`. Task 10 connects the serial parser to this boundary before mixed hardware validation.

- [ ] **Step 4: Wire DriverSDK `getIMU()` through the buffer**

In `src/rmd_driver_sdk.cpp`, add an IMU buffer member:

```cpp
    RmdCanSdk::ImuSnapshotBuffer imuBuffer_;
```

Update `DriverSDK::getIMU()`:

```cpp
void DriverSDK::getIMU(imuStruct& data) {
    impl_->getImu(data);
}
```

Add to `Impl`:

```cpp
    void getImu(imuStruct& data) {
        imuBuffer_.readInto(data);
    }
```

- [ ] **Step 5: Add source and run tests**

Modify `CMakeLists.txt`:

```cmake
    src/rs232_imu_backend.cpp
```

Run:

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Expected: all tests pass.

- [ ] **Step 6: Commit**

```bash
git add include/rmd_can_sdk/rs232_imu_backend.h src/rs232_imu_backend.cpp src/rmd_driver_sdk.cpp tests/rmd_can_sdk_tests.cpp CMakeLists.txt
git commit -m "Add IMU snapshot backend boundary"
```

---

### Task 10: Connect RS232 Parser To IMU Snapshot Backend

**Files:**
- Modify: `include/rmd_can_sdk/rs232_imu_backend.h`
- Modify: `src/rs232_imu_backend.cpp`
- Modify: `CMakeLists.txt`
- Test manually with the working IMU device

- [ ] **Step 1: Add parser runtime members**

Modify `include/rmd_can_sdk/rs232_imu_backend.h`:

```cpp
#include <atomic>
#include <string>
#include <thread>

namespace RmdCanSdk {

class Rs232ImuBackend {
public:
    explicit Rs232ImuBackend(ImuSnapshotBuffer& buffer);
    ~Rs232ImuBackend();

    int start(char const* device, int baudrate, char const* type);
    void stop();
    bool running() const { return running_.load(std::memory_order_acquire); }

private:
    void readLoop();

    ImuSnapshotBuffer& buffer_;
    std::string device_;
    std::string type_;
    int baudrate_ = 0;
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_{true};
    std::thread thread_;
};

} // namespace RmdCanSdk
```

- [ ] **Step 2: Add POSIX serial helpers**

Modify `src/rs232_imu_backend.cpp` by adding these includes:

```cpp
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
```

Add these helpers in an anonymous namespace:

```cpp
namespace {

speed_t baudToSpeed(int baudrate) {
    switch (baudrate) {
    case 921600: return B921600;
    case 576000: return B576000;
    case 460800: return B460800;
    case 230400: return B230400;
    case 115200: return B115200;
    case 57600: return B57600;
    case 38400: return B38400;
    case 19200: return B19200;
    case 9600: return B9600;
    case 4800: return B4800;
    case 2400: return B2400;
    case 1200: return B1200;
    case 300: return B300;
    default: return 0;
    }
}

int openSerial(char const* device, int baudrate) {
    speed_t const speed = baudToSpeed(baudrate);
    if (speed == 0) {
        return -1;
    }
    int fd = open(device, O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        return -1;
    }
    termios opt{};
    if (tcgetattr(fd, &opt) != 0) {
        close(fd);
        return -1;
    }
    cfsetispeed(&opt, speed);
    cfsetospeed(&opt, speed);
    opt.c_cflag &= ~CSIZE;
    opt.c_cflag |= CS8;
    opt.c_cflag &= ~PARENB;
    opt.c_cflag &= ~CSTOPB;
    opt.c_cflag &= ~CRTSCTS;
    opt.c_cflag |= (CLOCAL | CREAD);
    opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    opt.c_iflag &= ~INPCK;
    opt.c_iflag &= ~(ICRNL | INLCR);
    opt.c_iflag &= ~(IXON | IXOFF | IXANY);
    opt.c_oflag &= ~OPOST;
    opt.c_oflag &= ~(OCRNL | ONLCR);
    opt.c_cc[VTIME] = 1;
    opt.c_cc[VMIN] = 0;
    if (tcsetattr(fd, TCSANOW, &opt) != 0) {
        close(fd);
        return -1;
    }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

} // namespace
```

- [ ] **Step 3: Add decoded-sample publish loop**

Replace `start()` and `stop()` in `src/rs232_imu_backend.cpp`:

```cpp
int Rs232ImuBackend::start(char const* device, int baudrate, char const* type) {
    if (device == nullptr || type == nullptr) {
        return -1;
    }
    device_ = device;
    type_ = type;
    baudrate_ = baudrate;
    stop_.store(false, std::memory_order_release);
    running_.store(true, std::memory_order_release);
    thread_ = std::thread(&Rs232ImuBackend::readLoop, this);
    return 0;
}

void Rs232ImuBackend::stop() {
    stop_.store(true, std::memory_order_release);
    if (thread_.joinable()) {
        thread_.join();
    }
    running_.store(false, std::memory_order_release);
}
```

Add `readLoop()` with a conservative first parser boundary:

```cpp
void Rs232ImuBackend::readLoop() {
    int fd = openSerial(device_.c_str(), baudrate_);
    if (fd < 0) {
        running_.store(false, std::memory_order_release);
        return;
    }

    DriverSDK::imuStruct latest{};
    unsigned char buffer[512]{};
    while (!stop_.load(std::memory_order_acquire)) {
        ssize_t const n = read(fd, buffer, sizeof(buffer));
        if (n <= 0) {
            continue;
        }

        buffer_.publish(latest);
    }
    close(fd);
}
```

- [ ] **Step 4: Add type-specific IMU decoding inside `readLoop()`**

Use the working parser logic from the current robot branch or `heimaSDK/rs232.cpp`:

```text
Xsens:
  header bytes: 0xfa 0xff
  frame length: 50
  RPY byte offsets: 7, 11, 15
  ACC byte offsets: 22, 26, 30
  GYR byte offsets: 37, 41, 45

HiPNUC:
  header bytes: 0x5a 0xa5
  frame length: 82
  ACC byte offsets: 18, 22, 26
  GYR byte offsets: 30, 34, 38
  RPY byte offsets: 58, 54, 62

YeSense:
  header bytes: 0x59 0x53
  frame length: sizeof(YesenseImuFrame)
  use the existing Yesense decoder and publish only complete decoded frames
```

For every complete decoded sample, execute exactly one publish:

```cpp
buffer_.publish(latest);
```

Do not read `buffer_.readInto()` from the parser thread, and do not let `DriverSDK::getIMU()` access raw serial bytes.

- [ ] **Step 5: Build and run default tests**

Run:

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Expected: all tests pass.

- [ ] **Step 6: Validate with IMU hardware**

Run the project-specific IMU demo or a small SDK caller that calls `getIMU()` at 1 kHz.

Expected: `rpy`, `gyr`, and `acc` update without blocking motor backend threads.

- [ ] **Step 7: Commit**

```bash
git add include/rmd_can_sdk/rs232_imu_backend.h src/rs232_imu_backend.cpp CMakeLists.txt
git commit -m "Connect RS232 IMU parser to snapshot backend"
```

---

### Task 11: Add Hardware Validation Scripts

**Files:**
- Create: `scripts/run_mixed_bus_validation.sh`
- Modify: `docs/usage_guide_zh.md`

- [ ] **Step 1: Add target validation script**

Create `scripts/run_mixed_bus_validation.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail

BUILD_DIR="${BUILD_DIR:-build-ecat}"
CONFIG="${1:-config/example_rmd_can.xml}"

cmake -S . -B "${BUILD_DIR}" -DRMD_CAN_SDK_ENABLE_ECAT=ON
cmake --build "${BUILD_DIR}"
ctest --test-dir "${BUILD_DIR}" --output-on-failure

echo "Built and tested ${BUILD_DIR}"
echo "Use the hardware-specific mixed CAN/EtherCAT config as the first argument."
echo "Config path: ${CONFIG}"
```

- [ ] **Step 2: Make script executable**

Run:

```bash
chmod +x scripts/run_mixed_bus_validation.sh
```

- [ ] **Step 3: Document validation flow**

Add this section to `docs/usage_guide_zh.md`:

```markdown
## 混合 CAN/EtherCAT 实时 SDK 验证

第一阶段只验证 RMD 电机、CAN、EtherCAT 和 RS232 IMU。手爪、RS485 外设和非 RMD CAN 电机不在本阶段范围内。

在 EtherCAT 目标机上运行：

```bash
./scripts/run_mixed_bus_validation.sh path/to/mixed_rmd_config.xml
```

实时稳定性检查重点：

- `getMotorActual()` 返回完整快照，不直接访问 PDO 字段。
- `setMotorTarget()` 只发布完整目标帧。
- CAN 或 EtherCAT 反馈超时后，对应电机进入 stale/fault 状态。
- EtherCAT 编译必须显式启用 `-DRMD_CAN_SDK_ENABLE_ECAT=ON`。
```

- [ ] **Step 4: Run local non-hardware tests**

Run:

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Expected: all local tests pass.

- [ ] **Step 5: Commit**

```bash
git add scripts/run_mixed_bus_validation.sh docs/usage_guide_zh.md
git commit -m "Add mixed bus validation workflow"
```

---

## Final Verification

- [ ] Run default build without EtherCAT:

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Expected: all tests pass.

- [ ] Run EtherCAT-enabled build on target:

```bash
cmake -S . -B build-ecat -DRMD_CAN_SDK_ENABLE_ECAT=ON
cmake --build build-ecat
ctest --test-dir build-ecat --output-on-failure
```

Expected: all unit tests pass and `rmd_can_sdk` links with EtherCAT.

- [ ] Run hardware smoke tests with motors disabled or zero torque first.

Expected: CAN and EtherCAT RMD motors report feedback snapshots without stale state.

- [ ] Run mixed bus control at the target period.

Expected: no missed-deadline trend, no mixed-frame feedback, and stale/fault states are explicit when a bus or motor is disconnected.

## Self-Review Notes

Spec coverage:

- Whole-frame snapshot requirement is covered by Tasks 1, 5, 6, and 8.
- CAN backend isolation is covered by Task 5.
- EtherCAT backend and optional build gate are covered by Tasks 7 and 8.
- RS232 IMU snapshot boundary is covered by Task 9.
- Safety supervisor is covered by Task 3 and used by Tasks 5, 6, and 8.
- Hardware validation is covered by Task 11 and Final Verification.

Execution order is intentional: default build remains usable before EtherCAT is introduced, and EtherCAT hardware code is isolated behind an explicit CMake option.
