# Unified RMD Realtime SDK Design

Date: 2026-05-08

## Goal

Build one realtime motor SDK that supports RMD motors over both CAN and EtherCAT while keeping the public control API simple and stable.

The first version covers:

- RMD motor control over SocketCAN.
- RMD motor control over EtherCAT, using the already working EtherCAT configuration and protocol knowledge.
- RS232 IMU snapshot access as an independent module.
- Whole-frame target and feedback exchange to avoid the original heimaSDK `SwapList` field-tearing problem.
- Explicit safety behavior for stale feedback, bus errors, and out-of-range commands.

The first version does not cover:

- Hand grippers.
- RS485 peripherals.
- Non-RMD CAN motor brands.
- Web control or runtime hot plugging.

## First-Principles Model

A realtime motor SDK needs only four essential runtime concepts:

1. Target frame: the desired command for all motors in one control cycle.
2. Feedback frame: the observed state of all motors in one control cycle.
3. Bus backend: the transport-specific implementation for CAN or EtherCAT.
4. Safety supervisor: the rules for limits, stale data, bus faults, and degraded operation.

Everything else is implementation detail. CAN IDs, EtherCAT PDO offsets, socket handles, domain pointers, and IMU serial parsing must not leak into application control code.

## Latency Analysis For Snapshot Frames

The snapshot design should not add a full control-cycle delay when implemented correctly.

The dangerous design is:

```text
EtherCAT/CAN thread produces data
    -> waits for app thread
    -> app thread waits for backend
    -> backend publishes next cycle
```

This would add scheduling latency and create priority inversion risk. The proposed design does not do this.

The proposed design is:

```text
Backend realtime thread receives/parses current bus data
    -> writes one preallocated MotorActualFrame
    -> atomically publishes the frame pointer/index

Application thread calls getMotorActual()
    -> atomically reads the latest complete frame
    -> copies or references the complete snapshot
```

The extra cost is one bounded memory copy or pointer/index exchange. For typical motor counts, the copy is much smaller than the CAN/EtherCAT cycle budget. For example, 32 motors with a 64-byte aligned actual state is about 2 KiB per frame. At 1 kHz that is about 2 MiB/s of memory bandwidth, which is negligible on the target class of CPU compared with syscall, EtherCAT master, CAN bus, and control computation costs.

The snapshot can add at most "latest published frame" semantics, not an intentional one-cycle delay. If the application reads just before a backend publish, it sees the previous complete frame. If it reads just after publish, it sees the new complete frame. This is already true of any asynchronous bus system. The important property is that every returned frame is internally consistent.

Latency rules:

- No locks in realtime publish/read paths.
- No dynamic allocation after `init()`.
- No logging, XML parsing, map lookup, or exception throwing in realtime loops.
- No blocking waits between application thread and backend thread.
- Use fixed-size or preallocated frame storage.
- Publish complete frames only.

## Architecture

```text
Application / Controller
        |
        v
DriverSDK public API
  setMotorTarget(vector<motorTargetStruct>)
  getMotorActual(vector<motorActualStruct>)
  getIMU(imuStruct&)
        |
        v
Realtime Core
  MotorRegistry
  FrameBuffer<MotorTargetFrame>
  FrameBuffer<MotorActualFrame>
  SafetySupervisor
        |
        +-------------------+
        |                   |
        v                   v
RmdCanBackend        RmdEthercatBackend
SocketCAN/MIT        EtherCAT PDO/CoE
        |
        v
RMD motors

RS232 IMU Backend
        |
        v
IMU snapshot buffer
```

## Public API Boundary

The existing public motor API remains the first compatibility target:

```cpp
int setMotorTarget(std::vector<motorTargetStruct> const& data);
int getMotorActual(std::vector<motorActualStruct>& data);
void getIMU(imuStruct& data);
```

Internally, these calls must not access CAN sockets, EtherCAT domains, PDO pointers, or `SwapList` nodes directly.

`setMotorTarget()` validates and publishes a complete target frame.

`getMotorActual()` reads one complete actual snapshot and converts it to the compatibility struct format.

## Core Types

The new internal model uses fixed, whole-frame data.

```cpp
struct MotorTarget {
    float pos;
    float vel;
    float tor;
    float kp;
    float kd;
    int enabled;
};

struct MotorActual {
    float pos;
    float vel;
    float tor;
    short temp;
    short driveTemp;
    unsigned short voltage;
    unsigned short statusWord;
    unsigned short errorCode;
    bool valid;
    bool stale;
};

struct MotorTargetFrame {
    uint64_t sequence;
    TimePoint timestamp;
    std::array<MotorTarget, MaxMotors> motors;
    std::bitset<MaxMotors> valid;
};

struct MotorActualFrame {
    uint64_t sequence;
    TimePoint timestamp;
    std::array<MotorActual, MaxMotors> motors;
    std::bitset<MaxMotors> valid;
};
```

`MaxMotors` is a compile-time constant or init-time fixed capacity selected for the robot.

## FrameBuffer

Replace field-level `SwapList` access with a type-safe whole-frame buffer.

Requirements:

- Single producer, single consumer for each buffer instance.
- Three preallocated frame slots.
- `publish()` exchanges a complete frame.
- `readInto()` returns one internally consistent frame.
- No pointer to the internal slot escapes to application code.
- No field-level accessor can load the active buffer pointer independently.

The old heimaSDK problem was not triple buffering itself. The problem was exposing field access through `DataWrapper::operator->()`, where every field access could reload `nodePtr`. The new buffer forbids that access pattern.

## MotorRegistry

During `init()`, parse configuration and compile all runtime mappings.

Runtime should use integer indexes only:

```text
alias -> global motor index
global motor index -> backend id
backend id -> backend local motor index
backend local motor index -> CAN ID or EtherCAT PDO offset
```

No string type checks, XML traversal, or associative map lookups are allowed in realtime loops.

## Backend Interface

Both motor backends implement the same conceptual interface:

```cpp
class MotorBackend {
public:
    virtual int start() = 0;
    virtual void stop() = 0;
    virtual BackendStatus status() const = 0;
};
```

Backends receive target frames from the core and publish actual frames back to the core. They do not expose transport-specific memory to the public SDK.

## RmdCanBackend

Responsibilities:

- Own SocketCAN transport.
- Periodically send RMD MIT commands.
- Optionally send lower-rate RMD status commands such as `0x9A` and `0x9C`.
- Parse MIT feedback and standard RMD status feedback.
- Publish complete `MotorActualFrame` snapshots.
- Mark motors stale on missing feedback.
- Send safe zero-torque or disable commands during stop or fault handling.

CAN backend timing:

- One periodic transmit loop.
- One receive loop, or one poll-based combined loop if measurements show it is more stable.
- No dynamic allocation in loops.

## RmdEthercatBackend

Responsibilities:

- Own EtherCAT master/domain lifecycle.
- Use the already working RMD EtherCAT PDO mapping.
- Maintain local shadow copies for RxPDO and TxPDO data.
- Convert the latest target frame into RxPDO command data.
- Convert TxPDO data into a complete `MotorActualFrame`.
- Publish only whole actual frames.
- Report EtherCAT domain/master state to the safety supervisor.

The backend may reuse verified EtherCAT protocol constants and PDO layouts from heimaSDK, but must not expose `drivers[i].tx->Field` or `DataWrapper` to the public SDK layer.

EtherCAT realtime loop:

```text
receive/process domain
copy or snapshot TxPDO memory
parse all configured motors into MotorActualFrame
read latest MotorTargetFrame
write RxPDO commands
queue/send domain
publish MotorActualFrame
```

This preserves realtime behavior while preventing mixed-frame field reads.

## RS232 IMU Backend

IMU is separate from motor control.

Responsibilities:

- Own serial device and parser.
- Publish latest complete IMU snapshot.
- Expose `getIMU()` through the SDK compatibility API.

IMU timing must not block motor backend loops.

## Safety Supervisor

Safety is a core component, not demo logic.

Responsibilities:

- Validate target vector size.
- Clamp or reject position, velocity, torque, `kp`, and `kd` according to configured limits.
- Track per-motor feedback age.
- Track per-backend bus health.
- Mark stale motors explicitly.
- Convert backend faults into public `statusWord` and `errorCode`.
- Define stop behavior for CAN and EtherCAT motors.

Initial policy:

- Out-of-range targets are clamped before backend transmission, and optionally reported.
- Missing feedback beyond timeout marks the motor stale and invalid.
- A stale enabled motor receives safe zero-torque or disable command according to backend capability.
- EtherCAT domain/master failure marks all motors on that backend stale.
- CAN receive timeout marks only affected motors stale.

## Realtime Constraints

After `DriverSDK::init()` completes:

- No heap allocation in backend loops.
- No XML reads.
- No `std::map` or string lookup in backend loops.
- No `std::cout`, `printf`, or logging from realtime loops except optional lock-free counters.
- No exceptions from realtime loops.
- No blocking calls on application thread from backend thread.
- No blocking calls on backend thread from application thread.
- No public API access to backend-owned raw pointers.

## Configuration

The configuration should declare motor bus placement explicitly.

Example shape:

```xml
<Motors>
  <Motor alias="1" bus="CAN" master="0" id="14" type="RMD-X12">
    <Polarity>1</Polarity>
    <CountBias>0</CountBias>
    <MaximumTorque>320</MaximumTorque>
    <MinimumPosition>-1.5</MinimumPosition>
    <MaximumPosition>1.5</MaximumPosition>
  </Motor>
  <Motor alias="2" bus="ECAT" master="0" slave="3" type="RMD-X12">
    <Polarity>1</Polarity>
    <CountBias>0</CountBias>
    <MaximumTorque>320</MaximumTorque>
    <MinimumPosition>-1.5</MinimumPosition>
    <MaximumPosition>1.5</MaximumPosition>
  </Motor>
</Motors>
```

The parser may initially support the current config shape and normalize it into the new runtime registry. The runtime core should not depend on XML layout.

## Migration Strategy

Phase 1: Design boundary and internal types.

- Add `MotorTargetFrame`, `MotorActualFrame`, `FrameBuffer`, and backend interface.
- Keep public API compatible.
- Add deterministic tests proving whole-frame reads are internally consistent.

Phase 2: Move CAN into `RmdCanBackend`.

- Reuse current standalone RMD CAN protocol and SocketCAN transport.
- Publish actuals through the new frame buffer.
- Keep existing RMD CAN examples working.

Phase 3: Add `RmdEthercatBackend`.

- Reuse verified EtherCAT PDO layout and state-machine knowledge.
- Do not reuse public `SwapList/DataWrapper` field access.
- Add tests for PDO parsing/packing outside realtime hardware loops.

Phase 4: Add RS232 IMU snapshot backend.

- Keep IMU independent from motor frame timing.

Phase 5: Hardware validation.

- CAN-only RMD test.
- EtherCAT-only RMD test.
- Mixed CAN + EtherCAT RMD test.
- IMU read while motors run.
- Feedback timeout and bus fault tests.

## Testing Requirements

Unit tests:

- FrameBuffer never returns torn frames.
- Config normalizes aliases and backend mappings correctly.
- Target limits clamp or reject deterministically.
- RMD CAN MIT codec remains protocol-correct.
- EtherCAT PDO pack/parse functions match known byte layouts.
- Stale feedback sets expected status and error fields.

Integration tests:

- SDK initializes with CAN-only config.
- SDK initializes with EtherCAT-only config when EtherCAT support is enabled.
- SDK initializes with mixed config.
- `setMotorTarget()` and `getMotorActual()` work with mixed backend ownership.

Hardware tests:

- RMD CAN motor holds zero torque safely.
- RMD EtherCAT motor holds zero torque safely.
- Mixed bus loop runs at target period without missed deadlines beyond threshold.
- Disconnect or stop one CAN motor and verify stale state.
- EtherCAT domain fault marks backend degraded.

## Decision

Use scheme C: a new realtime core with isolated CAN, EtherCAT, and IMU backends.

Do not keep the original heimaSDK architecture as the center of the new SDK. Reuse only verified protocol details and hardware-specific knowledge. The public SDK should operate on complete target and actual frames, and the realtime implementation should prevent field-level tearing by construction.
