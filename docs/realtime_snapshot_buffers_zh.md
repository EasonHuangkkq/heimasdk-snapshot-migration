# Realtime Snapshot Buffer 说明

这份文档说明 SDK 现在使用的 `SnapshotBuffer<T>` / `FrameBuffer<Frame>` 数据交换模型。

## 1. 为什么需要 snapshot buffer

电机和 IMU 的 realtime 数据不能用逐字段共享。比如 `pos` 来自新一帧，`vel` 还来自旧一帧，就会得到撕裂快照。对控制系统来说，读到稍旧的一整帧通常可以接受，读到半新半旧的一帧更危险。

所以 SDK 需要的语义是：

- producer 每次发布一个完整对象。
- consumer 每次读取一个完整对象。
- consumer 可以读到旧帧，但不能读到半帧。
- producer 不写 consumer 正在读的 slot。
- consumer 不读 producer 正在写的 slot。

当前实现是 SPSC snapshot buffer。SPSC 指 single producer / single consumer，也就是一个写线程、一个读线程。它不是 queue，不保证每一帧都被消费。

## 2. 当前唯一实现

底层实现是：

```text
include/rmd_can_sdk/rmd_snapshot_buffer.h
```

核心类型是：

```cpp
template <typename T>
class SnapshotBuffer {
public:
    explicit SnapshotBuffer(T initial = T{});
    void publish(T const& value);
    T read() const;
    void readInto(T& out) const;
};
```

`SnapshotBuffer<T>` 是唯一的三槽并发算法实现。

业务层使用的名字是：

```cpp
template <typename Frame>
using FrameBuffer = SnapshotBuffer<Frame>;
```

位置：

```text
include/rmd_can_sdk/rmd_realtime_core.h
```

也就是说，runtime 里看到的 `FrameBuffer<...>` 只是表达“完整帧交换”的语义名字，底层实现统一走 `SnapshotBuffer`。

## 3. 当前使用点

现在电机和 IMU 都走同一套 snapshot buffer：

- CAN target: `FrameBuffer<MotorTargetFrame>`
- CAN actual: `FrameBuffer<MotorActualFrame>`
- EtherCAT target: `FrameBuffer<EthercatPackedTargetFrame>`
- EtherCAT actual: `FrameBuffer<MotorActualFrame>`
- IMU: `ImuSnapshotBuffer = FrameBuffer<DriverSDK::imuStruct>`

CAN 和 EtherCAT 的区别只在 target frame：

- CAN backend 读取 `MotorTargetFrame` 后，在 TX 线程里打包 MIT CAN 帧。
- EtherCAT backend 读取 `EthercatPackedTargetFrame`，实时线程直接写 PDO bytes。

actual 侧统一发布 `MotorActualFrame`。

IMU 不进入电机 PDO，也不和电机 actual 混在同一个 frame 里。RS232 IMU backend 解析出完整 `imuStruct` 后发布到 `ImuSnapshotBuffer`，上层通过 `DriverSDK::getIMU()` 读取最新完整样本。

## 4. 算法怎么工作

内部有三个 slot：

```cpp
std::array<T, 3> buffers_;
```

同时维护：

- `writing_`: producer 当前独占写入的 slot。
- `reading_`: consumer 当前独占读取的 slot。
- `latest_`: 原子状态，保存“最新完整帧”的版本号和 slot index。
- `writeVersion_`: producer 本地版本号。
- `readVersion_`: consumer 本地已读版本号。

`latest_` 把两个信息打包到一个 `uint64_t`：

```text
latest_ = (version << 2) | index
```

低 2 bit 是 slot index，高位是版本号。

producer 的流程：

1. 把完整对象写入 `buffers_[writing_]`。
2. 递增 `writeVersion_`。
3. 用 `latest_.exchange(...)` 把刚写完的 slot 发布成最新完整帧。
4. 把旧 latest slot 拿回来作为下一次 `writing_`。

consumer 的流程：

1. 读取 `latest_`。
2. 如果版本号没有变化，继续读当前 `reading_` slot。
3. 如果版本号变化，用 CAS 把当前 `reading_` slot 归还给 producer，并接管最新完整帧 slot。
4. 把 `buffers_[reading_]` 整体复制到输出对象。

关键点是：slot 只有完整写入后才会发布，consumer 读的是完整对象。

## 5. 使用约束

`SnapshotBuffer` 的约束：

- 只支持一个 producer 和一个 consumer。
- 不适合作为多生产者队列。
- 不保证 consumer 看到每一次 publish。
- 适合 target / actual / IMU 这种“只需要最新完整快照”的数据。

如果未来需要多 producer、多 consumer、历史队列或跨 backend 同步，需要另外设计结构，不能直接复用这个类型。

## 6. 测试覆盖

相关测试在：

```text
tests/rmd_can_sdk_tests.cpp
```

重点测试包括：

- `testSnapshotBufferSnapshotsAreWholeFrames`
- `testSnapshotBufferSequenceNeverMovesBackward`
- `testFrameBufferSnapshotsAreWholeFrames`
- `testFrameBufferSequenceNeverMovesBackward`
- `testImuSnapshotBufferReturnsWholeSample`

这些测试覆盖两类风险：

- consumer 不应看到字段混杂的撕裂快照。
- consumer 不应看到 sequence 倒退。

## 7. 结论

当前 repo 内只有一套三槽 snapshot buffer 实现：`SnapshotBuffer<T>`。

对业务代码来说，继续使用 `FrameBuffer<...>`。这个名字更贴近当前系统的数据语义：发布和读取完整 frame。不要在新代码里重新引入第二套三缓冲实现。
