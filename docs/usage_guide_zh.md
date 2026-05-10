# RMD CAN SDK 使用指南

这份文档说明当前 `rmd_can_sdk` 的构建、配置、硬件验证和 C++ API 用法。

当前 SDK 是从 `heimaSDK` 剥离出来的 RMD 电机实时控制版本。默认构建保持
SocketCAN 可用；同一套 `DriverSDK` 电机 API 现在可以通过显式配置走 CAN 或
EtherCAT，EtherCAT 代码必须用 `-DRMD_CAN_SDK_ENABLE_ECAT=ON` 在目标机上启用：

- 通过 SocketCAN 使用普通 CAN 设备，例如 `can0`
- 读取 `configuration.xml` / `config.xml` 风格配置
- 支持 RMD 电机按 `bus="CAN"` / `bus="ECAT"` 归属到不同 backend
- 每个 backend 使用独立 SPSC snapshot buffer，应用线程只读完整 target/actual 帧
- 高层 `DriverSDK` API 使用 MIT 周期控制
- RS232 IMU 保持独立 snapshot 路径，不进入电机 PDO/SwapList
- 底层 transport/protocol 可直接发送标准 RMD 命令，例如 `0x9A`、`0x9C`、`0xA4`

## 1. 构建

在仓库根目录执行：

```bash
cmake -S rmd_can_sdk -B rmd_can_sdk/build
cmake --build rmd_can_sdk/build
ctest --test-dir rmd_can_sdk/build --output-on-failure
```

如果把这份工程从 x86_64 电脑拷到 Orin/Jetson，不能复用原来的
`rmd_can_sdk/build`。那个目录里的可执行文件是 x86 架构，Orin 上需要重新生成
ARM64/aarch64 版本：

```bash
./rmd_can_sdk/scripts/build_on_target.sh
```

如果当前目录已经是 SDK 本体，也就是能直接看到 `CMakeLists.txt`、`src`、
`include`、`scripts`，则执行：

```bash
./scripts/build_on_target.sh
```

这个脚本会在当前机器上重新运行 CMake、编译和 CTest；如果发现 `build` 里已经有
另一种 CPU 架构的可执行文件，会先删除旧的 `build` 再重新构建。

构建后会生成：

- `rmd_can_sdk/build/librmd_can_sdk.a`
- `rmd_can_sdk/build/rmd_can_demo`
- `rmd_can_sdk/build/rmd_status_probe`
- `rmd_can_sdk/build/rmd_move_zero`
- `rmd_can_sdk/build/rmd_mit_sweep_test`
- `rmd_can_sdk/build/rmd_can_sdk_tests`

## 2. CAN 口准备

确认 CAN 口存在：

```bash
ip -details link show can0
```

如果 `can0` 还没有配置成 1Mbps，可以执行：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

再次确认状态：

```bash
ip -details link show can0
```

期望看到 `can0` 是 `UP`，bitrate 是 `1000000`，状态不是 `BUS-OFF`。

### 2.1 Orin + KH/PEAK USB-CAN

Orin 自带的 CAN 通常是 `can0`、`can1`，驱动是 `mttcan`。安装
`KH-UCANFD_Linux_SDK` 后，PEAK/KH USB-CAN 应该由 `kcan` 驱动接管。建议把
USB-CAN 的 6 路固定命名为 `kcan1..kcan6`，避免和板载 `can0/can1` 混在一起。

安装 KH 驱动后，在 Orin 上执行：

```bash
./rmd_can_sdk/scripts/setup_orin_kcan_names.sh --reload
```

如果当前目录已经是 SDK 本体，则执行：

```bash
./scripts/setup_orin_kcan_names.sh --reload
```

如果 `kcan` 正在被程序占用，`--reload` 可能失败；这时先拔掉 USB-CAN，或者配置后
重启。成功后的期望状态是：

```text
can0
can1
kcan1
kcan2
kcan3
kcan4
kcan5
kcan6
```

启动某一路普通 RMD CAN，例如 `kcan1`：

```bash
sudo ip link set kcan1 down
sudo ip link set kcan1 type can bitrate 1000000 restart-ms 100
sudo ip link set kcan1 up
```

RMD 当前使用普通 8-byte CAN 帧，不需要 CAN FD 参数。XML 里 `<Master device="can0">`
也要改成实际接电机的接口，例如：

```xml
<Master order="0" device="kcan1" canhal="false" baudrate="1000000" canfd="false" dbaudrate="5000000" division="1"/>
```

### 2.2 Orin/KH 实测避坑清单

这次在 Orin + KH/PEAK USB-CAN + RMD 14 号电机上的实测，最容易踩的坑是：

- 不要复用 x86 电脑上的 `build`。Orin 是 `aarch64`，必须在 Orin 上重新编译：

```bash
./scripts/build_on_target.sh
```

- 如果脚本提示 `Permission denied`，只是脚本没有执行权限：

```bash
chmod +x ./scripts/*.sh
```

- `can0/can1` 是 Orin 板载 CAN，KH/PEAK USB-CAN 建议固定成 `kcan1..kcan6`。重启后先确认：

```bash
ip -br link | grep -E 'can|kcan'
lskcan
```

- 不要依赖默认波特率。`lskcan` 可能显示默认 `500000`，RMD 当前测试用 `1000000`。每次测试脚本都会设置；手动启动时要写：

```bash
sudo ip link set kcan1 down
sudo ip link set kcan1 type can bitrate 1000000 restart-ms 100
sudo ip link set kcan1 up
```

- XML 里的 `<Master device="...">` 必须和真实接口一致。电机接在 `kcan1`，就不要留 `device="can0"`：

```bash
grep 'Master order' config/example_rmd_can.xml
```

- 如果只拷贝了 SDK 本体到 Orin，常用路径是 `~/rmd_can_sdk/config/example_rmd_can.xml`；如果是完整外层工程，路径是 `rmd_can_sdk/config/example_rmd_can.xml`。脚本现在按执行命令时的当前目录解析相对路径。最稳的方式仍然是用绝对路径：

```bash
./scripts/run_mit_sweep_test.sh /home/eason/rmd_can_sdk/config/example_rmd_can.xml kcan1 1000000 0 4 2500 1000 20 50 2 10 10
```

- 先跑只读或 A4，再跑 MIT。不要一上来就跑高刚度 MIT：

```bash
sudo ./build/rmd_status_probe kcan1 14
./scripts/run_a4_response_test.sh kcan1 1000000 14 60 50 0 30 1000
./scripts/run_mit_sweep_test.sh /home/eason/rmd_can_sdk/config/example_rmd_can.xml kcan1 1000000 0 4 2500 1000 20 50 2 10 10
```

- `target=0deg start_angle=0deg no visible motion` 不是错误。目标和当前位置一样，电机本来就不该动。

- A4 能动但 MIT 不动时，优先检查 XML 的 `device`、`motor_id/slave_id`、MIT 帧 ID，以及是否真的打开了同一个 CAN 口。

- 判断链路是否健康，看这些状态比看“厂商驱动宣传”更可靠：

```bash
ip -details -statistics link show kcan1
```

期望看到 `ERROR-ACTIVE`，`tx/rx error`、`dropped`、`bus-off` 都为 0。

## 3. 先做只读通信验证

假设电机 CAN id 是 `14`：

```bash
sudo ./rmd_can_sdk/build/rmd_status_probe can0 14
```

这个工具会发送只读命令：

- `0x9A` 读电机状态 1
- `0x9C` 读电机状态 2，包含 `iq_a`、速度、角度
- `0x92` 读多圈角度

正常时会看到类似：

```text
TX 14E [8] 9A 00 00 00 00 00 00 00
RX 24E [8] 9A ...
TX 14E [8] 9C 00 00 00 00 00 00 00
RX 24E [8] 9C ...
TX 14E [8] 92 00 00 00 00 00 00 00
RX 24E [8] 92 ...
```

RMD 标准帧 id 规则：

- 主机发给电机：`0x140 + motor_id`
- 电机回复主机：`0x240 + motor_id`

所以 id `14` 对应：

- TX: `0x14E`
- RX: `0x24E`

## 4. 低速转到 0 度

已经提供一个保守测试工具：

```bash
sudo ./rmd_can_sdk/build/rmd_move_zero can0 14 0 10
```

参数含义：

```text
rmd_move_zero <can口> <motor_id> <目标角度deg> <最大速度dps>
```

例如：

```bash
sudo ./rmd_can_sdk/build/rmd_move_zero can0 14 0 10
sudo ./rmd_can_sdk/build/rmd_move_zero can0 14 30 10
```

这个工具使用标准 `0xA4` 位置命令，不是 MIT。它会先做保护检查：

- 先读 `0x9A`
- 如果 `error_state != 0`，不发运动命令
- `0x9A` 的 `DATA[3]` 只作为抱闸释放指令状态打印，不作为独立机械抱闸传感器硬拦截
- 限制最大速度必须在 `1..100 dps`

成功时输出类似：

```text
status1 temp=32 mos=39 brake_release_cmd=release voltage=52.6 error=0x0
status2 angle=78deg speed=0dps iq=0A
A4 accepted target=0deg max_speed=10dps
    ...
    status2 angle=0deg speed=0dps iq=-0.11A
    target reached
```

如果 `0xA4` 命令能收到回复但电机不动，可以单独跑 A4 响应诊断。这个工具不进入 MIT，只会发送标准 `0xA4` 位置命令，并周期读取 `0x9C` 判断是否开始运动、何时到达目标。默认跑 5 分钟，目标在 `0deg` 和 `30deg` 之间来回切换：

```bash
./rmd_can_sdk/scripts/run_a4_response_test.sh can0 1000000 14 300 50 0 30 1000
```

参数含义：

```text
run_a4_response_test.sh <can口> <波特率> <motor_id> <测试秒数> <最大速度dps> <目标A deg> <目标B deg> <采样Hz> [输出前缀]
```

它会生成两个 CSV：

- `*_samples.csv`: 每次 `0x9C` 采样，包含角度、速度、电流、目标误差、是否检测到运动。
- `*_summary.csv`: 每个 A4 目标的汇总，包含 A4 回复时间、明显运动时间窗口、到达目标时间窗口、最终误差。

这里的采样是主动 `0x9C` request/reply 采样，不是 MIT 周期反馈。单电机 1Mbps 下可以用 `1000Hz`，反应时间窗口分辨率约 `1ms`；如果后续同一条 CAN 上挂多电机，再降到 `200Hz` 或 `500Hz` 会更稳。

## 5. 跑 MIT 高层 SDK demo

```bash
sudo ./rmd_can_sdk/build/rmd_can_demo rmd_can_sdk/config/example_rmd_can.xml
```

这个 demo 会：

- 读取 XML 配置
- 初始化 `DriverSDK`
- 对配置里的 active motor 周期发送 MIT 帧
- 每 20ms 打印一次 `pos / vel / tor / status`

默认 demo 设置：

```cpp
target.enabled = 1;
target.kp = 0.0f;
target.kd = 0.0f;
target.tor = 0.0f;
```

也就是只用于验证 MIT 通信，不主动加刚度拉位置。

如果只接了一个 `can0` 和一个 RMD 电机，可以运行手动硬件扫动测试：

```bash
sudo ./rmd_can_sdk/build/rmd_mit_sweep_test rmd_can_sdk/config/example_rmd_can.xml 0 4 2500 1000 20
```

也可以用一键脚本配置 CAN 口并运行测试：

```bash
./rmd_can_sdk/scripts/run_mit_sweep_test.sh
```

脚本默认参数等价于：

```text
config=rmd_can_sdk/config/example_rmd_can.xml
can_if=can0
bitrate=1000000
zero_speed_dps=0
cycles=4
hold_ms=2500
sample_hz=1000
command_deg=57.2957795
eval_deg=57.2957795
ramp_ms=0
kp=30
kd=2
tolerance_deg=10
prezero_tolerance_deg=10
```

需要覆盖参数时：

```bash
./rmd_can_sdk/scripts/run_mit_sweep_test.sh <xml配置> <can口> <波特率> <回零最大速度dps> <正负摆动轮数> <每个目标保持毫秒> <读取采样Hz> [命令角度deg] [kp] [kd] [容差deg] [预定位容差deg] [评价角度deg] [ramp毫秒] [敏感运动位置阈值deg] [敏感运动速度阈值deg/s] [输出前缀]
```

参数含义：

```text
rmd_mit_sweep_test <xml配置> <回零最大速度dps> <正负摆动轮数> <每个目标保持毫秒> <读取采样Hz> [命令角度deg] [kp] [kd] [容差deg] [预定位容差deg] [评价角度deg] [ramp毫秒] [敏感运动位置阈值deg] [敏感运动速度阈值deg/s] [输出前缀]
```

这个工具默认不再依赖标准 `0xA4` 回零。默认 `<回零最大速度dps>` 为 `0`，表示跳过 A4；随后初始化高层 `DriverSDK`，先用 MIT 模式预定位到 `0rad`，再使用 MIT 模式在 `+命令角度` 和 `-命令角度` 之间来回切换。默认参数为 `kp=30`、`kd=2`、`tor=0`、正式扫动容差 `10deg`、预定位容差 `10deg`、`ramp_ms=0`、敏感运动阈值 `0.1deg` / `2deg/s`。不传 `[命令角度deg]` 时保持旧行为：`57.2957795deg`，也就是 `1rad`。不传 `[评价角度deg]` 时，评价角度等于命令角度；传入评价角度后，电机会按命令角度发 MIT 目标，但 `target_reached` 和误差会按评价角度统计。`ramp_ms=0` 表示阶跃目标；`ramp_ms>0` 时，每个目标切换会在指定毫秒内线性过渡到新命令目标，更接近 position-based RL 的连续位置命令。测试双足短行程时建议显式传 `20`，并把正式扫动容差设到 `5deg` 到 `10deg` 先看低层效果。预定位容差建议先保持 `10deg`，避免无前馈力矩造成的静态误差阻塞正式测试。

MIT 测试会生成两个 CSV：

- `*_samples.csv`: 每次 `getMotorActual()` 采样，包含命令目标、评价目标、位置、速度、力矩、评价误差、敏感/明显运动标记、状态字和错误码。
- `*_summary.csv`: 每个 MIT 目标 step 的汇总，包含命令目标、评价目标、`kp`、`kd`、容差、敏感运动时间窗口、明显运动时间窗口、到达评价目标时间窗口、最终评价误差、最大超调、最大速度和最大力矩。

例如测试 `±20deg` 短行程，目标保持 2 秒，跑 5 分钟：

```bash
./rmd_can_sdk/scripts/run_mit_sweep_test.sh rmd_can_sdk/config/example_rmd_can.xml can0 1000000 0 75 2000 1000 20 30 2 2
```

上面命令没有显式传预定位容差，所以预定位仍使用默认 `10deg`，正式 `±20deg` 扫动使用 `2deg` 容差。

如果明确想在 MIT 前额外跑一次传统 A4 回零，可以把 `<回零最大速度dps>` 设为大于 `0` 的值，例如 `50`。A4 当前只作为可选前置诊断，不作为 MIT 主测试的默认依赖：

```bash
./rmd_can_sdk/scripts/run_mit_sweep_test.sh rmd_can_sdk/config/example_rmd_can.xml can0 1000000 50 4 2500 1000 20 30 2 10 10
```

如果要测试 position-based RL 里的“位置给大一点”策略，例如命令 `±30deg`，但按实际是否到 `±20deg` 来统计，可以传 `[评价角度deg]`：

```bash
./rmd_can_sdk/scripts/run_mit_sweep_test.sh rmd_can_sdk/config/example_rmd_can.xml can0 1000000 0 10 2000 1000 30 50 2 5 10 20
```

如果要更贴近 RL 的连续位置输出，可以再传 `ramp_ms`。例如 2 分钟测试，命令 `±25deg`、评价 `±20deg`、每次切换用 `250ms` 线性过渡，目标变化速度约 `200deg/s`：

```bash
./rmd_can_sdk/scripts/run_mit_sweep_test.sh rmd_can_sdk/config/example_rmd_can.xml can0 1000000 0 30 2000 1000 25 50 2 2 10 20 250
```

每次目标切换时会输出三个时间窗口：

- `sensitive_motion_start_window_ms`: 从发送 `setMotorTarget()` 开始，到检测到刚开始反应的时间。默认敏感运动定义为位置相对起点变化 `>= 0.1deg`，或速度相对起点变化 `>= 2deg/s`。
- `motion_start_window_ms`: 从发送 `setMotorTarget()` 开始，到检测到明显运动的时间。明显运动定义为位置相对起点变化 `>= 1deg`，或速度相对起点变化 `>= 5deg/s`。
- `target_reached_window_ms`: 从发送 `setMotorTarget()` 开始，到进入命令行指定目标容差内的时间。

窗口格式是 `[下界, 上界]`，因为读取采样不是连续的。例如读取 `1000Hz` 时采样周期约 `1ms`，检测到事件的真实时间只能确定在最近两次读取之间。程序也会打印 XML 控制周期，例如示例配置 `period=1000000ns`，单电机约 `1000Hz`。

RMD MIT 帧 id 规则：

- 主机发给电机：`0x400 + motor_id`
- 电机回复主机：`0x500 + motor_id`

所以 id `14` 对应：

- TX: `0x40E`
- RX: `0x50E`

## 6. XML 配置

示例配置在：

```text
rmd_can_sdk/config/example_rmd_can.xml
```

当前解析器主要读取：

- `<CAN><Masters period="...">`
- `<CAN><Masters><Master .../>`
- `<CAN><Slaves><Slave ...>1</Slave>`
- `<Motors><Motor alias="...">`

只有满足这些条件的电机会被启用：

- `<Slave>` 文本值是 `1`
- `type` 以 `RMD` 开头
- `alias` 能在 `<Motors>` 里找到对应参数
- `slave_id` 大于 0

单电机配置示例：

```xml
<Masters period="2000000">
    <Master order="0" device="can0" baudrate="1000000" division="1"/>
</Masters>
<Slaves>
    <Slave master="0" slave_id="14" alias="1" type="RMD-X12-P20-320">1</Slave>
</Slaves>
<Motors>
    <Motor limb="0" motor="0" alias="1">
        <Polarity>1</Polarity>
        <CountBias>0</CountBias>
        <EncoderResolution>131072</EncoderResolution>
        <GearRatioTor>20</GearRatioTor>
        <GearRatioPosVel>20</GearRatioPosVel>
        <RatedCurrent>42.42</RatedCurrent>
        <TorqueConstant>2.81</TorqueConstant>
        <RatedTorque>85</RatedTorque>
        <MaximumTorque>320</MaximumTorque>
        <MinimumPosition>-1.0</MinimumPosition>
        <MaximumPosition>1.0</MaximumPosition>
    </Motor>
</Motors>
```

多电机时增加多条 `<Slave>` 和对应 `<Motor>`：

```xml
<Slaves>
    <Slave master="0" slave_id="11" alias="1" type="RMD-X12-P20-320">1</Slave>
    <Slave master="0" slave_id="12" alias="2" type="RMD-X12-P20-320">1</Slave>
    <Slave master="0" slave_id="13" alias="3" type="RMD-X12-P20-320">1</Slave>
    <Slave master="0" slave_id="14" alias="4" type="RMD-X12-P20-320">1</Slave>
    <Slave master="0" slave_id="15" alias="5" type="RMD-X12-P20-320">1</Slave>
    <Slave master="0" slave_id="16" alias="6" type="RMD-X12-P20-320">1</Slave>
</Slaves>
```

`alias` 是 SDK 数组下标的来源：

- `alias="1"` 对应 `targets[0]` / `actuals[0]`
- `alias="2"` 对应 `targets[1]` / `actuals[1]`
- 以此类推

`slave_id` 是实际 CAN 电机 id。

## 6.1 多路 USB-CAN 配置

这个 SDK 使用 Linux SocketCAN：

```cpp
socket(PF_CAN, SOCK_RAW, CAN_RAW)
bind(can0 / can1 / ...)
```

所以多路 USB-CAN 是否支持，取决于设备有没有在系统里枚举成 `can0`、`can1`、`can2` 这类 SocketCAN 网络接口。

你的 PEAK 多路 USB-CAN 当前枚举形态是匹配的：

```text
can0 UP     pcan_usb_pro_fd bitrate 1000000
can1 DOWN   pcan_usb_pro_fd
can2 DOWN   pcan_usb_pro_fd
can3 DOWN   pcan_usb_pro_fd
can4 DOWN   pcan_usb_pro_fd
can5 DOWN   pcan_usb_pro_fd
```

注意：SDK 读取 XML 里的 `baudrate`，但目前不会自动执行 `ip link set ... bitrate ... up`。使用前需要先把每一路 CAN 由系统配置好：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
```

如果要启用 `can0`、`can1`、`can2` 三路，每路一个 master：

```xml
<Masters period="2000000">
    <Master order="0" device="can0" baudrate="1000000" division="1"/>
    <Master order="1" device="can1" baudrate="1000000" division="1"/>
    <Master order="2" device="can2" baudrate="1000000" division="1"/>
</Masters>
<Slaves>
    <Slave master="0" slave_id="11" alias="1" type="RMD-X12-P20-320">1</Slave>
    <Slave master="0" slave_id="12" alias="2" type="RMD-X12-P20-320">1</Slave>
    <Slave master="1" slave_id="13" alias="3" type="RMD-X12-P20-320">1</Slave>
    <Slave master="1" slave_id="14" alias="4" type="RMD-X12-P20-320">1</Slave>
    <Slave master="2" slave_id="15" alias="5" type="RMD-X12-P20-320">1</Slave>
    <Slave master="2" slave_id="16" alias="6" type="RMD-X12-P20-320">1</Slave>
</Slaves>
```

这里的含义是：

- `Master order="0"` 对应 `can0`
- `Master order="1"` 对应 `can1`
- `Master order="2"` 对应 `can2`
- `Slave master="0"` 表示这个电机在 `can0` 上
- `Slave master="1"` 表示这个电机在 `can1` 上
- `Slave master="2"` 表示这个电机在 `can2` 上

SDK 会为每个 master 创建独立的 CAN socket、TX 线程、RX 线程和 mask。不同 CAN 口上的电机可以使用相同 `slave_id`，但建议调试阶段仍然让 id 全局唯一，排查更简单。

## 7. 控制频率

高层 MIT 发送周期来自：

```xml
<Masters period="2000000">
```

单位是 ns：

- `1000000` = 1ms = 1000Hz
- `2000000` = 2ms = 500Hz
- `2500000` = 2.5ms = 400Hz
- `4000000` = 4ms = 250Hz

实际每个 master 的频率是：

```text
frequency_hz = 1e9 / (period_ns * division)
```

例如：

```xml
<Masters period="1000000">
    <Master order="0" device="can0" division="2"/>
</Masters>
```

等价于 `500Hz`。

如果一条 1Mbps CAN 总线上最多挂 6 个电机，建议：

- 保守调试：`250Hz`
- 常规使用：`400Hz`
- 上限尝试：`500Hz`
- 不建议 6 电机同总线跑 `1000Hz`

当前 SDK 每 `20 tick` 额外查询一次 `0x9C`，用于通过 `iq_a` 估算真实力矩：

- MIT `500Hz` 时，`0x9C` 约 `25Hz`
- MIT `400Hz` 时，`0x9C` 约 `20Hz`

## 8. C++ API 用法

头文件有两个包含方式：

```cpp
#include "heima_driver_sdk.h"
```

或：

```cpp
#include "rmd_can_sdk/heima_driver_sdk.h"
```

最小 MIT 控制代码：

```cpp
#include "heima_driver_sdk.h"

#include <chrono>
#include <thread>
#include <vector>

int main() {
    auto& sdk = DriverSDK::DriverSDK::instance();
    sdk.init("rmd_can_sdk/config/example_rmd_can.xml");

    int const n = sdk.getTotalMotorNr();
    std::vector<DriverSDK::motorTargetStruct> targets(n);
    std::vector<DriverSDK::motorActualStruct> actuals(n);

    for (int index : sdk.getActiveMotors()) {
        targets[index].enabled = 1;
        targets[index].pos = 0.0f;   // rad
        targets[index].vel = 0.0f;   // rad/s
        targets[index].tor = 0.0f;   // MIT feedforward torque, Nm
        targets[index].kp = 5.0f;
        targets[index].kd = 0.2f;
    }

    sdk.setMotorTarget(targets);

    for (int i = 0; i < 500; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        sdk.getMotorActual(actuals);
    }

    for (auto& target : targets) {
        target.enabled = 0;
        target.kp = 0.0f;
        target.kd = 0.0f;
        target.tor = 0.0f;
    }
    sdk.setMotorTarget(targets);
}
```

如果你的程序用 CMake，可以这样链接：

```cmake
add_subdirectory(rmd_can_sdk)

add_executable(my_control main.cpp)
target_link_libraries(my_control PRIVATE rmd_can_sdk)
```

## 9. 数据单位和力矩语义

`motorTargetStruct`：

- `pos`: MIT 目标位置，单位 `rad`
- `vel`: MIT 目标速度，单位 `rad/s`
- `tor`: MIT 前馈力矩 `t_ff`，单位 `Nm`
- `kp`: MIT 位置刚度
- `kd`: MIT 速度阻尼
- `enabled`: `1` 时发送目标，`0` 时发送零目标

V4.4 协议里 `t_ff` 是关节输出端前馈力矩，单位 `Nm`，SDK 按 XML 中每个电机的
`MaximumTorque` 映射到 12 位字段。电机驱动内部再按
`IqRef = [kp*(p_des-p_fb) + kd*(v_des-v_fb) + t_ff] / KT_OUT` 换算成输出电流；
这里最后是除以扭矩系数，不是在 SDK 里把 `t_ff` 先乘成电流。

`motorActualStruct`：

- `pos`: 当前位置，单位 `rad`，来自 MIT 高频回包
- `vel`: 当前速度，单位 `rad/s`，来自 MIT 高频回包
- `tor`: 当前真实力矩估算，来自标准 `0x9C` 的 `iq_a * TorqueConstant`，频率低于 MIT；这是电流反推力矩，方向与上面的 MIT 电流公式相反
- `temp`: `0x9C` 返回的温度
- `statusWord`: 当前简化状态字
- `errorCode`: 当前简化错误码

注意：MIT 回包里的 `t` 当前只作为 MIT feedback torque 解析保留，不再作为 `actual.tor` 输出。`actual.tor` 使用普通 `0x9C` 的 `iq_a` 估算，因为这更接近真实电流/力矩。`0x9C` 只更新 `tor/temp` 等辅助信息，不再覆盖 `actual.pos/actual.vel`，避免整数角度反馈污染 position-based RL 的 observation。

对 position-based RL，建议主 observation 使用：

- `actual.pos`: 高频 MIT 位置
- `actual.vel`: 高频 MIT 速度
- 上一次 policy 输出的目标位置
- `statusWord/errorCode` 转成的健康标记

`actual.tor/temp` 更适合做 safety、logging 或额外诊断，不建议作为最核心的运动状态来源。

同一 CAN master 下默认仍等待所有 RMD 电机回包后发布完整 actual 快照；如果只收到部分电机回包并超过内部超时阈值，SDK 会发布部分快照，并把未更新电机标记为 stale：`statusWord = 0xffff`、`errorCode = 1`，位置/速度/力矩保留最后一次有效值。

## 10. 混合 CAN/EtherCAT 实时 SDK 验证

第一阶段只验证 RMD 电机、CAN、EtherCAT 和 RS232 IMU。手爪、RS485 外设和非 RMD
CAN 电机不在本阶段范围内。

混合电机配置建议在 `<Motors><Motor>` 上显式声明总线归属：

```xml
<Motor alias="1" bus="CAN" master="0" id="14" type="RMD-X12-P20-320">
  ...
</Motor>
<Motor alias="2" bus="ECAT" master="0" slave="3" type="RMD-X12-P20-320">
  ...
</Motor>
```

在 EtherCAT 目标机上运行：

```bash
./scripts/run_mixed_bus_validation.sh path/to/mixed_rmd_config.xml
```

实时稳定性检查重点：

- `getMotorActual()` 返回由应用线程合并后的完整快照，不直接访问 PDO 字段。
- `setMotorTarget()` 向每个 backend 的独立 target buffer 发布完整目标帧。
- CAN 或 EtherCAT 反馈超时后，对应电机进入 stale/fault 状态。
- EtherCAT 编译必须显式启用 `-DRMD_CAN_SDK_ENABLE_ECAT=ON`。
- RS232 IMU 通过独立 snapshot 路径读取，不参与电机 backend 周期。

脚本只负责 EtherCAT enabled 构建和 CTest。真正上电验证前，先确认目标机有 IgH
EtherCAT 头文件和库、CAN 口已配置、急停可用，并从零力矩/disabled target 开始。

### 10.1 YeSense IMU 单独验证

原版 heimaSDK 的 YeSense decoder 已接入 RS232 snapshot backend。配置示例：

```xml
<IMU device="/dev/ttyACM0" baudrate="921600" type="YeSense"/>
```

目标机上先单独验证 IMU，不启动电机：

```bash
./build/yesense_imu_test /dev/ttyACM0 921600 200
```

期望输出中的 `rpy[rad]`、`gyr[rad/s]`、`acc` 会随 IMU 运动持续变化。`DriverSDK::init()`
如果配置了 `<IMU>` 但串口无法打开，会直接失败，避免系统误以为 IMU 可用。

## 11. 当前边界

当前高层 `DriverSDK` API 已实现：

- `init`
- `getTotalMotorNr`
- `getActiveMotors`
- `setMotorTarget`
- `getMotorActual`
- `setCntBias`
- `setMode`

CAN 后端仍按 RMD MIT 周期控制语义运行。EtherCAT MT_Device 后端单独按原 heimaSDK
已验证语义处理 PDO 单位、`countBias` 和 `setMode()`。

`setCntBias()` 只应在 `init()` 前调用；`init()` 后 backend 已持有运行时 registry 引用，
SDK 会拒绝重新写入 bias，避免控制线程运行期间重建映射造成数据竞争。

`setMode()` 也只应在 `init()` 前调用。初始化后 EtherCAT target frame 使用冻结的 mode
表写入 MT_Device RxPDO `Mode` 字段，SDK 会拒绝运行期改 mode，避免控制循环和外部线程同时
读写 mode vector。CAN MIT 周期控制不使用该 mode 字段。

以下接口为了兼容 `heimaSDK` 头文件存在，但当前未实现，会返回 `std::numeric_limits<int>::max()` 表示 unsupported：

- SDO / REG
- digit
- calibrate

`getIMU()` 当前从 IMU snapshot buffer 读取最新完整样本；配置 YeSense IMU 后，
`DriverSDK::init()` 会启动 RS232 backend 并把完整解码样本发布到 snapshot。如果未配置
IMU，则返回默认零样本。`getSensor()` 仍会清空输出结构并返回 unsupported。
EtherCAT MT_Device 会按原 heimaSDK 逻辑处理 Mode 8/5 下的 torque、kp、kd。

标准 RMD 命令目前可以参考这些示例直接使用底层 transport/protocol：

- `examples/rmd_status_probe.cpp`
- `examples/rmd_move_zero.cpp`

如果后面要把 `0xA1` 电流、`0xA2` 速度、`0xA4` 位置等也做成高层 API，可以在这个 SDK 上继续加一层 mode-specific wrapper。

## 12. 三缓冲、数据撕裂和实时性

当前 SDK 里的高层电机数据交换使用 `RmdCanSdk::FrameBuffer<T>`，位置在：

```text
rmd_can_sdk/include/rmd_can_sdk/rmd_realtime_core.h
```

这是明确的 SPSC 三缓冲所有权模型：

- `writing_`: producer 独占写缓冲
- `latest_`: 原子交换的最新完整帧
- `reading_`: consumer 独占读缓冲
- `dirty_`: 标记 producer 是否发布了新帧，避免 consumer 重复读取时把旧帧轮换回 latest

发布和读取通过 `std::atomic<int>::exchange(..., std::memory_order_acq_rel)` 交换缓冲区下标。这样 producer 不会写 consumer 正在读的 buffer，consumer 也不会读 producer 正在写的 buffer，所以不会出现一半字段来自旧帧、一半字段来自新帧的撕裂快照。

混合 CAN/EtherCAT 下没有让多个 backend 线程共用同一个 SPSC 三缓冲：

- 每个 backend 有自己的 target buffer：app thread -> 该 backend thread
- 每个 backend 有自己的 actual buffer：该 backend thread -> app thread
- `setMotorTarget()` 会把完整目标帧发布到每个 backend 的 target buffer
- `getMotorActual()` 会分别读取每个 backend 的 latest snapshot，然后按 registry 合并到用户传入的 `actuals` 数组

这意味着每一路 backend 的快照是完整帧。CAN 和 EtherCAT 之间没有强制全局同步屏障，所以两个 backend 的 actual 可能来自相邻周期；这通常符合混合总线的实际模型。如果后续需要“所有总线同一个逻辑周期”再一起发布，需要再加一个跨 backend 的 global frame coordinator。

实时性边界：

- TX 循环内不再拿 target mutex，也不再动态分配目标数组。
- CAN RX 解析 MIT / `0x9C` feedback 时直接使用固定 8 字节 `CanFrame`，不再在循环内构造
  `std::vector<unsigned char>`。
- RX 发布 actual 时不再拿全局 actual mutex。
- backend status 包含原子发布的运行指标：周期数、deadline miss、last/max cycle ns、
  stale frame、CAN RX timeout、EtherCAT incomplete WKC domain 计数。
- SocketCAN 的 `write/read/poll` 仍然是 Linux 系统调用，不等同于 EtherCAT master 那种硬实时路径。
- 当前线程还没有设置 `SCHED_FIFO`、CPU affinity、内存锁页。
- `setMotorTarget()` 和 `getMotorActual()` 假设由一个上层控制线程调用；不要多个 app 线程同时读写同一个 SDK 实例。

所以现在的状态是：数据快照层面已经按三缓冲处理撕裂风险；backend 循环内的动态分配和状态读写
竞争已经进一步收紧；真正上机器人时仍需要 Thor 侧 `SCHED_FIFO`、CPU affinity、`mlockall()` 和
总线负载/jitter 验证。

## 13. 安全建议

第一次试电机时建议按这个顺序：

1. 电机固定牢靠，机械输出端不要挂危险负载。
2. 确认 CAN id、供电电压、急停方式。
3. 先跑 `rmd_status_probe`，确认只读通信。
4. 再跑 `rmd_move_zero can0 <id> 0 10`，低速小范围移动。
5. 最后再跑 MIT demo，并从很小的 `kp/kd/tor` 开始。

对 6 电机单 CAN 总线，建议先把 XML 频率设到 `400Hz` 或 `500Hz`，确认稳定后再调高。不要一开始就用大刚度、大速度或大前馈力矩。
