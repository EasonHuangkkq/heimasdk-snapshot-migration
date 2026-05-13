# EtherCAT 快速上手

这份文档面向 RMD `MT_Device` EtherCAT 电机。当前实现来自已验证的
heimaSDK EtherCAT 路径，并重新整理为 typed PDO、版本化三缓冲和独立 realtime
backend。

如果是在 Intel I210 上评估 IgH native `ec_igb` 驱动，先看
[I210 native ec_igb 编译与切换记录](ethercat_i210_igb_build_zh.md)。当前 Orin
机器已经配置成开机默认由 native `ec_igb` 接管 I210；这只启动 EtherCAT master
和网卡接管，不会自动启动 SDK 或电机控制。`generic` 路径已通过位置和长时间 mock
控制验证；native `ec_igb + dc=true` 已通过 13 电机 read-only、短时间 402 enable /
hold、5 分钟姿态保持。当前实现会在 DC 打开时显式选择第一个 MT_Device 作为
reference clock，避免默认落到 slave 0 的 EV1 分支器；MT_Device 的 SYNC0 shift
使用 `0`，避免 native DC 进入 OP 时出现 `0x001A Synchronization error`。SDK
start 会等待所有配置电机连续 20 个实时帧 fresh 后再返回；stop 会先降
SAFEOP/PREOP 再停实时线程。

## 1. 前提

- Linux 目标机已经安装 IgH EtherCAT master。
- `sudo ethercat slaves` 能看到从站。
- `libethercat` 能被链接器和运行时加载。
- 电机已经机械限位、架空或处于安全测试姿态。
- 外部急停可用。

如果 `libethercat.so` 安装在 `/usr/local/lib`：

```bash
echo /usr/local/lib | sudo tee /etc/ld.so.conf.d/local-libethercat.conf
sudo ldconfig
```

## 2. 确认总线

```bash
sudo ethercat slaves
```

带分支器的拓扑里，分支器只应该出现在 `ethercat slaves` 里，不要在 SDK XML
里配置成 `MT_Device`。XML 只配置实际 RMD 电机对应的 slave position。

示例：

```text
0  PREOP  +  EV1-5S(...)
1  PREOP  +  MT_Device
2  PREOP  +  MT_Device
```

这种情况下，XML 里第一个电机应该写 `slave="1"`，不是 `slave="0"`。

## 3. 构建

```bash
cmake -S . -B build-ecat -DRMD_CAN_SDK_ENABLE_ECAT=ON
cmake --build build-ecat
ctest --test-dir build-ecat --output-on-failure
```

如果 build 目录来自另一台机器，先删掉旧目录再重新构建：

```bash
rm -rf build-ecat
cmake -S . -B build-ecat -DRMD_CAN_SDK_ENABLE_ECAT=ON
cmake --build build-ecat
```

## 4. 配置要点

`<ECAT><Masters>` 里配置周期和 DC。目标控制路径使用 `dc="true"`。在带 EV1
分支器的拓扑里，SDK 会选择第一个 MT_Device slave 作为 DC reference clock：

```xml
<Master order="0" dc="true" period="1000000"/>
```

`period="1000000"` 表示 1 ms，也就是 1 kHz。

DC 下从 PREOP 到 OP 可能需要几秒。不要用 probe 开头的 stale 帧判断失败；SDK
会在 start 阶段等待 20 个连续 fresh 实时帧，默认最多 30 秒。可用
`RMD_ECAT_START_READY_FRAMES` 和 `RMD_ECAT_START_READY_TIMEOUT_MS` 临时调整诊断门槛。

当前 Orin 默认开机 native 接管 I210。确认方式：

```bash
systemctl is-enabled ethercat.service ethercat-i210-native-bind.service
grep -E '^(MASTER0_DEVICE|DEVICE_MODULES|UPDOWN_INTERFACES)=' /etc/ethercat.conf
readlink -f /sys/bus/pci/devices/0005:01:00.0/driver
sudo ethercat slaves
```

期望是 `DEVICE_MODULES="igb"`、`UPDOWN_INTERFACES=""`、驱动为
`/sys/bus/pci/drivers/ec_igb`，并且能扫到 15 个从站。手动调试 native
`ec_igb` 时，不要提前 `modprobe ec_igb`。先让 `ethercatctl start` 加载带
`main_devices=<MAC>` 的 `ec_master`，再通过 sysfs 把 `0005:01:00.0` 从普通
`igb` unbind 并 bind 到 `ec_igb`。

`<ECAT><Domains>` 定义 IgH domain。一个 domain 可以放一个电机，也可以放多个电机。
一个电机一个 domain 更接近原 heimaSDK 的配置方式；一个 master 一个 domain 通常
能减少 `ecrt_domain_process` / `ecrt_domain_queue` 调用次数。

```xml
<Domain master="0" order="0" division="1"/>
```

`<ECAT><Slaves>` 里把电机绑定到 domain 和 slave position：

```xml
<Slave master="0" domain="0" slave="1" alias="1" type="MT_Device">1</Slave>
```

`alias` 对应 `<Motors><Motor alias="...">`，也是 SDK 里电机数组的逻辑编号。
`slave` 是 IgH 看到的物理从站位置。

## 5. PDO 映射

当前 MT_Device 实现按运行模式选择 RxPDO：

| 模式 | 含义 | RxPDO | Rx 大小 | TxPDO | Tx 大小 |
| --- | --- | --- | --- | --- | --- |
| `8` | CSP/位置模式 | `0x1600` | 16 B | `0x1A02` | 22 B |
| `10` | CST/力矩模式 | `0x1600` | 16 B | `0x1A02` | 22 B |
| `5` | PVT/MIT 模式 | `0x1601` | 22 B | `0x1A02` | 22 B |

因此一个电机一个 domain 时：

- mode `8` / `10`：每个 domain 约 38 B
- mode `5`：每个 domain 约 44 B

模式和最大电流必须在 `DriverSDK::init()` 前设置：

```cpp
DriverSDK sdk;
std::vector<char> modes(motor_count, 8);
std::vector<unsigned short> max_current(motor_count, 500);
sdk.setMode(modes);
sdk.setMaxCurr(max_current);
sdk.init("config/example_rmd_ethercat_mt_device.xml");
```

## 6. 只读探测

先跑 snapshot probe，确认 TxPDO 能读到状态、位置、速度、力矩：

```bash
sudo ./build-ecat/rmd_ecat_snapshot_probe config/example_rmd_ethercat_mt_device.xml 450 8
```

建议至少 `450` 个采样。复杂拓扑或带分支器时，后面的 domain 进入稳定状态可能比
几十个采样更慢。

## 7. 位置保持

保持当前位置：

```bash
sudo ./build-ecat/rmd_ecat_hold_position config/example_rmd_ethercat_mt_device.xml 10000 500 450 5 current
```

参数含义：

```text
config.xml hold_ms max_current settle_samples period_ms target ramp_ms
```

- `hold_ms`: 总保持时间，单位 ms
- `max_current`: 写入 PDO 的最大电流/最大力矩限制字段
- `settle_samples`: 进入使能前读取并等待稳定的采样数
- `period_ms`: 应用层下发目标周期
- `target`: `current` 保持当前位置，`zero` 缓慢移动到 0 位
- `ramp_ms`: 移动到目标位置的斜坡时间，只对 `zero` 明显有用

零位保持示例：

```bash
sudo ./build-ecat/rmd_ecat_hold_position config/example_rmd_ethercat_mt_device.xml 30000 500 450 5 zero 8000
```

首次测试不要直接让带负载机构回零。先确认当前位置、限位、方向和急停。

## 8. Orin 13 电机长时间姿态保持

当前 Orin 13 电机拓扑使用 `config/configOriginal_heima.xml`。这里的 `motorN`
指 SDK 逻辑编号，也就是 XML 里的 `alias`，不是 EtherCAT slave position。按
`03241.urdf` 机构编号理解：

```text
motor1..6   = 右腿
motor7..12  = 左腿
motor13     = 腰
```

右腿 `motor1..6` 的机构语义：

```text
motor1 = 大腿 roll
motor2 = 大腿 yaw
motor3 = 大腿前后摆动，也就是 hip pitch
motor4 = 膝盖电机，通过拉杆控制小腿
motor5 = 脚踝十字轴之一，通过拉杆控制
motor6 = 脚踝十字轴之一，通过拉杆控制
```

左腿 `motor7..12` 按相同顺序镜像理解：

```text
motor7  = 大腿 roll
motor8  = 大腿 yaw
motor9  = 大腿前后摆动，也就是 hip pitch
motor10 = 膝盖电机，通过拉杆控制小腿
motor11 = 脚踝十字轴之一，通过拉杆控制
motor12 = 脚踝十字轴之一，通过拉杆控制
```

EtherCAT 物理链路里还有分支器，所以 slave position 不等同于机构编号：
slave 0 和 13 是 EV1 分支器，13 个 MT_Device 电机在 slave 1..12 和 14。

基准对称姿态：

```text
右腿: motor3=0.25, motor4=-0.30, motor5=0.34, motor6=0.29
左腿: motor9=0.25, motor10=-0.30, motor11=0.34, motor12=0.29
motor1/2/7/8/13: 保持启动时当前位置
```

长时间保持命令如下。整行复制执行，不要在中间手动回车；退出时按 `Ctrl+C`。

```bash
cd /home/eason/heimasdk-snapshot-migration && export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH && mkdir -p logs && sudo ethercat states PREOP >/dev/null && sudo ./build-open-source-check/rmd_ecat_position_targets config/configOriginal_heima.xml 300000 500 450 5 8000 3=0.25 4=-0.3 5=0.34 6=0.29 9=0.25 10=-0.3 11=0.34 12=0.29 | tee logs/rmd_ecat_position_targets_13motors_long_hold.log
```

更深对称站立姿态：

```text
右腿: motor3=0.35, motor4=-0.49, motor5=0.49, motor6=0.41
左腿: motor9=0.35, motor10=-0.49, motor11=0.49, motor12=0.41
motor1/2/7/8/13: 保持启动时当前位置
```

这组姿态先用 60 秒验证。整行复制执行；如果要跑 5 分钟，把第一个
`60000` 改成 `300000`。

```bash
cd /home/eason/heimasdk-snapshot-migration && export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH && mkdir -p logs && sudo ethercat states PREOP >/dev/null && sudo ./build-open-source-check/rmd_ecat_position_targets config/configOriginal_heima.xml 60000 500 450 5 8000 3=0.35 4=-0.49 5=0.49 6=0.41 9=0.35 10=-0.49 11=0.49 12=0.41 | tee logs/rmd_ecat_position_targets_13motors_deeper_60s.log
```

右腿 joint-space 测试使用 `rmd_ecat_joint_targets`。它先把
`hip_pitch/knee_pitch/ankle_pitch/ankle_roll` 解算为 motor3/4/5/6，再复用同一套
EtherCAT settle、402 enable、ramp 和 hold 流程。下面命令对应
`J_hip_r_pitch=0.35, J_knee_r_pitch=-0.70, J_ankle_r_pitch=0.35,
J_ankle_r_roll=0`，预期解析结果约为 `motor3=0.350000 motor4=-0.493657
motor5=0.476713 motor6=0.404559`。

```bash
cd /home/eason/heimasdk-snapshot-migration && export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH && mkdir -p logs && TS=$(date +%Y%m%d_%H%M%S) && sudo ethercat states PREOP >/dev/null && sudo env LD_LIBRARY_PATH="$LD_LIBRARY_PATH" RMD_ECAT_RT_CPU=11 RMD_ECAT_RT_PRIORITY=80 RMD_ECAT_APP_CPU=10 RMD_ECAT_APP_RT_PRIORITY=60 RMD_ECAT_BACKEND_STATUS_CSV="logs/rmd_ecat_joint_targets_backend_${TS}.csv" RMD_ECAT_APP_TIMING_CSV="logs/rmd_ecat_joint_targets_app_${TS}.csv" ./build-open-source-check/rmd_ecat_joint_targets config/configOriginal_heima.xml 60000 500 450 5 8000 right hip_pitch=0.35 knee_pitch=-0.70 ankle_pitch=0.35 ankle_roll=0 | tee "logs/rmd_ecat_joint_targets_${TS}.log"
```

右腿 joint path demo 使用同一组姿态作为路径起点和终点，中间点只做小幅变化。先用
audit 确认默认路径解析：

```bash
cd /home/eason/heimasdk-snapshot-migration && ./build-open-source-check/rmd_ecat_joint_path_demo --audit-self-test
```

悬吊状态下再跑 10 秒 EtherCAT 路径 demo：

```bash
cd /home/eason/heimasdk-snapshot-migration && export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH && mkdir -p logs && TS=$(date +%Y%m%d_%H%M%S) && sudo ethercat states PREOP >/dev/null && sudo env LD_LIBRARY_PATH="$LD_LIBRARY_PATH" RMD_ECAT_RT_CPU=11 RMD_ECAT_RT_PRIORITY=80 RMD_ECAT_APP_CPU=10 RMD_ECAT_APP_RT_PRIORITY=60 ./build-open-source-check/rmd_ecat_joint_path_demo config/configOriginal_heima.xml 10000 500 450 5 8000 right | tee "logs/rmd_ecat_joint_path_demo_${TS}.log"
```

2026-05-13 native `ec_igb` 60 秒验证日志：
`logs/rmd_ecat_position_targets_symmetric_deeper_20260513_200034.log`。
结果为 `RC=0`，13 个电机均为 `status=0x1237 err=0x0000`，
`interval_avg_ms=5`，`interval_max_ms=6.4128`，`overruns=0`。和基准姿态的
稳定末段相比，这组姿态让左膝 `motor10` 平均力矩从约 `52 Nm` 降到约
`28 Nm`，右膝 `motor4` 从约 `-1 Nm` 升到约 `21 Nm`，左髋 roll `motor7`
从约 `33 Nm` 降到约 `0.5 Nm`；站立负载更接近左右分担。

参数含义：

```text
300000  = 保持 5 分钟
500     = max_current
450     = 启动前 settle samples
5       = 应用层目标下发周期 5 ms
8000    = 8 秒线性 ramp 到目标姿态
```

退出后如需手动确认总线状态：

```bash
sudo ethercat states PREOP
sudo ethercat slaves
```

native `ec_igb + dc=true` 当前验证日志：

```text
logs/rmd_ecat_position_targets_i210_native_dc_shift0_short_hold_20260513_185439.log
logs/rmd_ecat_position_targets_i210_native_dc_shift0_5min_hold_20260513_185559.log
```

5 分钟 native 保持结果：13 个电机最终 `status=0x1237 err=0x0000`；marker 后没有新的
`0x001A` 或 PDO map error；`timing_summary period_ms=5 interval_samples=59999
interval_avg_ms=5 interval_max_ms=12.8902 overruns=1`。

## 9. 运行时环境变量

- `RMD_ECAT_RT_CPU`: 绑定 EtherCAT realtime 线程到指定 CPU。
- `RMD_ECAT_RT_PRIORITY`: 设置 SCHED_FIFO 优先级，默认 `80`。
- `RMD_ECAT_RT_MLOCK`: 是否调用 `mlockall`，默认启用。
- `RMD_ECAT_APP_CPU`: `rmd_ecat_position_targets` / `rmd_ecat_joint_targets`
  应用控制循环绑定 CPU。
- `RMD_ECAT_APP_RT_PRIORITY`: `rmd_ecat_position_targets` / `rmd_ecat_joint_targets`
  应用控制循环的
  SCHED_FIFO 优先级；不设置时保持普通调度。
- `RMD_ECAT_COMMAND_TIMEOUT_MS`: 命令超时自动 disable，默认 `100`。
- `RMD_ECAT_LOG_STATE`: 打印 master/domain 状态变化。
- `RMD_ECAT_DEBUG_RAW`: 打印前 N 帧原始 TxPDO。

Jetson Orin 当前有 12 个 online CPU，编号 `0..11`。长时间保持或控制循环测试时，
优先把 EtherCAT backend 和应用控制循环分到不同 CPU：backend 使用 CPU 11 /
SCHED_FIFO 80，应用循环使用 CPU 10 / SCHED_FIFO 60。backend 优先级必须高于
app，避免上层控制抢占 1 kHz EtherCAT 实时线程。

分核实时运行示例：

```bash
cd /home/eason/heimasdk-snapshot-migration && export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH && mkdir -p logs && sudo ethercat states PREOP >/dev/null && sudo env LD_LIBRARY_PATH="$LD_LIBRARY_PATH" RMD_ECAT_RT_CPU=11 RMD_ECAT_RT_PRIORITY=80 RMD_ECAT_APP_CPU=10 RMD_ECAT_APP_RT_PRIORITY=60 ./build-open-source-check/rmd_ecat_position_targets config/configOriginal_heima.xml 300000 500 450 5 8000 1=0 2=0 3=0.35 4=-0.49 5=0.49 6=0.41 7=0 8=0 9=0.35 10=-0.49 11=0.49 12=0.41 | tee logs/rmd_ecat_position_targets_split_cpu_rt_5min.log
```

## 10. 退出后恢复总线

测试结束后可以手动恢复到 PREOP：

```bash
sudo ethercat states PREOP
sudo ethercat slaves
```

如果某个测试异常退出，先恢复总线状态，再重新启动 SDK。
