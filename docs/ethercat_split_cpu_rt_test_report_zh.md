# EtherCAT 分核实时测试流程与报告

本文记录 2026-05-13 对 EtherCAT native `ec_igb` 链路、应用层 5 ms 控制循环、
backend 1 kHz 实时线程的排查流程和验证结果。测试目标是确认此前应用层偶发
`>5 ms` 周期抖动的来源，并验证 backend 和 app loop 分别绑 CPU、分别设置
`SCHED_FIFO` 后的实时性。

## 1. 测试背景

当前硬件为 Jetson Orin，online CPU 为 `0..11`。EtherCAT backend 已经使用
native `ec_igb`，电机配置为 `config/configOriginal_heima.xml`，物理链路包含
13 个 MT_Device 电机和 2 个 EV1 分支器。控制姿态使用更深对称站立姿态：

```text
motor1=0, motor2=0
motor3=0.35, motor4=-0.49, motor5=0.49, motor6=0.41
motor7=0, motor8=0
motor9=0.35, motor10=-0.49, motor11=0.49, motor12=0.41
motor13 保持当前位置
```

排查前现象：EtherCAT backend 指标显示没有 deadline miss，但应用层
`rmd_ecat_position_targets` 偶发出现 5 ms 周期迟到。需要确认迟到来自
EtherCAT 链路、应用代码耗时，还是 Linux 普通线程调度。

## 2. 代码与诊断能力

本轮新增和使用的诊断能力：

- `RMD_ECAT_BACKEND_STATUS_CSV`: 每 1 秒采样 backend 运行状态，包括
  `deadline_miss_count`、`max_cycle_ns`、`max_wakeup_latency_ns`、
  `wc_incomplete_count`、`stale_frame_count`。
- `RMD_ECAT_APP_TIMING_CSV`: 在 app 控制循环内只记录内存行，退出并
  `disableMotors()` 后再落盘 CSV，避免实时周期内写文件。
- app timing 字段包含 `interval_ms`、`target_prep_ms`、`set_target_ms`、
  `get_actual_ms`、`print_actuals_ms`、`backend_status_ms`、`active_ms`、
  `overrun_ms`。
- `RMD_ECAT_APP_CPU` 和 `RMD_ECAT_APP_RT_PRIORITY`: 为
  `rmd_ecat_position_targets` 应用控制循环设置 CPU affinity 和 `SCHED_FIFO`。
- backend 原有环境变量继续使用：`RMD_ECAT_RT_CPU`、
  `RMD_ECAT_RT_PRIORITY`。

关键实现位置：

```text
include/rmd_can_sdk/rmd_bench_workflow.h
src/rmd_bench_workflow.cpp
examples/rmd_ecat_position_targets.cpp
tests/rmd_can_sdk_tests.cpp
docs/ethercat_quickstart_zh.md
```

## 3. 测试流程

### 3.1 构建与单测

```bash
cmake --build build-open-source-check --target rmd_can_sdk_tests rmd_ecat_position_targets -j2
./build-open-source-check/rmd_can_sdk_tests
```

结果：构建通过，`rmd_can_sdk_tests passed`。非法环境变量也已验证：
`RMD_ECAT_APP_CPU=cpu10` 会在启动前拒绝，返回 `2`。

### 3.2 10 分钟基线诊断

目的：在原始 app 普通调度模型下，同时采集 backend status 和 app timing。

命令结构：

```bash
sudo ethercat states PREOP
sudo env LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH \
  RMD_ECAT_BACKEND_STATUS_CSV=logs/rmd_ecat_backend_status_deeper_zero_hip_10min_appdiag_20260513_210430.csv \
  RMD_ECAT_APP_TIMING_CSV=logs/rmd_ecat_app_timing_deeper_zero_hip_10min_20260513_210430.csv \
  ./build-open-source-check/rmd_ecat_position_targets \
  config/configOriginal_heima.xml 600000 500 450 5 8000 \
  1=0 2=0 3=0.35 4=-0.49 5=0.49 6=0.41 \
  7=0 8=0 9=0.35 10=-0.49 11=0.49 12=0.41
```

### 3.3 stdout 丢弃对照

目的：排除主日志写文件和终端输出对 app 周期的影响。

```bash
sudo env LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH \
  RMD_ECAT_BACKEND_STATUS_CSV=logs/rmd_ecat_backend_status_deeper_zero_hip_5min_devnull_20260513_211659.csv \
  RMD_ECAT_APP_TIMING_CSV=logs/rmd_ecat_app_timing_deeper_zero_hip_5min_devnull_20260513_211659.csv \
  ./build-open-source-check/rmd_ecat_position_targets \
  config/configOriginal_heima.xml 300000 500 450 5 8000 \
  1=0 2=0 3=0.35 4=-0.49 5=0.49 6=0.41 \
  7=0 8=0 9=0.35 10=-0.49 11=0.49 12=0.41 \
  >/dev/null
```

### 3.4 外部 `chrt` 对照

目的：验证应用迟到是否来自普通 Linux 调度晚唤醒。

```bash
sudo env LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH \
  RMD_ECAT_BACKEND_STATUS_CSV=logs/rmd_ecat_backend_status_deeper_zero_hip_5min_chrt60_20260513_212311.csv \
  RMD_ECAT_APP_TIMING_CSV=logs/rmd_ecat_app_timing_deeper_zero_hip_5min_chrt60_20260513_212311.csv \
  chrt -f 60 ./build-open-source-check/rmd_ecat_position_targets \
  config/configOriginal_heima.xml 300000 500 450 5 8000 \
  1=0 2=0 3=0.35 4=-0.49 5=0.49 6=0.41 \
  7=0 8=0 9=0.35 10=-0.49 11=0.49 12=0.41 \
  >/dev/null
```

### 3.5 最终分核实时验证

目的：验证最终实现，不依赖外部 `chrt`，由程序自己设置 app 主循环实时属性。

推荐分配：

```text
EtherCAT backend: CPU 11, SCHED_FIFO 80
app 控制循环:     CPU 10, SCHED_FIFO 60
```

```bash
sudo ethercat states PREOP
sudo env LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH \
  RMD_ECAT_RT_CPU=11 RMD_ECAT_RT_PRIORITY=80 \
  RMD_ECAT_APP_CPU=10 RMD_ECAT_APP_RT_PRIORITY=60 \
  RMD_ECAT_BACKEND_STATUS_CSV=logs/rmd_ecat_backend_status_split_cpu_rt_5min_20260513_213620.csv \
  RMD_ECAT_APP_TIMING_CSV=logs/rmd_ecat_app_timing_split_cpu_rt_5min_20260513_213620.csv \
  ./build-open-source-check/rmd_ecat_position_targets \
  config/configOriginal_heima.xml 300000 500 450 5 8000 \
  1=0 2=0 3=0.35 4=-0.49 5=0.49 6=0.41 \
  7=0 8=0 9=0.35 10=-0.49 11=0.49 12=0.41 \
  >/dev/null
```

## 4. 测试结果

### 4.1 app timing 汇总

| 测试 | 时长 | 调度方式 | app overruns | max interval ms | max active ms | 说明 |
| --- | ---: | --- | ---: | ---: | ---: | --- |
| 基线 appdiag | 10 min | 普通调度，写主日志 | 0 | 8.14844 | 3.36243 | `printActuals()` 最大 3.33715 ms |
| devnull | 5 min | 普通调度，stdout 丢弃 | 1 | 12.5143 | 0.937258 | overrun 行 `active_ms=0.043357` |
| chrt60 | 5 min | 外部 `chrt -f 60` | 0 | 5.0684 | 0.730392 | 普通调度迟到消失 |
| split CPU RT | 5 min | app CPU 10/FIFO 60, backend CPU 11/FIFO 80 | 0 | 5.05429 | 0.710939 | 最终方案验证通过 |

`devnull` 测试的关键 overrun 行：

```text
sample=16342
interval_ms=12.5143
active_ms=0.043357
overrun_ms=2.61655
monitored=0
backend_status_sampled=0
```

该行没有 `getActual()`、没有 `printActuals()`、没有 backend status 采样，说明这次
app overrun 不是业务代码执行太慢，而是普通调度线程被系统晚唤醒。

### 4.2 backend status 汇总

| 测试 | cycle delta | deadline delta | stale delta | WKC delta | max cycle ns | max wakeup latency ns |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 基线 appdiag 10 min | 599995 | 0 | 0 | 0 | 164952 | 82215 |
| devnull 5 min | 299995 | 0 | 0 | 0 | 164470 | 89627 |
| chrt60 5 min | 299995 | 0 | 0 | 0 | 192149 | 61643 |
| split CPU RT 5 min | 299995 | 0 | 0 | 0 | 177398 | 55254 |

所有测试中 EtherCAT backend 均无 deadline miss、无新增 WKC incomplete、无新增 stale
frame、无 RX timeout。主站 `Lost frames` 计数没有在测试中继续增长。

### 4.3 线程状态确认

分核实时短测中，用 `ps -eLo pid,tid,cls,rtprio,psr,comm,args` 直接确认：

```text
app main thread: FF priority 60 CPU 10
rmd_ecat_rt:     FF priority 80 CPU 11
```

另有一个普通 `TS` 辅助线程，不在 5 ms app 控制循环和 1 kHz EtherCAT backend
关键路径上。

## 5. 结论

1. 当前 native `ec_igb + DC` EtherCAT backend 链路是通的，1 kHz backend 线程在这些测试中没有 deadline miss。
2. 之前看到的 app 5 ms 偶发迟到，根因不是 PDO/DC/native igb，而是应用主循环普通 Linux 调度晚唤醒。
3. `printActuals()` 很重，写完整主日志时最大约 3.34 ms；长时间测试不应该把大量状态打印作为控制周期内的常规动作。
4. app 控制循环使用 `SCHED_FIFO 60` 后，普通调度导致的 10 ms 级晚唤醒消失。
5. 最终推荐模型是 backend 和 app 分核、分优先级：
   - backend: CPU 11, `SCHED_FIFO 80`
   - app: CPU 10, `SCHED_FIFO 60`
   - backend 优先级必须高于 app，避免上层控制抢占 EtherCAT 1 kHz 实时线程。

## 6. 后续使用建议

- 后续真实 RL 或上层控制循环应继承这个模型：backend 高优先级独立 CPU，app 控制循环次高优先级独立 CPU，日志线程保持普通优先级。
- 控制周期内只做 target 计算、`setMotorTarget()`、必要的 snapshot 读取；大日志、CSV、分析统计应放到非实时路径或退出后落盘。
- 长测时建议继续保留 `RMD_ECAT_BACKEND_STATUS_CSV` 和
  `RMD_ECAT_APP_TIMING_CSV`，方便对 app 抖动和 backend 抖动分开定位。
- 如果以后引入 IMU 或 RL policy 线程，建议先给它们低于 app loop 的优先级，确认不会抢占 CPU 10/11 上的两个关键实时线程。
