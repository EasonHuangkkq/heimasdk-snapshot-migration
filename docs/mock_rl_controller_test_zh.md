# Mock RL 控制闭环测试流程

本文记录 `rmd_ecat_mock_controller` 的现场测试流程，用来验证：

```text
EtherCAT actual + IMU
-> RL observation
-> mock RL action
-> joint-space target
-> 膝盖/脚踝机构解算
-> motor target
-> EtherCAT setMotorTarget()
-> 电机实际反馈
```

当前流程使用 `config/configOriginal_heima.xml`。物理 slave `0` 和 `13` 是 EV1 分支器，不是电机；MT_Device 在物理 slave `1..12` 和 `14`。配置里的电机 alias 是 `1..13`，其中 alias `13` 对应物理 slave `14`。

## 安全前提

- 机器人已吊起或有人准备急停。
- 只允许一个进程占用 EtherCAT master。
- IMU 已插入并能被 root 打开。
- 默认动作只控制双腿 `3/4/5/6` 和 `9/10/11/12`；`1/2/7/8/13` 保持当前位置。
- 脚踝电机 `5/11` 上限较紧，mock RL 保持 `ankle_pitch=0.35`，不要随意加大脚踝幅度。

## 构建和本地检查

```bash
cmake --build build-open-source-check --target rmd_ecat_mock_controller -j2
ctest --test-dir build-open-source-check -R 'rmd_ecat_mock_controller_audit_mock_rl_policy|rmd_ecat_mock_controller_audit_rl_observation|rmd_ecat_mock_controller_rejects' --output-on-failure
./build-open-source-check/rmd_ecat_mock_controller --audit-mock-rl-policy
```

成功时应看到：

```text
mock rl policy audit passed
```

当前 mock action：

```text
hip_pitch   = 0.35 +/- 0.10 rad
knee_pitch  = -0.70 +/- 0.22 rad
ankle_pitch = 0.35 rad
ankle_roll  = 0
```

## 上电前预检

```bash
pgrep -af 'rmd_ecat|yesense|ethercat' || true
sudo ethercat master
sudo ethercat slaves
```

预期：

- 没有其他 `rmd_ecat_*` 程序正在运行。
- master 为 `Idle`，link 为 `UP`。
- `Tx errors` 为 `0`。
- 15 个 slave 可见，EV1 分支器在 slave `0` 和 `13`，MT_Device 在其余位置。

可选 IMU 检查：

```bash
sudo env LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH ./build-open-source-check/yesense_imu_test /dev/ttyACM0 921600 20
```

## 16 秒明显动作测试

用于肉眼确认机器人在动。前 2 秒从当前位置 ramp 到 mock RL 起始动作，之后按 100 Hz policy 输出动作。

```bash
cd /home/eason/heimasdk-snapshot-migration && export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH && mkdir -p logs && TS=$(date +%Y%m%d_%H%M%S) && LOG="logs/rmd_ecat_mock_rl_policy_visible_16s_${TS}.log" && sudo ethercat states PREOP >/dev/null && sudo env LD_LIBRARY_PATH="$LD_LIBRARY_PATH" RMD_ECAT_RT_CPU=11 RMD_ECAT_RT_PRIORITY=80 ./build-open-source-check/rmd_ecat_mock_controller config/configOriginal_heima.xml 16000 100 500 450 2000 60 --mock-rl-joint-policy >"$LOG" 2>&1; RC=$?; echo "RC=$RC"; echo "LOG=$LOG"; tail -n 80 "$LOG"; test $RC -eq 0
```

成功判据：

```text
RC=0
feedback_misses=0
overruns=0
imu_valid_samples=1600
status=0x1237
err=0x0000
```

日志中应看到 action 和 observation 跟随，例如：

```text
mock_rl_action ... hip=0.45 knee=-0.48
rl_obs         ... hip≈0.41 knee≈-0.60

mock_rl_action ... hip=0.25 knee=-0.92
rl_obs         ... hip≈0.29 knee≈-0.81
```

## 2 分钟稳定性测试

确认 100 Hz 上层循环、IMU、EtherCAT feedback、解算和电机跟随能持续运行。

```bash
cd /home/eason/heimasdk-snapshot-migration && export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH && mkdir -p logs && TS=$(date +%Y%m%d_%H%M%S) && LOG="logs/rmd_ecat_mock_rl_policy_visible_120s_${TS}.log" && sudo ethercat states PREOP >/dev/null && sudo env LD_LIBRARY_PATH="$LD_LIBRARY_PATH" RMD_ECAT_RT_CPU=11 RMD_ECAT_RT_PRIORITY=80 ./build-open-source-check/rmd_ecat_mock_controller config/configOriginal_heima.xml 120000 100 500 450 2000 60 --mock-rl-joint-policy >"$LOG" 2>&1; RC=$?; echo "RC=$RC"; echo "LOG=$LOG"; tail -n 120 "$LOG"; test $RC -eq 0
```

最近一次通过结果：

```text
policy_hz=100
iterations=12000
interval_avg_ms=10.000001
interval_max_ms=10.041987
overruns=0
feedback_misses=0
imu_valid_samples=12000
imu_zero_after_first_valid=0
```

## 测后收尾检查

```bash
sudo ethercat master
sudo ethercat slaves
ctest --test-dir build-open-source-check --output-on-failure
```

预期：

- master 回到 `Idle`。
- 所有 slave 回到 `PREOP`。
- `Tx errors` 仍为 `0`。
- `ctest` 全部通过。

## 日志分析

查看关键结果：

```bash
LOG=logs/rmd_ecat_mock_rl_policy_visible_120s_YYYYMMDD_HHMMSS.log
rg -n 'mock_controller_summary|mock_final|feedback_miss|failed|outside configured|IMU never|final motor feedback' "$LOG"
du -h "$LOG"
```

成功日志应包含：

```text
mock_controller_summary ... overruns=0 ... feedback_misses=0 ...
mock_final ... status=0x1237 err=0x0000 ...
```

重点看温度：

- `drive_temp` 短测到 50 多度正常。
- 如果 10 分钟以上测试，重点盯脚踝 `11` 号和 `12` 号 drive temp。
- 温度持续快速上升时应停止测试，降低动作幅度或缩短时长。

## 异常处理

- `outside configured position limits`：mock action 解算后超过 XML 限位。不要绕过，先减小对应 joint 幅度。
- `feedback_misses > 0`：检查 EtherCAT 状态、WKC、线缆和 backend 状态。
- `imu_valid_samples=0`：确认 `/dev/ttyACM0`、波特率 `921600`、权限和接线。
- `status` 不是 `0x1237` 或 `err` 非 0：先停机，查 402 enable 状态和电机错误码。
- 程序未退出或 master 未回 Idle：先确认进程，再手动 `ethercat states PREOP`。
