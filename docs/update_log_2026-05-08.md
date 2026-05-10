# rmd_can_sdk 更新记录 - 2026-05-08

本次更新把 `rmd_can_sdk` 从基础 RMD CAN/MIT demo 扩展为面向 RMD V4.4、MIT 高频控制、position-based RL 实测和 Orin 现场部署的工具链。

## 1. RMD V4.4 协议适配

- MIT 位置范围更新为 `[-12.566, 12.566] rad`。
- MIT `kd` 范围更新为 `0..50`。
- MIT 前馈力矩 `t_ff` 改为按每个电机 XML 中的 `MaximumTorque` 动态缩放。
- 新增 `0x9A` 状态1解析：温度、MOS 温度、抱闸释放命令状态、电压和错误码。
- 新增 `0x92` 多圈角度解析。
- 保留 `0x9C` 状态2解析，用于电流、估算输出力矩和温度等辅助诊断。

## 2. DriverSDK 反馈语义调整

- `actual.pos` 和 `actual.vel` 改为来自 MIT 高频反馈。
- `0x9C` 不再覆盖 `actual.pos/actual.vel`，只更新 `actual.tor` 和 `actual.temp`。
- `actual.tor` 使用 `0x9C` 的 `iq_a * TorqueConstant` 估算。
- 该调整避免低精度整数角度反馈污染 position-based RL observation。

## 3. 硬件诊断和实验工具

- 新增 `rmd_a4_response_test`：诊断标准 `0xA4` 位置命令响应，输出运动开始、到达目标和采样 CSV。
- 增强 `rmd_mit_sweep_test`：默认跳过 A4 回零，使用 MIT 预定位和正负扫动；支持命令角度、评价角度、`kp/kd`、容差、ramp 和敏感运动阈值。
- 新增 `rmd_mit_sine_test`：单电机 MIT 正弦位置轨迹测试，输出误差、速度、力矩和采样周期统计。
- 新增 `rmd_mit_dual_sine_test`：双电机 MIT 正弦测试，支持相对当前位置和绝对中心点模式，并用 `0x92/0x9A` 做硬安全监控。
- 新增 `rmd_mit_sine_autotune_test`：自动扫描 `kp/kd/command_gain` 候选组合，按误差、力矩和采样抖动评分。
- 新增 `rmd_mit_torque_probe`：测试 MIT 前馈力矩 `t_ff` 的实际效果，输出命令字节、样本和汇总。

## 4. 轨迹和安全限位模块

- 新增 `rmd_motion_plan.h/.cpp`。
- 支持正弦目标生成。
- 支持按 XML 中的 `MinimumPosition`、`MaximumPosition` 和安全 margin 检查目标轨迹。
- 双电机测试在启动前和运行中使用该模块做目标范围和标准角度安全检查。

## 5. Orin/Jetson 现场部署支持

- 新增 `scripts/build_on_target.sh`：检测并清理错误 CPU 架构的 `build/`，然后在当前机器重新 CMake、编译和 CTest。
- 新增 `scripts/ensure_target_binary.sh`：运行脚本前按需构建指定 CMake target。
- 新增 `scripts/setup_orin_kcan_names.sh`：把 KH/PEAK USB-CAN 固定命名为 `kcan1..kcan6`。
- 文档新增 Orin + KH/PEAK USB-CAN 实测流程和避坑清单。

## 6. A4 和前端状态语义修正

- `rmd_move_zero` 的 A4 最大速度限制从 `30 dps` 放宽到 `100 dps`。
- `0x9A DATA[3]` 明确为抱闸释放命令状态，不再作为独立机械抱闸传感器硬拦截。
- Web 前端将状态标签从 `Brake` 改为 `Brake Cmd`，与协议语义保持一致。
- Python 协议解析和运动 gate 逻辑同步更新，只在 `error_state != 0` 时阻止运动。

## 7. 测试覆盖

- 新增 RMD V4.4 MIT 编解码范围测试。
- 新增 `0x9A` 错误码和抱闸命令状态解析测试。
- 新增 `0x92` 多圈角度解析测试。
- 新增 `0x9C` 不覆盖 MIT observation 的测试。
- 新增正弦轨迹限位检查测试。
- 新增前端 `Brake Cmd` 标签和 Python motion gate 语义测试。
