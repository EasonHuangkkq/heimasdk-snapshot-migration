# 腿部连杆解算说明

本文记录当前 SDK 中右腿关节目标到电机目标的解算来源和已验证数值。代码入口是
`include/rmd_can_sdk/rmd_leg_kinematics.h`，实现是
`src/rmd_leg_kinematics.cpp`。该模块是纯 C++ 数学模块，不依赖 EtherCAT、CAN 或
`DriverSDK`。

## 1. 机构编号

右腿先落地验证：

```text
motor3 = J_hip_r_pitch，电机直接驱动
motor4 = J_knee_r_pitch 的膝盖拉杆电机
motor5 = 右脚踝 E 侧拉杆电机
motor6 = 右脚踝 F 侧拉杆电机
```

左腿对应 `motor9..12`，但左腿符号方向暂时不在这个模块里猜测；必须用实测数据确认后再加。

## 2. 公式来源

膝盖解算来自 `examples/knee_visualizer`。核心是两圆交点：膝关节角给出摇臂端点
`K`，电机曲柄端点 `E` 必须满足 `|E-K| = 215 mm`。存在两个数学分支时，选择最接近
上一帧电机角或关节角的分支。

脚踝解算来自 `examples/ankle_visualizer`。给定 ankle pitch/roll 后，先旋转平台上的
`C`、`D` 点，再用一维 Newton 分别求 `thetaE`、`thetaF`，使两根拉杆长度满足
`CE = 317.837 mm` 和 `DF = 236.273 mm`。从电机角反解 pitch/roll 时，用二维 Newton。

## 3. 数值 checkpoint

单元测试固定以下 visualizer 对齐点：

```text
motor3 = 0.35 -> J_hip_r_pitch = 0.35
motor4 = -0.49 -> J_knee_r_pitch = -0.696303743 rad
J_knee_r_pitch = -0.70 -> motor4 = -0.493656687 rad
J_ankle_r_pitch = 0.35, J_ankle_r_roll = 0 -> motor5 = 0.476713266, motor6 = 0.404559301
motor5 = 0.49, motor6 = 0.41 -> J_ankle_r_pitch = 0.357188577, J_ankle_r_roll = 0.005450772
```

因此之前的 motor-space 姿态 `3=0.35 4=-0.49 5=0.49 6=0.41` 约等于：

```text
J_hip_r_pitch = 0.35
J_knee_r_pitch = -0.6963
J_ankle_r_pitch = 0.3572
J_ankle_r_roll = 0.00545
```

如果 joint-space 测试命令指定严格的 `ankle_pitch=0.35 ankle_roll=0`，解析出的脚踝电机
目标会是 `motor5=0.4767 motor6=0.4046`，不会等于旧的 `0.49/0.41`。

## 4. 右腿路径 demo

`rmd_ecat_joint_path_demo` 是一个最小 EtherCAT 路径控制 demo。它不做复杂规划，只在
关节空间里定义三段 waypoint，再按应用层周期线性插值：

```text
0 ms:      hip=0.35, knee=-0.70, ankle_pitch=0.35, ankle_roll=0
duration/2 hip=0.37, knee=-0.68, ankle_pitch=0.33, ankle_roll=0.02
duration: hip=0.35, knee=-0.70, ankle_pitch=0.35, ankle_roll=0
```

每个周期的流程是：

```text
sample joint path -> solve motor3/4/5/6 -> check XML limits -> setMotorTarget()
```

程序启动后先从当前位置 ramp 到第一个 waypoint，然后才开始按路径运动。其他 active
motors 保持启动时当前位置。无硬件检查命令：

```bash
./build-open-source-check/rmd_ecat_joint_path_demo --audit-self-test
```
