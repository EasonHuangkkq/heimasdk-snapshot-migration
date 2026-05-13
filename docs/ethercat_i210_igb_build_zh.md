# Intel I210 native ec_igb 编译与切换记录

这份记录写的是 2026-05-13 在当前 Orin 机器上实际做过的一次流程：把 IgH
EtherCAT master 从 `generic` 路径扩展编译出 Intel I210 对应的 native
`ec_igb` 模块，切到 native 后做只读验证，并记录最初失败、最小化复测收敛和
最终把当前机器配置成开机默认由 `ec_igb` 接管 I210。`generic` 仍保留为可回退基线。

## 1. 最终结论

- 编译成功：`ec_igb.ko` 已经从 `/home/eason/Eason/ethercat` 编译并安装到
  `/lib/modules/5.15.148-rt-tegra/ethercat/devices/igb/ec_igb.ko`。
- native 接管成功：PCI 设备 `0005:01:00.0` 可以从 Linux 内置 `igb` 解绑后绑定到
  `/sys/bus/pci/drivers/ec_igb`，`ethercat master` 能看到 15 个从站。
- 重新诊断结论：`ec_igb` 编译和基本链路不是问题；native 不稳定主要来自 SDK
  初始化策略和当前 DC 配置。关闭 DC 只能作为隔离变量，不是最终控制方案。
- 目标路径仍然是 native + DC。当前 SDK 在 DC 打开时会显式选择第一个
  MT_Device slave 作为 reference clock，避免 IgH 默认把 slave 0 的 EV1 分支器
  选成参考时钟。
- DC 启动不能只靠 probe 最后的 10 帧判定。SDK start 现在会等待所有配置电机连续
  20 个实时帧 fresh，默认最多等待 30 秒；stop 时先降到 SAFEOP/PREOP，再停止实时
  线程，避免 release master 时触发同步错误。
- 当前机器默认启动路径已经切到 native：
  `DEVICE_MODULES="igb"`，`UPDOWN_INTERFACES=""`，并通过 systemd 在开机时自动把
  I210 从 Linux `igb` 绑定到 `ec_igb`。

## 2. 环境与路径

当前 SDK repo：

```bash
/home/eason/heimasdk-snapshot-migration
```

IgH/EtherCAT 源码：

```bash
/home/eason/Eason/ethercat
```

内核版本：

```bash
uname -r
# 5.15.148-rt-tegra
```

内核头路径：

```bash
readlink -f /lib/modules/$(uname -r)/build
# /usr/src/linux-headers-5.15.148-rt-tegra-ubuntu22.04_aarch64/3rdparty/canonical/linux-jammy/kernel-source
```

I210 网卡：

```text
PCI device: 0005:01:00.0
Interface:  enP5p1s0
MAC:        <I210_MAC>
Linux driver before native switch: igb
```

## 3. 编译前确认

之前的 EtherCAT master 是 `generic` 编译：

```text
ENABLE_GENERIC=1
ENABLE_IGB=0
```

已有模块只有：

```text
ec_master.ko
ec_generic.ko
```

没有：

```text
ec_igb.ko
```

这说明如果只继续用 `generic`，不需要重新编译；如果要使用 I210 native
driver，就必须重新 configure 并启用 `--enable-igb`。

## 4. configure

在 EtherCAT 源码目录执行：

```bash
cd /home/eason/Eason/ethercat
./configure --prefix=/usr/local --sysconfdir=/etc --with-linux-dir=/lib/modules/$(uname -r)/build --enable-generic --enable-igb
```

这次 configure 成功，关键点是：

```text
Linux kernel sources: /lib/modules/5.15.148-rt-tegra/build
Kernel family:        5.15
igb driver support:   enabled
```

这里保留 `--enable-generic` 是有意的：这样 native 路径不通时，可以继续退回
`generic`，不用再重新编译一次。

## 5. 编译用户态与内核模块

执行：

```bash
make -j$(nproc)
make modules -j$(nproc)
```

`make modules` 过程中可以看到 `devices/igb/` 下的文件参与编译，并最终生成：

```text
/home/eason/Eason/ethercat/devices/igb/ec_igb.ko
```

验证编译产物：

```bash
modinfo /home/eason/Eason/ethercat/devices/igb/ec_igb.ko
```

这次看到的关键信息：

```text
description: Intel(R) Gigabit Ethernet Network Driver
depends:     ec_master
vermagic:    5.15.148-rt-tegra SMP preempt_rt mod_unload modversions aarch64
```

`vermagic` 和当前内核一致，说明模块是给当前 `5.15.148-rt-tegra` 编的。

## 6. 安装模块

先停掉当前 EtherCAT service，避免安装和切换时 master 正在占用设备：

```bash
sudo systemctl stop ethercat.service
```

安装模块：

```bash
cd /home/eason/Eason/ethercat
sudo make modules_install
sudo depmod -a $(uname -r)
```

安装后确认：

```bash
modinfo ec_igb
```

这次解析到的安装位置：

```text
/lib/modules/5.15.148-rt-tegra/ethercat/devices/igb/ec_igb.ko
```

`modules_install` 曾提示缺少 `System.map`，但随后 `depmod -a` 正常完成。这个提示不等于
`ec_igb` 编译失败。

## 7. 切换 /etc/ethercat.conf 到 native igb

切换前的可用 `generic` 配置是：

```bash
MASTER0_DEVICE="<I210_MAC>"
DEVICE_MODULES="generic"
UPDOWN_INTERFACES="enP5p1s0"
```

切到 native `igb` 时，先备份：

```bash
sudo cp /etc/ethercat.conf /etc/ethercat.conf.bak_i210_generic_$(date +%Y%m%d_%H%M%S)
```

然后把配置改成：

```bash
MASTER0_DEVICE="<I210_MAC>"
DEVICE_MODULES="igb"
UPDOWN_INTERFACES=""
```

注意：`native igb` 路径下，不再让 EtherCAT service 通过普通 Linux 网络接口
`enP5p1s0` 的 up/down 流程接管设备，所以 `UPDOWN_INTERFACES` 置空。

启动：

```bash
sudo systemctl start ethercat.service
```

第一次启动后，`ec_igb` 已加载，但 master 还在等待设备：

```text
Phase: Waiting for device(s)...
Main: <I210_MAC> (waiting...)
```

原因是这台机器上的普通 Linux `igb` 是 built-in，不是可卸载模块：

```bash
modinfo igb
# filename: (builtin)
```

所以 service 不能通过 `rmmod igb` 自动释放 I210。

## 8. 手动从 Linux igb 解绑并绑定到 ec_igb

确认当前 PCI 设备还在普通 `igb` 下：

```bash
readlink -f /sys/class/net/enP5p1s0/device/driver
# /sys/bus/pci/drivers/igb
```

native 切换顺序很重要：

1. 先让 `ethercatctl start` 加载 `ec_master main_devices=<MAC>`，让 master 处于
   waiting-for-device 状态。
2. 再把 PCI 设备从普通 `igb` unbind，并 bind 到 `ec_igb`。

不要在 `ethercatctl start` 之前手动 `modprobe ec_igb`。`ec_igb` 依赖
`ec_master`，提前加载会让 `ec_master` 不带 `main_devices` 参数启动，dmesg 会显示
`0 masters waiting for devices`，随后不会创建可用的 `/dev/EtherCAT0`。

手动切换绑定：

```bash
sudo sh -c 'echo 0005:01:00.0 > /sys/bus/pci/drivers/igb/unbind; echo 0005:01:00.0 > /sys/bus/pci/drivers/ec_igb/bind'
```

确认绑定结果：

```bash
readlink -f /sys/bus/pci/devices/0005:01:00.0/driver
# /sys/bus/pci/drivers/ec_igb
```

此时 `ethercat master` 应该能看到链路和从站：

```text
driver=ec_igb
main_devices=<I210_MAC>
ethercat master: Link UP, Slaves 15
```

```text
Link: UP
Slaves: 15
Lost frames: 0
```

`dmesg` 里也能看到：

```text
Accepting <I210_MAC> as main device for master 0.
Link state of ecm0 changed to UP
15 slave(s) responding
```

## 9. 开机默认 native 接管 I210

当前机器已经不再依赖每次手动 sysfs bind。开机时先启动 `ethercat.service`，
由 `ethercatctl start` 加载带 `main_devices=<I210_MAC>` 的 `ec_master` 和
`ec_igb`；随后 `ethercat-i210-native-bind.service` 把 PCI 设备 `0005:01:00.0`
从普通 Linux `igb` 解绑并绑定到 `ec_igb`。

当前 `/etc/ethercat.conf`：

```bash
MASTER0_DEVICE="<I210_MAC>"
DEVICE_MODULES="igb"
UPDOWN_INTERFACES=""
```

已安装的本机 systemd/script 文件：

```text
/usr/local/sbin/ethercat-i210-native-clean
/usr/local/sbin/ethercat-i210-native-bind
/etc/systemd/system/ethercat.service.d/40-native-clean.conf
/etc/systemd/system/ethercat.service.d/50-native.conf
/etc/systemd/system/ethercat-i210-native-bind.service
```

`40-native-clean.conf` 的作用是清理从 `generic` 切到 native 时可能残留的
`ec_generic`、`ec_igb`、`ec_master`，并把 PCI 设备临时绑回 Linux `igb`，让
`ethercatctl start` 每次都从干净状态启动。这个步骤不启动电机，只准备 EtherCAT
master 和网卡接管。

确认开机默认 native 的命令：

```bash
systemctl is-enabled ethercat.service ethercat-i210-native-bind.service
systemctl status ethercat.service ethercat-i210-native-bind.service --no-pager
grep -E '^(MASTER0_DEVICE|DEVICE_MODULES|UPDOWN_INTERFACES)=' /etc/ethercat.conf
readlink -f /sys/bus/pci/devices/0005:01:00.0/driver
lsmod | grep -E '^(ec_|igb)'
sudo ethercat master
sudo ethercat slaves
```

期望状态：

```text
ethercat.service: enabled
ethercat-i210-native-bind.service: enabled
DEVICE_MODULES="igb"
UPDOWN_INTERFACES=""
/sys/bus/pci/drivers/ec_igb
ec_master 只被 ec_igb 使用，没有 ec_generic
Link UP, Slaves 15
```

如果要临时回退到 `generic`：

```bash
sudo systemctl disable --now ethercat-i210-native-bind.service
sudo rm -f /etc/systemd/system/ethercat.service.d/40-native-clean.conf /etc/systemd/system/ethercat.service.d/50-native.conf
sudo sed -i -E 's/^DEVICE_MODULES=.*/DEVICE_MODULES="generic"/; s/^UPDOWN_INTERFACES=.*/UPDOWN_INTERFACES="enP5p1s0"/' /etc/ethercat.conf
sudo systemctl daemon-reload
sudo systemctl restart ethercat.service
```

回退后用 `readlink -f /sys/class/net/enP5p1s0/device/driver` 确认 I210 回到 Linux
`igb`，再用 `ethercat master` / `ethercat slaves` 确认总线。

## 10. native ec_igb 只读 SDK 验证

只跑 read-only snapshot probe，不跑电机 enable：

```bash
cd /home/eason/heimasdk-snapshot-migration
env LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH ./build-open-source-check/rmd_ecat_snapshot_probe config/configOriginal_heima.xml 1500 8
```

第一次日志：

```text
logs/rmd_ecat_snapshot_probe_i210_ec_igb_20260513_173154.log
```

结果：

```text
exit_status=1
motor 1: status=0xffff err=0x0001
motor 2..13: 大部分能读到 status=0x0231 err=0x0000
```

内核日志关键错误：

```text
Failed to map PDO entry 0x6071:00 (16 bit) to position 4.
Failed to configure mapping of PDO 0x1600.
Failed to set SAFEOP state, slave refused state change (PREOP + ERROR).
AL status message 0x001D: "Invalid output configuration".
```

随后把总线拉回 INIT/PREOP：

```bash
sudo ethercat states INIT
sleep 2
sudo ethercat states PREOP
sleep 2
sudo ethercat slaves
```

第二次 retry 日志：

```text
logs/rmd_ecat_snapshot_probe_i210_ec_igb_retry_20260513_173432.log
```

结果仍然失败：

```text
exit_status=1
motor 1: status=0xffff err=0x0001
```

第二次内核日志更明确地显示 CoE/PDO 配置超时：

```text
Failed to clear PDO mapping.
Reception of CoE download request for SDO 0x1c12:0 failed with timeout after 1000 ms: No response.
Failed to clear PDO assignment of SM2.
Reception of CoE download request for SDO 0x1a02:0 failed with timeout after 1000 ms: No response.
Failed to configure mapping of PDO 0x1A02.
AL status message 0x001D: "Invalid output configuration".
```

这个失败后来没有稳定复现。进一步最小化排查发现：必须用 root 权限访问
`/dev/EtherCAT0`，并且 native 切换后先做一次干净的 INIT/PREOP 状态恢复，再跑
SDK probe。有效复测结果如下：

```text
logs/rmd_ecat_snapshot_probe_i210_native_alias1_sudo_20260513_175524.log
logs/rmd_ecat_snapshot_probe_i210_native_alias2_sudo_20260513_175524.log
logs/rmd_ecat_snapshot_probe_i210_native_alias1_2_sudo_20260513_175747.log
logs/rmd_ecat_snapshot_probe_i210_native_full13_sudo_20260513_175747.log
```

复测结论：

```text
alias1 rc=0
alias2 rc=0
alias1+alias2 rc=0
full13 rc=0
```

完整 13 电机 native read-only probe 末尾 13 个电机都是：

```text
status=0x0231 err=0x0000
```

随后尝试进入 5 分钟保持前，又先跑了一次 read-only precheck：

```text
logs/rmd_ecat_snapshot_probe_i210_native_prehold_20260513_180228.log
```

这次 precheck 失败：

```text
Failed to execute SDO download: Input/output error
EtherCAT slave 5 SDO download clear RxPDO assignment 0x1c12:00 failed, abortCode=0x00000000
rmd_can_sdk init failed: starting motor backend failed
probe failed: starting motor backend failed
```

dmesg 里对应：

```text
EtherCAT ERROR 0-5: Reception of CoE download response failed: No response.
EtherCAT ERROR 0-5: Failed to process SDO request.
```

因此，最初的 PDO/CoE 失败不是 `ec_igb` 没有编译好；native 链路能扫到 15 个
从站，问题在进入 SDK 初始化后的 PDO/DC 配置路径。

## 11. native 失败根因复测

重新排查时先加了 native 扫描 gate：切到 `ec_igb` 后循环检查
`ethercat slaves`，直到 13 个 `MT_Device` 都出现且没有 `???` /
`0x00000000`。实际观察是第 1 次还没扫到电机，第 2 次有大量 unknown，第 3
次只有 10 个 `MT_Device`，第 4 次才完整。

随后做 A/B：

```text
默认路径（手动 remap + DC true）：5/5 fail
跳过手动 remap + DC true，且扫描已完整：3/3 fail
手动 remap + DC false，且扫描已完整：1/3 pass，2/3 fail
跳过手动 remap + DC false，且扫描已完整：5/5 pass
```

失败日志的特征：

```text
EtherCAT slave N SDO download clear RxPDO assignment 0x1c12:00 failed
AL status message 0x001A: "Synchronization error"
Domain X: Working counter changed to 1/3
```

稳定日志的特征：

```text
Domain 0..12: Working counter changed to 3/3
Slave states on main device: OP
13 个 motor 最后都是 status=0x0231 err=0x0000
```

对应 repo 处理：

- `config/configOriginal_heima.xml` 继续使用目标控制路径 `dc="true"`。
- `src/rmd_ethercat_backend.cpp` 在 DC 打开时显式选择第一个 MT_Device 作为
  reference clock。
- `src/rmd_ethercat_backend.cpp` 默认不再做 SDK 手动 PDO remap；需要回旧路径时才
  设置 `RMD_ECAT_FORCE_MANUAL_PDO_REMAP=1`。
- `src/rmd_ethercat_backend.cpp` 在 start 阶段等待 20 个连续 fresh 实时帧，避免
  DC 下总线还没全 OP 就把控制权交给上层。
- `src/rmd_ethercat_backend.cpp` 在 stop 阶段先请求 SAFEOP/PREOP，并等待 OP 位清掉
  后再停实时线程。
- 仍然保留 `RMD_ECAT_DISABLE_DC=1` 作为诊断开关，用来临时隔离 DC 变量。

旧处理合入默认路径后曾做过 5 次 native read-only 复测：

```text
logs/native_igb_default_after_fix_summary_20260513_181730.txt
result: 4/5 pass
failed case: 第 2 次 motor13 stale，tail 里仍是 status=0xffff err=0x0001
```

随后在 `generic + dc=true` 下复测新的 DC 启停路径：

```text
logs/rmd_ecat_snapshot_probe_i210_generic_dc_statewait_20260513_183502.log
result: pass
13 个 motor 末尾都是 status=0x0231 err=0x0000
最新 dmesg: stop 后无新的 0x001A Synchronization error，最终回 PREOP
```

随后按正确 native 绑定顺序复测：

```text
logs/rmd_ecat_snapshot_probe_i210_native_dc_sysfs_bind_20260513_184035.log
result: pass

logs/rmd_ecat_snapshot_probe_i210_native_dc_repeat1_20260513_184208.log
logs/rmd_ecat_snapshot_probe_i210_native_dc_repeat2_20260513_184220.log
logs/rmd_ecat_snapshot_probe_i210_native_dc_repeat3_20260513_184232.log
result: 3/3 pass

logs/rmd_ecat_snapshot_probe_i210_native_dc_cleanstop_20260513_184339.log
result: pass; stop 后无新的 0x001A Synchronization error
```

所以这里的正确结论是：DC 可以作为目标路径保留；之前 read-only 假失败的一部分来自
SDK start 太早返回，stop 又太早停实时线程。native 的 service/module 接管问题也
不是编译问题，而是启动顺序问题：`ec_master` 必须先带 MAC 参数等待设备，然后再把
PCI 设备从普通 `igb` 绑定到 `ec_igb`。

## 12. 回退到 generic

如果要放弃开机 native 接管，先停掉 native bind service 和 EtherCAT service：

```bash
sudo systemctl disable --now ethercat-i210-native-bind.service
sudo systemctl stop ethercat.service
sudo rm -f /etc/systemd/system/ethercat.service.d/40-native-clean.conf /etc/systemd/system/ethercat.service.d/50-native.conf
sudo systemctl daemon-reload
```

把 `/etc/ethercat.conf` 改回：

```bash
MASTER0_DEVICE="<I210_MAC>"
DEVICE_MODULES="generic"
UPDOWN_INTERFACES="enP5p1s0"
```

由于刚才手动把 PCI 设备绑定到了 `ec_igb`，停止 service 后 `enP5p1s0` 可能暂时不存在。
需要把 PCI 设备重新绑回普通 Linux `igb`：

```bash
sudo sh -c 'echo 0005:01:00.0 > /sys/bus/pci/drivers/igb/bind'
```

再启动 generic EtherCAT：

```bash
sudo systemctl start ethercat.service
```

恢复后确认：

```bash
grep -E '^(MASTER0_DEVICE|DEVICE_MODULES|UPDOWN_INTERFACES)=' /etc/ethercat.conf
lsmod | grep -E 'ec_|igb'
readlink -f /sys/class/net/enP5p1s0/device/driver
ethercat master
ethercat slaves
```

这次恢复后的状态：

```text
DEVICE_MODULES="generic"
UPDOWN_INTERFACES="enP5p1s0"
enP5p1s0 driver: /sys/bus/pci/drivers/igb
EtherCAT master: Link UP, Slaves 15, Lost frames 0
```

## 13. generic 路径已通过的验证

在同一块 I210 上，`generic` 路径已经通过：

```text
logs/rmd_ecat_position_targets_i210_generic_60s_20260513_170744.log
logs/rmd_ecat_mock_controller_i210_generic_5min_20260513_171250.log
```

60 秒位置目标测试结论：

```text
13 个电机最终 status=0x1237 err=0x0000
timing_summary period_ms=5, overruns=0
```

5 分钟模拟上层控制循环结论：

```text
policy_hz=100
iterations=30000
interval_avg_ms=10
interval_max_ms=10.0489
overruns=0
feedback_misses=0
imu_valid_samples=30000
```

所以当前已经验证过：

- `generic`：read-only、60 秒位置目标、5 分钟模拟上层控制循环通过。
- `generic + dc=true`：新的 DC start gate / stop state-wait 路径下，13 电机
  read-only snapshot probe 通过。
- native `ec_igb + dc=true`：正确 sysfs bind 顺序、scan gate、显式 MT_Device
  reference clock、跳过手动 remap、20 帧 start gate 后，13 电机 read-only
  snapshot probe 连续 3/3 通过；clean stop 复测没有新的 0x001A sync error。
- native `ec_igb + dc=true + SYNC0 shift=0`：13 电机短时间 402 enable / hold 通过，
  5 分钟姿态保持通过。5 分钟日志：
  `logs/rmd_ecat_position_targets_i210_native_dc_shift0_5min_hold_20260513_185559.log`。
  最终 13 个电机 `status=0x1237 err=0x0000`，marker 后没有新的 `0x001A` 或 PDO
  map error；`timing_summary period_ms=5 interval_samples=59999 interval_avg_ms=5
  interval_max_ms=12.8902 overruns=1`。

注意：半周期 SYNC0 shift 会在 native DC 进入 OP 时触发 `0x001A Synchronization
error`。单纯增加 OP 前 warmup 不能解决这个问题；当前代码使用
`ecrt_slave_config_dc(..., sync0Cycle, 0, 0, 0)`。

## 14. 后续排查方向

native `ec_igb` 的下一步不是重新编译；当前电机路径已经能完成 native DC 位置保持。
后续继续保留 `generic` 作为基线，并把 native 作为目标路径继续拉长测试：

1. 继续保留 `generic` 作为可用基线，避免电机验证被 native 调试阻塞。
2. native 每次切换后先确认 `/dev/EtherCAT0` 已创建，`ethercat master` 是
   `Link UP`、`Slaves 15`。
3. 等 `ethercat slaves` 显示 13 个 `MT_Device` 且没有 unknown，再启动 SDK。
4. 后续重点看更长时间运行、真实上层控制循环、IMU 同步路径和 overrun 来源。

## 15. 常用状态检查

检查当前 service：

```bash
systemctl status ethercat.service --no-pager
```

检查 master：

```bash
ethercat master
ethercat slaves
```

检查当前模块：

```bash
lsmod | grep -E 'ec_master|ec_generic|ec_igb'
```

检查 I210 绑定在哪个驱动：

```bash
readlink -f /sys/bus/pci/devices/0005:01:00.0/driver
```

检查普通网络接口是否恢复：

```bash
ip -o link show enP5p1s0
readlink -f /sys/class/net/enP5p1s0/device/driver
```
