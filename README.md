# RMD CAN SDK

RMD CAN SDK is a C++20 motor-control SDK for RMD motors. It provides one
high-level `DriverSDK` API over SocketCAN and an optional IgH EtherCAT backend
for RMD `MT_Device` drives migrated from a working heimaSDK EtherCAT path.

This repository is an open-source alpha. It is suitable for development,
inspection, and controlled bench testing. Treat every hardware command as
potentially dangerous until you have verified the configuration, motor limits,
direction, current limit, and emergency stop path on your own robot.

## Features

- SocketCAN backend for RMD CAN motors.
- Optional IgH EtherCAT backend with MT_Device PDO remapping.
- EtherCAT CSP/position mode `8`, PVT mode `5`, and CST/torque mode `10`.
- Versioned single-producer/single-consumer frame buffers between the user
  thread and realtime backends.
- EtherCAT process lock so only one process opens the same master.
- EtherCAT command watchdog and backend status counters.
- Example tools for CAN probing, MIT tests, EtherCAT snapshot probing, and
  EtherCAT hold-position tests.

## Safety

- Run first tests with motors unloaded or mechanically constrained.
- Start with read-only probes before enabling torque.
- Use conservative current limits and position limits.
- Verify motor polarity and encoder zero before closed-loop tests.
- Keep an emergency stop path outside this SDK.
- Only one process should control an EtherCAT master at a time.

## Dependencies

- CMake 3.16 or newer
- C++20 compiler
- pthreads
- Linux SocketCAN for CAN usage
- IgH EtherCAT master and `libethercat` for EtherCAT usage

For EtherCAT builds, make sure `libethercat` is visible to the linker and
runtime loader. On many targets this is enough:

```bash
echo /usr/local/lib | sudo tee /etc/ld.so.conf.d/local-libethercat.conf
sudo ldconfig
```

## Build

CAN-only build:

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

EtherCAT-enabled build:

```bash
cmake -S . -B build-ecat -DRMD_CAN_SDK_ENABLE_ECAT=ON
cmake --build build-ecat
ctest --test-dir build-ecat --output-on-failure
```

Do not reuse build directories copied from another CPU architecture. If this
repository is copied from x86_64 to an Orin or Jetson, rebuild on the target:

```bash
./scripts/build_on_target.sh
```

## CAN Quick Start

Bring up a CAN interface:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
```

Run the CAN demo:

```bash
sudo ./build/rmd_can_demo config/example_rmd_can.xml
```

## EtherCAT Quick Start

Check that the IgH master sees the bus:

```bash
sudo ethercat slaves
```

Build with EtherCAT support:

```bash
cmake -S . -B build-ecat -DRMD_CAN_SDK_ENABLE_ECAT=ON
cmake --build build-ecat --target rmd_ecat_snapshot_probe rmd_ecat_hold_position
```

Run a read-only position-mode probe:

```bash
sudo ./build-ecat/rmd_ecat_snapshot_probe config/example_rmd_ethercat_mt_device.xml 450 8
```

Edit `config/example_rmd_ethercat_mt_device.xml` before running this command.
The example assumes two MT_Device slaves at physical positions `0` and `1`.
If your topology has branch or splitter devices, skip those positions and map
only the actual MT_Device drives.

Hold current position in position mode with a current limit:

```bash
sudo ./build-ecat/rmd_ecat_hold_position config/example_rmd_ethercat_mt_device.xml 10000 500 450 5 current
```

For more EtherCAT details, see
[docs/ethercat_quickstart_zh.md](docs/ethercat_quickstart_zh.md).

## Configuration

The parser keeps the familiar heimaSDK XML shape:

- `<CAN>` configures SocketCAN masters and RMD CAN slaves.
- `<ECAT>` configures IgH EtherCAT masters, domains, and MT_Device slaves.
- `<Motors>` stores motor parameters shared by both backends.

For RMD CAN, `slave_id` is the motor CAN id:

- Standard TX/RX: `0x140 + slave_id`, `0x240 + slave_id`
- MIT TX/RX: `0x400 + slave_id`, `0x500 + slave_id`

For MT_Device EtherCAT:

- Mode `8` and mode `10` use RxPDO `0x1600`.
- Mode `5` uses RxPDO `0x1601`.
- TxPDO uses `0x1A02` for status, position, velocity, torque, error,
  temperature, drive temperature, voltage, and mode display.

## Contributing

Keep changes small and testable. Hardware-specific behavior should be backed by
a bench-test note or a reproducible probe command. Pure logic changes should
include CTest coverage. See [CONTRIBUTING.md](CONTRIBUTING.md).

Run before submitting changes:

```bash
cmake -S . -B build-check -DRMD_CAN_SDK_ENABLE_ECAT=ON
cmake --build build-check
ctest --test-dir build-check --output-on-failure
```

## License

MIT. See [LICENSE](LICENSE).
