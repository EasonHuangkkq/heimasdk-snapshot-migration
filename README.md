# RMD CAN SDK

CAN-only C++ SDK for RMD motors. This is intentionally separate from
`heimaSDK`: it has no EtherCAT, RS232, RS485, IMU, or CANHAL dependency.

## Build

```bash
cmake -S rmd_can_sdk -B rmd_can_sdk/build
cmake --build rmd_can_sdk/build
ctest --test-dir rmd_can_sdk/build --output-on-failure
```

`build/` is machine-local. If this repository is copied from x86_64 to an
Orin/Jetson, rebuild on the Orin instead of reusing the x86 binaries:

```bash
./rmd_can_sdk/scripts/build_on_target.sh
```

For hardware usage, see the Chinese guide:

- [docs/usage_guide_zh.md](docs/usage_guide_zh.md)

## Configuration

The parser keeps the familiar `configuration.xml` / `config.xml` shape and
reads only:

- `<CAN><Masters>`
- enabled `<CAN><Slaves><Slave>` entries whose `type` starts with `RMD`
- `<Motors><Motor>` parameters referenced by CAN slave `alias`

For RMD, `slave_id` is treated as the motor CAN id. The frame ids are:

- standard TX/RX: `0x140 + slave_id`, `0x240 + slave_id`
- MIT TX/RX: `0x400 + slave_id`, `0x500 + slave_id`

## Demo

```bash
sudo ./rmd_can_sdk/build/rmd_can_demo rmd_can_sdk/config/example_rmd_can.xml
```

The demo sends zero-gain MIT frames to enabled motors and prints decoded MIT
feedback.
