# Repository Guidelines

## Project Structure & Module Organization

This is a C++20 motor-control SDK. Public headers live in `include/rmd_can_sdk/`; implementation files live in `src/`. Example and hardware probe tools are in `examples/`, with names such as `rmd_ecat_position_targets.cpp` and `yesense_imu_test.cpp`. Unit and CLI regression tests are in `tests/` and registered through `CMakeLists.txt`. XML motor and bus configurations are in `config/`; operational notes and hardware procedures are in `docs/`. Third-party vendored code is isolated under `third_party/`.

`rmd_can_sdk` is a historical public API name, not a CAN-only boundary. Treat this repository as a motor-control SDK that currently supports CAN, EtherCAT, realtime snapshot exchange, IMU input, and leg/joint kinematics. Do not rename `include/rmd_can_sdk/`, the `rmd_can_sdk` CMake target, or the `RmdCanSdk` namespace casually; they are compatibility surfaces.

## Build, Test, and Development Commands

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

On target hardware, prefer a fresh build directory. Do not reuse build trees copied across architectures. Use `./scripts/build_on_target.sh` when validating on Jetson/Orin.

## Coding Style & Naming Conventions

Use C++20, 4-space indentation, and existing brace/layout style. Keep APIs in the `RmdCanSdk` or `DriverSDK` namespaces as appropriate. Prefer descriptive lowerCamelCase for functions and variables, PascalCase for types, and `rmd_*` executable names. Keep comments short and only where they explain non-obvious realtime, EtherCAT, or safety behavior.

## Testing Guidelines

Add logic coverage to `tests/rmd_can_sdk_tests.cpp` and CLI rejection tests to `CMakeLists.txt` via CTest. Hardware tools should fail fast on invalid arguments before SDK init, so they are testable without devices. Run `ctest --output-on-failure` before handoff. For EtherCAT or IMU changes, include the exact probe command and log path used for bench validation.

## Commit & Pull Request Guidelines

Recent commits use short imperative summaries, for example `Extract shared EtherCAT bench workflow helpers`. Keep commits focused and avoid mixing hardware config changes with unrelated refactors. PRs should describe behavior changes, list verification commands, call out hardware used, and link relevant logs or docs. Mention any safety limits, current limits, or topology assumptions.

## Safety & Configuration Notes

Treat every motor command as hazardous. Start with read-only probes, conservative current limits, verified XML limits, and a separate emergency stop path. For the validated Orin EtherCAT setup, `config/configOriginal_heima.xml` is the main config; splitter devices are present in the physical chain but are not motors. Only one process should own the EtherCAT master at a time.
