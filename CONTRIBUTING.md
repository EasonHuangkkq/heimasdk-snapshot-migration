# Contributing

This project controls real motors. Keep changes small, reviewable, and backed
by tests or a reproducible hardware probe.

## Before Sending A Change

Run the software checks:

```bash
cmake -S . -B build-check
cmake --build build-check
ctest --test-dir build-check --output-on-failure
```

If your change touches EtherCAT code, also build on a target with IgH
EtherCAT installed:

```bash
cmake -S . -B build-ecat-check -DRMD_CAN_SDK_ENABLE_ECAT=ON
cmake --build build-ecat-check
ctest --test-dir build-ecat-check --output-on-failure
```

## Hardware Changes

For hardware-facing changes, include:

- hardware topology
- motor type and firmware version if known
- config file used
- exact command run
- current/torque limits used
- final motor status and error code

Start with read-only probes, then low-current closed-loop tests.

## Code Style

- Prefer existing SDK patterns over new abstractions.
- Keep realtime paths allocation-free after backend startup.
- Avoid logging in realtime loops unless guarded by an environment variable.
- Keep XML examples generic and clearly mark hardware-specific configs.
- Do not commit build directories, logs, local plans, or vendor PDFs.

## EtherCAT Notes

- `setMode()` and `setMaxCurr()` must be called before `DriverSDK::init()`.
- MT_Device mode `5` uses RxPDO `0x1601`.
- MT_Device modes `8` and `10` use RxPDO `0x1600`.
- TxPDO uses `0x1A02`.
- Branch or splitter devices should appear in `ethercat slaves`, but not as
  SDK `MT_Device` entries.
