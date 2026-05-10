# EtherCAT MT_Device Profile Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Support MT_Device CSP/CSV/CST/PVT using the correct EtherCAT PDO layouts from `MT-Device_260224.xml`.

**Architecture:** Keep PDO remapping static at `init()` time because `0x1C12/0x1C13` assignments must be changed in PREOP. Select RxPDO profile from each motor's configured mode: mode 5 uses PVT RxPDO `0x1601`; modes 8, 9, and 10 use standard RxPDO `0x1600`. Use extended TxPDO `0x1A02` for all modes so temperature, drive temperature, and voltage remain available.

**Tech Stack:** C++17, IgH EtherCAT master, existing DriverSDK compatibility API, unit tests in `tests/rmd_can_sdk_tests.cpp`.

---

### Task 1: Add Explicit MT_Device PDO Profiles

**Files:**
- Modify: `include/rmd_can_sdk/heima_ecat_types.h`
- Modify: `include/rmd_can_sdk/rmd_ethercat_mt_device.h`
- Modify: `src/rmd_ethercat_mt_device.cpp`

- [ ] Add packed `HeimaStandardRxData` for RxPDO `0x1600`:
  - `ControlWord` `0x6040` u16
  - `TargetPosition` `0x607A` i32
  - `TargetVelocity` `0x60FF` i32
  - `TargetTorque` `0x6071` i16
  - `MaxTorque` `0x6072` u16
  - `Mode` `0x6060` i8
  - padding `0x2FFD` i8
  - assert size 16 bytes.
- [ ] Rename the existing 22-byte PVT layout through aliases so existing code can migrate cleanly.
- [ ] Add `EthercatMtDeviceRxProfile` and `EthercatMtDeviceTxProfile` enums.
- [ ] Change `EthercatMtDevicePdoSpec` to use vectors for Rx/Tx entries because `0x1600` has 7 Rx entries and `0x1601` has 8.
- [ ] Add helpers:
  - `ethercatMtDeviceRxProfileForMode(int mode)`
  - `ethercatMtDeviceRxPdoSpec(EthercatMtDeviceRxProfile profile)`
  - `ethercatMtDeviceTxPdoSpec(EthercatMtDeviceTxProfile profile)`
  - `ethercatMtDeviceRxPdoSize(...)`
  - `ethercatMtDeviceTxPdoSize(...)`

### Task 2: Store Profile Selection In Bindings

**Files:**
- Modify: `include/rmd_can_sdk/rmd_ethercat_bindings.h`
- Modify: `src/rmd_ethercat_bindings.cpp`
- Modify: `src/rmd_driver_sdk.cpp`

- [ ] Add profile fields and byte sizes to `EthercatPdoBinding`.
- [ ] Add `assignEthercatRxProfiles(bindings, operatingModes)` helper.
- [ ] Before starting the EtherCAT backend, pass the frozen `operatingModes_` into config or registry-facing binding construction so each motor has a deterministic Rx profile.
- [ ] Keep `setMode()` rejected after `init()` to prevent runtime PDO remapping.

### Task 3: Remap And Register PDOs By Profile

**Files:**
- Modify: `src/rmd_ethercat_backend.cpp`

- [ ] Replace fixed `ethercatMtDevicePdoSpec()` usage with binding-specific Rx/Tx specs.
- [ ] Remap `0x1C12:01` to `0x1600` for standard modes and `0x1601` for PVT.
- [ ] Remap `0x1C13:01` to `0x1A02` for all modes.
- [ ] Register Rx/Tx entries using vector sizes, preserving first-entry offset capture.
- [ ] Store the selected profile and byte sizes on the binding after successful configuration.

### Task 4: Pack And Parse PDO Snapshots By Profile

**Files:**
- Modify: `include/rmd_can_sdk/rmd_ethercat_pdo.h`
- Modify: `src/rmd_ethercat_pdo.cpp`
- Modify: `src/rmd_ethercat_snapshot.cpp`

- [ ] Add `packEthercatStandardRxPdo()` and `packEthercatPvtRxPdo()`.
- [ ] For standard RxPDO, write `MaxTorque = target.maxCurrent`.
- [ ] For PVT RxPDO, write `PvtKp/PvtKd` only in mode 5.
- [ ] Keep TxPDO parsing through `0x1A02` extended feedback.
- [ ] Snapshot range checks must use binding-specific Rx/Tx sizes instead of fixed 22-byte sizes.

### Task 5: Unit Tests And Hardware-Safe Verification

**Files:**
- Modify: `tests/rmd_can_sdk_tests.cpp`

- [ ] Add tests for 16-byte standard RxPDO layout and entry order from `MT-Device_260224.xml`.
- [ ] Update existing PVT layout tests to explicitly assert `0x1601`.
- [ ] Add tests for mode-to-profile selection: 5 -> PVT, 8/9/10 -> Standard.
- [ ] Add tests that CST mode writes `MaxTorque` in the standard RxPDO struct.
- [ ] Run:
  - `cmake --build build-ecat-orin --target rmd_can_sdk_tests rmd_ecat_snapshot_probe`
  - `ctest --test-dir build-ecat-orin --output-on-failure`
  - `sudo ./build-ecat-orin/rmd_ecat_snapshot_probe config/configOriginal_heima.xml 450`
