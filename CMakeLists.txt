cmake_minimum_required(VERSION 3.16)

project(rmd_can_sdk VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()

option(RMD_CAN_SDK_ENABLE_ECAT "Build IgH EtherCAT backend" OFF)

set(RMD_CAN_SDK_SOURCES
    src/common.cpp
    src/rmd_bench_workflow.cpp
    src/heima_ecat_types.cpp
    src/rmd_can_backend.cpp
    src/rmd_can_config.cpp
    src/rmd_can_transport.cpp
    src/rmd_driver_sdk.cpp
    src/rmd_ethercat_bindings.cpp
    src/rmd_ethercat_mt_device.cpp
    src/rmd_ethercat_pdo.cpp
    src/rmd_ethercat_snapshot.cpp
    src/rmd_motion_plan.cpp
    src/rmd_motor_registry.cpp
    src/rmd_protocol.cpp
    src/rmd_safety.cpp
    src/rmd_types.cpp
    src/rs232_imu_backend.cpp
    src/yesense_imu_decoder.cpp
    third_party/tinyxml2.cpp
    third_party/yesense/yesense_decoder.cpp
    third_party/yesense/yesense_std_out_decoder.cpp
)

if(RMD_CAN_SDK_ENABLE_ECAT)
    list(APPEND RMD_CAN_SDK_SOURCES src/rmd_ethercat_backend.cpp)
else()
    list(APPEND RMD_CAN_SDK_SOURCES src/rmd_ethercat_backend_stub.cpp)
endif()

add_library(rmd_can_sdk STATIC ${RMD_CAN_SDK_SOURCES})

target_include_directories(rmd_can_sdk PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/third_party
)

find_package(Threads REQUIRED)
target_link_libraries(rmd_can_sdk PUBLIC Threads::Threads)

if(RMD_CAN_SDK_ENABLE_ECAT)
    target_compile_definitions(rmd_can_sdk PUBLIC RMD_CAN_SDK_ENABLE_ECAT=1)
    target_link_libraries(rmd_can_sdk PUBLIC ethercat)
endif()

add_executable(rmd_can_sdk_tests
    tests/rmd_can_sdk_tests.cpp
)

target_link_libraries(rmd_can_sdk_tests PRIVATE
    rmd_can_sdk
)

add_test(NAME rmd_can_sdk_tests COMMAND rmd_can_sdk_tests)

add_executable(rmd_can_demo
    examples/rmd_can_demo.cpp
)

target_link_libraries(rmd_can_demo PRIVATE
    rmd_can_sdk
)

add_executable(rmd_ecat_snapshot_probe
    examples/rmd_ecat_snapshot_probe.cpp
)

target_link_libraries(rmd_ecat_snapshot_probe PRIVATE
    rmd_can_sdk
)

add_executable(rmd_ecat_hold_position
    examples/rmd_ecat_hold_position.cpp
)

target_link_libraries(rmd_ecat_hold_position PRIVATE
    rmd_can_sdk
)

add_executable(rmd_ecat_position_targets
    examples/rmd_ecat_position_targets.cpp
)

target_link_libraries(rmd_ecat_position_targets PRIVATE
    rmd_can_sdk
)

add_executable(rmd_ecat_step_response_test
    examples/rmd_ecat_step_response_test.cpp
)

target_link_libraries(rmd_ecat_step_response_test PRIVATE
    rmd_can_sdk
)

add_executable(rmd_ecat_pvt_torque_fit
    examples/rmd_ecat_pvt_torque_fit.cpp
)

target_link_libraries(rmd_ecat_pvt_torque_fit PRIVATE
    rmd_can_sdk
)

add_executable(rmd_status_probe
    examples/rmd_status_probe.cpp
)

target_link_libraries(rmd_status_probe PRIVATE
    rmd_can_sdk
)

add_executable(rmd_move_zero
    examples/rmd_move_zero.cpp
)

target_link_libraries(rmd_move_zero PRIVATE
    rmd_can_sdk
)

add_executable(rmd_a4_response_test
    examples/rmd_a4_response_test.cpp
)

target_link_libraries(rmd_a4_response_test PRIVATE
    rmd_can_sdk
)

add_executable(rmd_mit_sweep_test
    examples/rmd_mit_sweep_test.cpp
)

target_link_libraries(rmd_mit_sweep_test PRIVATE
    rmd_can_sdk
)

add_executable(rmd_mit_sine_test
    examples/rmd_mit_sine_test.cpp
)

target_link_libraries(rmd_mit_sine_test PRIVATE
    rmd_can_sdk
)

add_executable(rmd_mit_dual_sine_test
    examples/rmd_mit_dual_sine_test.cpp
)

target_link_libraries(rmd_mit_dual_sine_test PRIVATE
    rmd_can_sdk
)

add_executable(rmd_mit_sine_autotune_test
    examples/rmd_mit_sine_autotune_test.cpp
)

target_link_libraries(rmd_mit_sine_autotune_test PRIVATE
    rmd_can_sdk
)

add_executable(rmd_mit_torque_probe
    examples/rmd_mit_torque_probe.cpp
)

target_link_libraries(rmd_mit_torque_probe PRIVATE
    rmd_can_sdk
)

add_executable(yesense_imu_test
    examples/yesense_imu_test.cpp
)

target_link_libraries(yesense_imu_test PRIVATE
    rmd_can_sdk
)

add_test(
    NAME yesense_imu_test_rejects_missing_args
    COMMAND yesense_imu_test
)
set_tests_properties(yesense_imu_test_rejects_missing_args PROPERTIES
    WILL_FAIL TRUE
    FAIL_REGULAR_EXPRESSION "usage:"
)

add_test(
    NAME rmd_ecat_position_targets_rejects_bad_assignment
    COMMAND rmd_ecat_position_targets /tmp/rmd_missing_config.xml 1000 500 10 5 100 bad
)
set_tests_properties(rmd_ecat_position_targets_rejects_bad_assignment PROPERTIES
    WILL_FAIL TRUE
    FAIL_REGULAR_EXPRESSION "target assignment must be motor=position"
)

add_test(
    NAME rmd_ecat_position_targets_rejects_zero_motor_index
    COMMAND rmd_ecat_position_targets /tmp/rmd_missing_config.xml 1000 500 10 5 100 0=0.1
)
set_tests_properties(rmd_ecat_position_targets_rejects_zero_motor_index PROPERTIES
    WILL_FAIL TRUE
    FAIL_REGULAR_EXPRESSION "motor index must be positive"
)

add_test(
    NAME rmd_ecat_step_response_rejects_invalid_period
    COMMAND rmd_ecat_step_response_test /tmp/rmd_missing_config.xml 3 0.03 1000 2000 500 10 0 /tmp/rmd_step.csv 0
)
set_tests_properties(rmd_ecat_step_response_rejects_invalid_period PROPERTIES
    WILL_FAIL TRUE
    FAIL_REGULAR_EXPRESSION "period_us must be positive"
)

add_test(
    NAME rmd_ecat_step_response_rejects_invalid_mode
    COMMAND rmd_ecat_step_response_test /tmp/rmd_missing_config.xml 3 0.03 1000 2000 500 10 1000 /tmp/rmd_step.csv 0 999 200 0.9 0
)
set_tests_properties(rmd_ecat_step_response_rejects_invalid_mode PROPERTIES
    WILL_FAIL TRUE
    FAIL_REGULAR_EXPRESSION "mode must fit int8"
)

add_test(
    NAME rmd_mit_sweep_rejects_invalid_prezero_tolerance
    COMMAND rmd_mit_sweep_test /tmp/rmd_missing_config.xml 0 1 100 1000 20 30 2 2 -1
)
set_tests_properties(rmd_mit_sweep_rejects_invalid_prezero_tolerance PROPERTIES
    WILL_FAIL TRUE
    FAIL_REGULAR_EXPRESSION "loading config XML failed"
)

add_test(
    NAME rmd_mit_sweep_rejects_invalid_eval_deg
    COMMAND rmd_mit_sweep_test /tmp/rmd_missing_config.xml 0 1 100 1000 20 30 2 5 10 -1
)
set_tests_properties(rmd_mit_sweep_rejects_invalid_eval_deg PROPERTIES
    WILL_FAIL TRUE
    FAIL_REGULAR_EXPRESSION "loading config XML failed"
)

add_test(
    NAME rmd_mit_sweep_rejects_kd_above_protocol_range
    COMMAND rmd_mit_sweep_test /tmp/rmd_missing_config.xml 0 1 100 1000 20 30 60 5 10
)
set_tests_properties(rmd_mit_sweep_rejects_kd_above_protocol_range PROPERTIES
    WILL_FAIL TRUE
    FAIL_REGULAR_EXPRESSION "kd must be in 0..50"
)

add_test(
    NAME rmd_mit_sweep_rejects_invalid_ramp_ms
    COMMAND rmd_mit_sweep_test /tmp/rmd_missing_config.xml 0 1 100 1000 20 30 2 5 10 20 -1
)
set_tests_properties(rmd_mit_sweep_rejects_invalid_ramp_ms PROPERTIES
    WILL_FAIL TRUE
    FAIL_REGULAR_EXPRESSION "loading config XML failed"
)

add_test(
    NAME rmd_mit_sweep_rejects_invalid_sensitive_motion_pos
    COMMAND rmd_mit_sweep_test /tmp/rmd_missing_config.xml 0 1 100 1000 20 30 2 5 10 20 0 -0.1
)
set_tests_properties(rmd_mit_sweep_rejects_invalid_sensitive_motion_pos PROPERTIES
    WILL_FAIL TRUE
    FAIL_REGULAR_EXPRESSION "loading config XML failed"
)

add_test(
    NAME rmd_mit_sweep_rejects_invalid_sensitive_motion_vel
    COMMAND rmd_mit_sweep_test /tmp/rmd_missing_config.xml 0 1 100 1000 20 30 2 5 10 20 0 0.1 6
)
set_tests_properties(rmd_mit_sweep_rejects_invalid_sensitive_motion_vel PROPERTIES
    WILL_FAIL TRUE
    FAIL_REGULAR_EXPRESSION "loading config XML failed"
)

add_test(
    NAME rmd_mit_sine_rejects_invalid_amplitude
    COMMAND rmd_mit_sine_test /tmp/rmd_missing_config.xml 20 1000 0 2 50 2
)
set_tests_properties(rmd_mit_sine_rejects_invalid_amplitude PROPERTIES
    WILL_FAIL TRUE
    FAIL_REGULAR_EXPRESSION "amplitude_deg must be in 0..180"
)

add_test(
    NAME rmd_mit_sine_rejects_invalid_frequency
    COMMAND rmd_mit_sine_test /tmp/rmd_missing_config.xml 20 1000 20 0 50 2
)
set_tests_properties(rmd_mit_sine_rejects_invalid_frequency PROPERTIES
    WILL_FAIL TRUE
    FAIL_REGULAR_EXPRESSION "frequency_hz must be in 0..20"
)

add_test(
    NAME rmd_mit_sine_rejects_invalid_center
    COMMAND rmd_mit_sine_test /tmp/rmd_missing_config.xml 20 1000 20 2 50 2 /tmp/rmd_mit_sine_test_out nope
)
set_tests_properties(rmd_mit_sine_rejects_invalid_center PROPERTIES
    WILL_FAIL TRUE
    FAIL_REGULAR_EXPRESSION "center_deg must be a number"
)

add_test(
    NAME rmd_mit_sine_rejects_center_outside_config_limits
    COMMAND rmd_mit_sine_test ${CMAKE_SOURCE_DIR}/config/config4.xml 8 500 25 0.2 200 0.9 /tmp/rmd_mit_sine_test_out -200
)
set_tests_properties(rmd_mit_sine_rejects_center_outside_config_limits PROPERTIES
    WILL_FAIL TRUE
    FAIL_REGULAR_EXPRESSION "sine target outside configured limits"
)

add_test(
    NAME rmd_mit_sine_rejects_invalid_phase
    COMMAND rmd_mit_sine_test /tmp/rmd_missing_config.xml 20 1000 20 2 50 2 /tmp/rmd_mit_sine_test_out 0 nope
)
set_tests_properties(rmd_mit_sine_rejects_invalid_phase PROPERTIES
    WILL_FAIL TRUE
    FAIL_REGULAR_EXPRESSION "phase_deg must be a number"
)

add_test(
    NAME rmd_mit_dual_sine_audit_self_test
    COMMAND rmd_mit_dual_sine_test --audit-self-test
)

add_test(
    NAME rmd_mit_dual_sine_rejects_partial_absolute_args
    COMMAND rmd_mit_dual_sine_test /tmp/rmd_missing_config.xml 8 500 0.2 30 200 0.9 25 120 0.9 /tmp/rmd_dual_out 0
)
set_tests_properties(rmd_mit_dual_sine_rejects_partial_absolute_args PROPERTIES
    WILL_FAIL TRUE
    FAIL_REGULAR_EXPRESSION "absolute mode requires m3_center_deg m3_phase_deg m4_center_deg m4_phase_deg"
)

add_test(
    NAME rmd_mit_dual_sine_rejects_invalid_monitor_hz
    COMMAND rmd_mit_dual_sine_test /tmp/rmd_missing_config.xml 8 500 0.2 30 200 0.9 25 120 0.9 /tmp/rmd_dual_out 0 0 -25 90 nope
)
set_tests_properties(rmd_mit_dual_sine_rejects_invalid_monitor_hz PROPERTIES
    WILL_FAIL TRUE
    FAIL_REGULAR_EXPRESSION "monitor_hz must be numeric"
)

add_test(
    NAME rmd_mit_sine_autotune_rejects_invalid_eval_amplitude
    COMMAND rmd_mit_sine_autotune_test /tmp/rmd_missing_config.xml 5 1000 0 1
)
set_tests_properties(rmd_mit_sine_autotune_rejects_invalid_eval_amplitude PROPERTIES
    WILL_FAIL TRUE
    FAIL_REGULAR_EXPRESSION "eval_amplitude_deg must be in 0..180"
)

add_test(
    NAME rmd_mit_sine_autotune_rejects_too_small_command_limit
    COMMAND rmd_mit_sine_autotune_test /tmp/rmd_missing_config.xml 5 1000 20 1 80 0.5 1.0 10
)
set_tests_properties(rmd_mit_sine_autotune_rejects_too_small_command_limit PROPERTIES
    WILL_FAIL TRUE
    FAIL_REGULAR_EXPRESSION "max_command_amplitude_deg must be >= eval_amplitude_deg"
)

add_test(
    NAME rmd_mit_sine_autotune_audit_self_test
    COMMAND rmd_mit_sine_autotune_test --audit-self-test
)

add_test(
    NAME rmd_mit_torque_probe_rejects_invalid_torque
    COMMAND rmd_mit_torque_probe /tmp/rmd_missing_config.xml -0.1 80 0.5 1000 1000
)
set_tests_properties(rmd_mit_torque_probe_rejects_invalid_torque PROPERTIES
    WILL_FAIL TRUE
    FAIL_REGULAR_EXPRESSION "ff_torque_nm must be in \\[0,10\\]"
)

add_test(
    NAME rmd_mit_torque_probe_rejects_torque_above_probe_range
    COMMAND rmd_mit_torque_probe /tmp/rmd_missing_config.xml 10.1 80 1 1000 1000
)
set_tests_properties(rmd_mit_torque_probe_rejects_torque_above_probe_range PROPERTIES
    WILL_FAIL TRUE
    FAIL_REGULAR_EXPRESSION "ff_torque_nm must be in \\[0,10\\]"
)

add_test(
    NAME rmd_mit_torque_probe_rejects_invalid_target_position
    COMMAND rmd_mit_torque_probe /tmp/rmd_missing_config.xml 1 80 1 1000 1000 /tmp/rmd_torque_probe_out nope
)
set_tests_properties(rmd_mit_torque_probe_rejects_invalid_target_position PROPERTIES
    WILL_FAIL TRUE
    FAIL_REGULAR_EXPRESSION "target_pos_rad must be a number"
)

add_test(
    NAME rmd_mit_torque_probe_rejects_invalid_torque_min
    COMMAND rmd_mit_torque_probe /tmp/rmd_missing_config.xml 1 80 1 1000 1000 /tmp/rmd_torque_probe_out 0.1 1 nope
)
set_tests_properties(rmd_mit_torque_probe_rejects_invalid_torque_min PROPERTIES
    WILL_FAIL TRUE
    FAIL_REGULAR_EXPRESSION "torque_min_nm must be a number"
)
