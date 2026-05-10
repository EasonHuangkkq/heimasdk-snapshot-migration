#!/usr/bin/env bash
set -euo pipefail

BUILD_DIR="${BUILD_DIR:-build-ecat}"
CONFIG="${1:-config/example_rmd_can.xml}"

cmake -S . -B "${BUILD_DIR}" -DRMD_CAN_SDK_ENABLE_ECAT=ON
cmake --build "${BUILD_DIR}"
ctest --test-dir "${BUILD_DIR}" --output-on-failure

echo "Built and tested ${BUILD_DIR}"
echo "Use the hardware-specific mixed CAN/EtherCAT config as the first argument."
echo "Config path: ${CONFIG}"
