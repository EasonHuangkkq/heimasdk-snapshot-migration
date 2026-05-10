#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

CONFIG="${1:-${SDK_DIR}/config/example_rmd_can.xml}"
CAN_IF="${2:-can0}"
BITRATE="${3:-1000000}"
DURATION_S="${4:-20}"
SAMPLE_HZ="${5:-1000}"
AMPLITUDE_DEG="${6:-20}"
FREQUENCY_HZ="${7:-2}"
KP="${8:-50}"
KD="${9:-2}"
OUTPUT_PREFIX="${10:-}"

BIN="${SDK_DIR}/build/rmd_mit_sine_test"

"${SCRIPT_DIR}/ensure_target_binary.sh" rmd_mit_sine_test "${BIN}"

echo "Configuring ${CAN_IF} bitrate=${BITRATE}"
sudo ip link set "${CAN_IF}" down || true
sudo ip link set "${CAN_IF}" type can bitrate "${BITRATE}"
sudo ip link set "${CAN_IF}" up

echo "CAN status:"
ip -details -statistics link show "${CAN_IF}"

if [[ -z "${OUTPUT_PREFIX}" ]]; then
    mkdir -p logs
    OUTPUT_PREFIX="logs/rmd_mit_sine_$(date +%Y%m%d_%H%M%S)"
fi

echo "Running MIT sine test"
echo "  config=${CONFIG}"
echo "  duration_s=${DURATION_S}"
echo "  sample_hz=${SAMPLE_HZ}"
echo "  amplitude_deg=${AMPLITUDE_DEG}"
echo "  frequency_hz=${FREQUENCY_HZ}"
echo "  kp=${KP}"
echo "  kd=${KD}"
echo "  output_prefix=${OUTPUT_PREFIX}"

set +e
sudo "${BIN}" "${CONFIG}" "${DURATION_S}" "${SAMPLE_HZ}" "${AMPLITUDE_DEG}" "${FREQUENCY_HZ}" "${KP}" "${KD}" "${OUTPUT_PREFIX}"
STATUS=$?
if [[ -n "${SUDO_USER:-}" ]]; then
    sudo chown "${SUDO_USER}:${SUDO_USER}" "${OUTPUT_PREFIX}"_samples.csv "${OUTPUT_PREFIX}"_summary.csv 2>/dev/null || true
fi
exit "${STATUS}"
