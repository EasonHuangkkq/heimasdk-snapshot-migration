#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

CAN_IF="${1:-can0}"
BITRATE="${2:-1000000}"
MOTOR_ID="${3:-14}"
DURATION_S="${4:-300}"
MAX_SPEED_DPS="${5:-10}"
TARGET_A_DEG="${6:-0}"
TARGET_B_DEG="${7:-30}"
SAMPLE_HZ="${8:-1000}"
OUTPUT_PREFIX="${9:-}"

BIN="${SDK_DIR}/build/rmd_a4_response_test"

"${SCRIPT_DIR}/ensure_target_binary.sh" rmd_a4_response_test "${BIN}"

echo "Configuring ${CAN_IF} bitrate=${BITRATE}"
sudo ip link set "${CAN_IF}" down || true
sudo ip link set "${CAN_IF}" type can bitrate "${BITRATE}"
sudo ip link set "${CAN_IF}" up

echo "CAN status:"
ip -details -statistics link show "${CAN_IF}"

echo "Running A4 response test"
echo "  can_if=${CAN_IF}"
echo "  motor_id=${MOTOR_ID}"
echo "  duration_s=${DURATION_S}"
echo "  max_speed_dps=${MAX_SPEED_DPS}"
echo "  target_a_deg=${TARGET_A_DEG}"
echo "  target_b_deg=${TARGET_B_DEG}"
echo "  sample_hz=${SAMPLE_HZ}"

if [[ -z "${OUTPUT_PREFIX}" ]]; then
    mkdir -p logs
    OUTPUT_PREFIX="logs/rmd_a4_response_$(date +%Y%m%d_%H%M%S)"
fi
echo "  output_prefix=${OUTPUT_PREFIX}"

set +e
sudo "${BIN}" "${CAN_IF}" "${MOTOR_ID}" "${DURATION_S}" "${MAX_SPEED_DPS}" "${TARGET_A_DEG}" "${TARGET_B_DEG}" "${SAMPLE_HZ}" "${OUTPUT_PREFIX}"
STATUS=$?
if [[ -n "${SUDO_USER:-}" ]]; then
    sudo chown "${SUDO_USER}:${SUDO_USER}" "${OUTPUT_PREFIX}"_samples.csv "${OUTPUT_PREFIX}"_summary.csv 2>/dev/null || true
fi
exit "${STATUS}"
