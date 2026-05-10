#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

CONFIG="${1:-${SDK_DIR}/config/example_rmd_can.xml}"
CAN_IF="${2:-can0}"
BITRATE="${3:-1000000}"
ZERO_SPEED_DPS="${4:-0}"
CYCLES="${5:-4}"
HOLD_MS="${6:-2500}"
SAMPLE_HZ="${7:-1000}"
SWEEP_DEG="${8:-57.2957795}"
KP="${9:-30}"
KD="${10:-2}"
TOLERANCE_DEG="${11:-10}"
PREZERO_TOLERANCE_DEG="${12:-10}"
EVAL_DEG="${13:-}"
RAMP_MS="${14:-0}"
SENSITIVE_MOTION_POS_DEG="${15:-0.1}"
SENSITIVE_MOTION_VEL_DEG_S="${16:-2}"
OUTPUT_PREFIX="${17:-}"

if [[ -n "${EVAL_DEG}" && ! "${EVAL_DEG}" =~ ^[-+]?[0-9]+([.][0-9]+)?$ ]]; then
    OUTPUT_PREFIX="${EVAL_DEG}"
    EVAL_DEG=""
    RAMP_MS="0"
elif [[ -n "${EVAL_DEG}" ]]; then
    if [[ -n "${RAMP_MS}" && ! "${RAMP_MS}" =~ ^[0-9]+$ ]]; then
        OUTPUT_PREFIX="${RAMP_MS}"
        RAMP_MS="0"
    fi
elif [[ -n "${RAMP_MS}" && ! "${RAMP_MS}" =~ ^[0-9]+$ ]]; then
    OUTPUT_PREFIX="${RAMP_MS}"
    RAMP_MS="0"
fi

if [[ -n "${SENSITIVE_MOTION_POS_DEG}" && ! "${SENSITIVE_MOTION_POS_DEG}" =~ ^[-+]?[0-9]+([.][0-9]+)?$ ]]; then
    OUTPUT_PREFIX="${SENSITIVE_MOTION_POS_DEG}"
    SENSITIVE_MOTION_POS_DEG="0.1"
    SENSITIVE_MOTION_VEL_DEG_S="2"
elif [[ -n "${SENSITIVE_MOTION_VEL_DEG_S}" && ! "${SENSITIVE_MOTION_VEL_DEG_S}" =~ ^[-+]?[0-9]+([.][0-9]+)?$ ]]; then
    OUTPUT_PREFIX="${SENSITIVE_MOTION_VEL_DEG_S}"
    SENSITIVE_MOTION_VEL_DEG_S="2"
fi

BIN="${SDK_DIR}/build/rmd_mit_sweep_test"

"${SCRIPT_DIR}/ensure_target_binary.sh" rmd_mit_sweep_test "${BIN}"

echo "Configuring ${CAN_IF} bitrate=${BITRATE}"
sudo ip link set "${CAN_IF}" down || true
sudo ip link set "${CAN_IF}" type can bitrate "${BITRATE}"
sudo ip link set "${CAN_IF}" up

echo "CAN status:"
ip -details -statistics link show "${CAN_IF}"

echo "Running MIT sweep test"
echo "  config=${CONFIG}"
echo "  zero_speed_dps=${ZERO_SPEED_DPS}"
echo "  cycles=${CYCLES}"
echo "  hold_ms=${HOLD_MS}"
echo "  sample_hz=${SAMPLE_HZ}"
echo "  command_deg=${SWEEP_DEG}"
echo "  eval_deg=${EVAL_DEG:-${SWEEP_DEG}}"
echo "  ramp_ms=${RAMP_MS}"
echo "  kp=${KP}"
echo "  kd=${KD}"
echo "  tolerance_deg=${TOLERANCE_DEG}"
echo "  prezero_tolerance_deg=${PREZERO_TOLERANCE_DEG}"
echo "  sensitive_motion_pos_deg=${SENSITIVE_MOTION_POS_DEG}"
echo "  sensitive_motion_vel_deg_s=${SENSITIVE_MOTION_VEL_DEG_S}"

if [[ -z "${OUTPUT_PREFIX}" ]]; then
    mkdir -p logs
    OUTPUT_PREFIX="logs/rmd_mit_sweep_$(date +%Y%m%d_%H%M%S)"
fi
echo "  output_prefix=${OUTPUT_PREFIX}"

set +e
ARGS=("${BIN}" "${CONFIG}" "${ZERO_SPEED_DPS}" "${CYCLES}" "${HOLD_MS}" "${SAMPLE_HZ}" "${SWEEP_DEG}" "${KP}" "${KD}" "${TOLERANCE_DEG}" "${PREZERO_TOLERANCE_DEG}")
ARGS+=("${EVAL_DEG:-${SWEEP_DEG}}")
ARGS+=("${RAMP_MS}")
ARGS+=("${SENSITIVE_MOTION_POS_DEG}" "${SENSITIVE_MOTION_VEL_DEG_S}")
ARGS+=("${OUTPUT_PREFIX}")
sudo "${ARGS[@]}"
STATUS=$?
if [[ -n "${SUDO_USER:-}" ]]; then
    sudo chown "${SUDO_USER}:${SUDO_USER}" "${OUTPUT_PREFIX}"_samples.csv "${OUTPUT_PREFIX}"_summary.csv 2>/dev/null || true
fi
exit "${STATUS}"
