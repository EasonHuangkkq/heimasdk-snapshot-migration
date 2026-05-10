#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 2 ]]; then
    echo "usage: ensure_target_binary.sh <cmake-target> <binary-path>" >&2
    exit 2
fi

TARGET="$1"
BIN="$2"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${SDK_DIR}/build"

needs_build=0
host_arch="$(uname -m)"

if [[ ! -x "${BIN}" ]]; then
    needs_build=1
elif command -v file >/dev/null 2>&1; then
    kind="$(file -b "${BIN}")"
    case "${host_arch}" in
        aarch64|arm64)
            if [[ "${kind}" == *"x86-64"* ]]; then
                echo "Removing x86_64 build directory before rebuilding for ${host_arch}: ${BUILD_DIR}"
                rm -rf "${BUILD_DIR}"
                needs_build=1
            fi
            ;;
        x86_64|amd64)
            if [[ "${kind}" == *"ARM aarch64"* || "${kind}" == *"aarch64"* ]]; then
                echo "Removing ARM64 build directory before rebuilding for ${host_arch}: ${BUILD_DIR}"
                rm -rf "${BUILD_DIR}"
                needs_build=1
            fi
            ;;
    esac
fi

if (( needs_build )); then
    cmake -S "${SDK_DIR}" -B "${BUILD_DIR}"
    cmake --build "${BUILD_DIR}" --target "${TARGET}" -j "$(nproc)"
fi
