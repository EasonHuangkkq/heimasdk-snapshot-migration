#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${1:-${SDK_DIR}/build}"

host_arch="$(uname -m)"

remove_foreign_build_if_needed() {
    [[ -d "${BUILD_DIR}" ]] || return 0

    local probe=""
    probe="$(find "${BUILD_DIR}" -maxdepth 1 -type f -perm -111 -print -quit 2>/dev/null || true)"
    [[ -n "${probe}" ]] || return 0
    command -v file >/dev/null 2>&1 || return 0

    local kind
    kind="$(file -b "${probe}")"

    case "${host_arch}" in
        aarch64|arm64)
            if [[ "${kind}" == *"x86-64"* ]]; then
                echo "Removing x86_64 build directory before rebuilding for ${host_arch}: ${BUILD_DIR}"
                rm -rf "${BUILD_DIR}"
            fi
            ;;
        x86_64|amd64)
            if [[ "${kind}" == *"ARM aarch64"* || "${kind}" == *"aarch64"* ]]; then
                echo "Removing ARM64 build directory before rebuilding for ${host_arch}: ${BUILD_DIR}"
                rm -rf "${BUILD_DIR}"
            fi
            ;;
    esac
}

remove_foreign_build_if_needed

cmake -S "${SDK_DIR}" -B "${BUILD_DIR}"
cmake --build "${BUILD_DIR}" -j "$(nproc)"
ctest --test-dir "${BUILD_DIR}" --output-on-failure

echo "Build completed for ${host_arch}: ${BUILD_DIR}"
