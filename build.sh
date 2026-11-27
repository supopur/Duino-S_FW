#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

PROFILE=""
UPLOAD=""
CLEAN=0

usage() {
  cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Build F407PLC firmware (.elf, .bin, .hex) and optionally flash over DFU or DAPLink.
If options are omitted, an interactive prompt is shown.

Options:
  -p, --profile <dev|debug|release>   Build profile (maps to CMake preset Debug / Release)
  -u, --upload <none|dfu|daplink>     Upload method after build
  -c, --clean                         Clean build directory before configuring/building
  -h, --help                          Show this help message

Examples:
  $(basename "$0")                                  # Interactive mode
  $(basename "$0") --profile release --upload none  # Build Release .bin only
  $(basename "$0") -p dev -u daplink                # Build Debug and flash via DAPLink
  $(basename "$0") -p release -u dfu                # Build Release and flash via USB DFU
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -p|--profile)
      PROFILE="${2:-}"
      shift 2
      ;;
    -u|--upload)
      UPLOAD="${2:-}"
      shift 2
      ;;
    -c|--clean)
      CLEAN=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

# Interactive profile selection if omitted
if [[ -z "${PROFILE}" ]]; then
  echo "Select build profile:"
  echo "  1) dev (Debug)"
  echo "  2) release (Release)"
  read -r -p "Choice [1/2] (default: 1): " p_choice
  case "${p_choice:-1}" in
    2|release|Release|RELEASE) PROFILE="release" ;;
    *) PROFILE="dev" ;;
  esac
fi

case "$(echo "${PROFILE}" | tr '[:upper:]' '[:lower:]')" in
  dev|debug) PRESET="Debug" ;;
  release|rel) PRESET="Release" ;;
  *)
    echo "Error: Invalid profile '${PROFILE}'. Use 'dev' (or 'debug') or 'release'." >&2
    exit 1
    ;;
esac

# Interactive upload selection if omitted
if [[ -z "${UPLOAD}" ]]; then
  echo ""
  echo "Select upload method after build:"
  echo "  1) none (build .bin only)"
  echo "  2) dfu (USB DFU)"
  echo "  3) daplink (DAPLink / CMSIS-DAP)"
  read -r -p "Choice [1/2/3] (default: 1): " u_choice
  case "${u_choice:-1}" in
    2|dfu|DFU) UPLOAD="dfu" ;;
    3|daplink|DAPLink|DAPLINK) UPLOAD="daplink" ;;
    *) UPLOAD="none" ;;
  esac
fi

UPLOAD="$(echo "${UPLOAD}" | tr '[:upper:]' '[:lower:]')"
case "${UPLOAD}" in
  none|dfu|daplink) ;;
  *)
    echo "Error: Invalid upload method '${UPLOAD}'. Use 'none', 'dfu', or 'daplink'." >&2
    exit 1
    ;;
esac

BUILD_DIR="${SCRIPT_DIR}/build/${PRESET}"
ELF_FILE="${BUILD_DIR}/F407PLC.elf"
BIN_FILE="${BUILD_DIR}/F407PLC.bin"

if [[ ${CLEAN} -eq 1 && -d "${BUILD_DIR}" ]]; then
  echo "==> Cleaning ${BUILD_DIR}..."
  rm -rf "${BUILD_DIR}"
fi

if [[ ! -d "${BUILD_DIR}" ]]; then
  echo "==> Configuring CMake preset: ${PRESET}"
  cmake --preset "${PRESET}"
fi

echo "==> Building (${PRESET})..."
cmake --build --preset "${PRESET}" -- -j8

if [[ ! -f "${BIN_FILE}" ]]; then
  echo "Error: Expected .bin not found at ${BIN_FILE}" >&2
  exit 1
fi

BIN_SIZE="$(wc -c < "${BIN_FILE}" | tr -d ' ')"
echo "==> Built ${BIN_FILE} (${BIN_SIZE} bytes)"

if [[ "${UPLOAD}" == "dfu" ]]; then
  if command -v dfu-util >/dev/null 2>&1; then
    echo "==> Waiting for STM32 ROM DFU device (0483:df11)..."
    for _ in $(seq 1 20); do
      if dfu-util -l 2>/dev/null | grep -qi "0483:df11"; then
        break
      fi
      sleep 0.5
    done
    echo "==> Flashing ${BIN_FILE} via dfu-util..."
    dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D "${BIN_FILE}"
  elif command -v STM32_Programmer_CLI >/dev/null 2>&1; then
    echo "==> Flashing ${BIN_FILE} via STM32_Programmer_CLI (USB DFU)..."
    STM32_Programmer_CLI -c port=USB1 -w "${BIN_FILE}" 0x08000000 -v -s
  else
    echo "Error: Neither dfu-util nor STM32_Programmer_CLI found in PATH." >&2
    exit 1
  fi
elif [[ "${UPLOAD}" == "daplink" ]]; then
  echo "==> Flashing ${ELF_FILE} via DAPLink (CMSIS-DAP)..."
  openocd \
    -f interface/cmsis-dap.cfg \
    -f target/stm32f4x.cfg \
    -c "adapter speed 1000" \
    -c "reset_config none" \
    -c "program ${ELF_FILE} verify reset exit"
fi

echo "==> Done!"
