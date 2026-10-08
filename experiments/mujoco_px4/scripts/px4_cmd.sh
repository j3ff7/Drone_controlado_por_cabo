#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BIN="${ROOT}/px4/PX4-Autopilot/build/px4_sitl_default/bin"
ROOTFS="${ROOT}/experiments/mujoco_px4/runtime/rootfs"
[[ $# -gt 0 ]] || { echo "usage: $0 COMMAND [ARGS...]" >&2; exit 2; }
cd "${ROOTFS}"
exec "${BIN}/px4-$1" "${@:2}"
