#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
EXP="${ROOT}/experiments/mujoco_px4"
PX4_DIR="${ROOT}/px4/PX4-Autopilot"
VENV="${EXP}/.venv"
RUNTIME="${EXP}/runtime"

if [[ ! -x "${VENV}/bin/python" ]]; then
  python3 -m venv "${VENV}"
fi
"${VENV}/bin/python" -m pip install -r "${EXP}/requirements.txt"

if [[ ! -x "${PX4_DIR}/build/px4_sitl_default/bin/px4" ]]; then
  cmake --build "${PX4_DIR}/build/px4_sitl_default" --target px4
fi

rm -rf "${RUNTIME}/etc"
mkdir -p "${RUNTIME}/rootfs"
cp -a "${PX4_DIR}/build/px4_sitl_default/etc" "${RUNTIME}/etc"
install -m 0755 "${EXP}/configs/22010_mujoco_x500" \
  "${RUNTIME}/etc/init.d-posix/airframes/22010_mujoco_x500"
install -m 0755 "${EXP}/configs/22010_mujoco_x500.post" \
  "${RUNTIME}/etc/init.d-posix/airframes/22010_mujoco_x500.post"

printf '%s\n' \
  "MuJoCo: $("${VENV}/bin/python" -c 'import mujoco; print(mujoco.__version__)')" \
  "PX4: $(git -C "${PX4_DIR}" describe --always --tags --dirty)" \
  "Runtime: ${RUNTIME}"
