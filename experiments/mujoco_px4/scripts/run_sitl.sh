#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
EXP="${ROOT}/experiments/mujoco_px4"
PX4_DIR="${ROOT}/px4/PX4-Autopilot"
RUNTIME="${EXP}/runtime"
PYTHON="${EXP}/.venv/bin/python"
INSTANCE="${PX4_INSTANCE:-0}"
VIEWER=0
MAX_SIM_TIME=""
MODEL="${EXP}/models/x500.xml"
PHYSICS_RATE="1000"

usage() {
  printf 'Usage: %s [--viewer] [--model FILE] [--physics-rate HZ] [--max-sim-time SEC]\n' "$0"
}
while (($#)); do
  case "$1" in
    --viewer) VIEWER=1 ;;
    --model) shift; MODEL="$1" ;;
    --physics-rate) shift; PHYSICS_RATE="$1" ;;
    --max-sim-time) shift; MAX_SIM_TIME="$1" ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

[[ -x "${PYTHON}" ]] || { echo "run scripts/prepare.sh first" >&2; exit 1; }
[[ -d "${RUNTIME}/etc" ]] || { echo "run scripts/prepare.sh first" >&2; exit 1; }

SIM_ARGS=(--instance "${INSTANCE}"
  --model "${MODEL}"
  --rotors "${EXP}/configs/x500_rotors.yaml"
  --imu ideal --physics-rate "${PHYSICS_RATE}")
((VIEWER)) && SIM_ARGS+=(--viewer)
[[ -n "${MAX_SIM_TIME}" ]] && SIM_ARGS+=(--max-sim-time "${MAX_SIM_TIME}")

SIM_PID=""
PX4_PID=""
stop_pid() {
  local pid="$1"
  [[ -n "${pid}" ]] || return 0
  kill -TERM "${pid}" 2>/dev/null || true
  for _ in {1..30}; do
    kill -0 "${pid}" 2>/dev/null || break
    sleep 0.1
  done
  kill -KILL "${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
}
cleanup() {
  trap - INT TERM EXIT
  stop_pid "${PX4_PID}"
  stop_pid "${SIM_PID}"
}
trap cleanup INT TERM EXIT

PYTHONPATH="${EXP}/bridge" "${PYTHON}" -m mujoco_px4_sitl "${SIM_ARGS[@]}" &
SIM_PID=$!
sleep 0.5
kill -0 "${SIM_PID}" 2>/dev/null || { echo "MuJoCo bridge exited during startup" >&2; exit 1; }

mkdir -p "${RUNTIME}/rootfs"
rm -f "${RUNTIME}/rootfs/px4_instance_${INSTANCE}.pid"
(
  cd "${RUNTIME}/rootfs"
  PX4_SYS_AUTOSTART=22010 exec \
    "${PX4_DIR}/build/px4_sitl_default/bin/px4" -d -i "${INSTANCE}" "${RUNTIME}/etc"
) &
PX4_PID=$!

while kill -0 "${SIM_PID}" 2>/dev/null && kill -0 "${PX4_PID}" 2>/dev/null; do
  sleep 0.5
done
