#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
EXP="${ROOT}/experiments/mujoco_px4"
MODEL="${EXP}/models/x500_tether_n70.xml"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${EXP}/results/${STAMP}_m4_horizontal_n70"
mkdir -p "${OUT}"
RUNNER_PID=""; RECORDER_PID=""
cleanup() {
  trap - INT TERM EXIT
  [[ -z "${RECORDER_PID}" ]] || kill "${RECORDER_PID}" 2>/dev/null || true
  [[ -z "${RUNNER_PID}" ]] || kill -TERM "${RUNNER_PID}" 2>/dev/null || true
  [[ -z "${RUNNER_PID}" ]] || wait "${RUNNER_PID}" 2>/dev/null || true
}
trap cleanup INT TERM EXIT

printf '%s\n' \
  "date=$(date --iso-8601=seconds)" \
  "project_commit=$(git -C "${ROOT}" rev-parse HEAD)" \
  "px4_commit=$(git -C "${ROOT}/px4/PX4-Autopilot" rev-parse HEAD)" \
  "mujoco_version=$("${EXP}/.venv/bin/python" -c 'import mujoco; print(mujoco.__version__)')" \
  "configuration=N=70,L=2.5,rho=0.06,anchored=true,physics_rate=1000Hz,dx=0.5m" \
  "command=bash experiments/mujoco_px4/scripts/run_m4_n70.sh" >"${OUT}/metadata.txt"

bash "${EXP}/scripts/run_sitl.sh" --model "${MODEL}" --physics-rate 1000 >"${OUT}/sitl.log" 2>&1 &
RUNNER_PID=$!
ready=0
for _ in {1..180}; do
  if grep -q "Ready for takeoff" "${OUT}/sitl.log"; then ready=1; break; fi
  kill -0 "${RUNNER_PID}" 2>/dev/null || break
  sleep 1
done
((ready)) || { echo FAIL >"${OUT}/result.txt"; tail -100 "${OUT}/sitl.log"; exit 1; }

"${EXP}/.venv/bin/python" "${EXP}/scripts/record_sidechannel.py" \
  --duration 440 --output "${OUT}" >"${OUT}/recorder.log" 2>&1 &
RECORDER_PID=$!
sleep 2
{
  "${EXP}/scripts/px4_cmd.sh" param set MIS_TAKEOFF_ALT 1.0
  "${EXP}/scripts/px4_cmd.sh" commander arm
  sleep 2
  "${EXP}/scripts/px4_cmd.sh" commander takeoff
  sleep 150
  "${EXP}/.venv/bin/python" "${EXP}/scripts/offboard_displacement.py" \
    --dx 0.5 --timeout 90 --output "${OUT}/displacement.json"
  "${EXP}/scripts/px4_cmd.sh" commander land
  sleep 100
  "${EXP}/scripts/px4_cmd.sh" commander status
} >"${OUT}/px4_commands.log" 2>&1
wait "${RECORDER_PID}"; RECORDER_PID=""
kill -TERM "${RUNNER_PID}" 2>/dev/null || true
wait "${RUNNER_PID}" 2>/dev/null || true; RUNNER_PID=""

"${EXP}/.venv/bin/python" - "${OUT}" <<'PY'
import json, sys
from pathlib import Path
out=Path(sys.argv[1])
m=json.loads((out/'metrics.json').read_text())
d=json.loads((out/'displacement.json').read_text())
sitl=(out/'sitl.log').read_text(errors='replace')
passed=(d['target_reached'] and d['return_reached'] and abs(d['achieved_dx_m']-.5)<.15
        and d['return_error_m']<.15 and not m['nan_detected']
        and (m['hover_roll_max_abs_deg'] or 999)<15
        and (m['hover_pitch_max_abs_deg'] or 999)<15
        and m['tether_connection_error_max_m']<.02
        and m['tether_anchor_drift_max_m']<1e-9
        and 'Landing detected' in sitl and 'Disarmed by landing' in sitl
        and 'Critical failure detected' not in sitl)
(out/'result.txt').write_text('PASS\n' if passed else 'FAIL\n')
print('PASS' if passed else 'FAIL')
print(json.dumps(m,indent=2))
print(json.dumps(d,indent=2))
raise SystemExit(0 if passed else 1)
PY
echo "results=${OUT}"
