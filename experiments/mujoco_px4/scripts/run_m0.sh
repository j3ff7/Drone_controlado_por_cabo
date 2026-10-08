#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
EXP="${ROOT}/experiments/mujoco_px4"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${EXP}/results/${STAMP}_m0_x500"
mkdir -p "${OUT}"

RUNNER_PID=""
RECORDER_PID=""
cleanup() {
  trap - INT TERM EXIT
  [[ -z "${RECORDER_PID}" ]] || kill "${RECORDER_PID}" 2>/dev/null || true
  [[ -z "${RUNNER_PID}" ]] || kill -TERM "${RUNNER_PID}" 2>/dev/null || true
  [[ -z "${RUNNER_PID}" ]] || wait "${RUNNER_PID}" 2>/dev/null || true
}
trap cleanup INT TERM EXIT

cat >"${OUT}/metadata.txt" <<EOF
date=$(date --iso-8601=seconds)
project_commit=$(git -C "${ROOT}" rev-parse HEAD)
px4_commit=$(git -C "${ROOT}/px4/PX4-Autopilot" rev-parse HEAD)
px4_version=$(git -C "${ROOT}/px4/PX4-Autopilot" describe --always --tags --dirty)
mujoco_version=$("${EXP}/.venv/bin/python" -c 'import mujoco; print(mujoco.__version__)')
command=bash experiments/mujoco_px4/scripts/run_m0.sh
model=models/x500.xml
rotors=configs/x500_rotors.yaml
EOF

bash "${EXP}/scripts/run_sitl.sh" >"${OUT}/sitl.log" 2>&1 &
RUNNER_PID=$!

ready=0
for _ in {1..30}; do
  if grep -q "Ready for takeoff" "${OUT}/sitl.log"; then ready=1; break; fi
  kill -0 "${RUNNER_PID}" 2>/dev/null || break
  sleep 1
done
if (( ! ready )); then
  echo "FAIL: PX4 did not become ready" | tee "${OUT}/result.txt"
  tail -100 "${OUT}/sitl.log"
  exit 1
fi

"${EXP}/.venv/bin/python" "${EXP}/scripts/record_sidechannel.py" \
  --duration 32 --output "${OUT}" >"${OUT}/recorder.log" 2>&1 &
RECORDER_PID=$!
sleep 1

{
  echo '--- preflight ---'
  "${EXP}/scripts/px4_cmd.sh" commander status
  echo '--- arm ---'
  "${EXP}/scripts/px4_cmd.sh" commander arm
  sleep 1
  echo '--- takeoff ---'
  "${EXP}/scripts/px4_cmd.sh" commander takeoff
  sleep 12
  echo '--- hover position ---'
  "${EXP}/scripts/px4_cmd.sh" listener vehicle_local_position -n 1
  echo '--- hover attitude ---'
  "${EXP}/scripts/px4_cmd.sh" listener vehicle_attitude -n 1
  echo '--- land ---'
  "${EXP}/scripts/px4_cmd.sh" commander land
  sleep 12
  echo '--- final status ---'
  "${EXP}/scripts/px4_cmd.sh" commander status
} >"${OUT}/px4_commands.log" 2>&1

wait "${RECORDER_PID}"
RECORDER_PID=""
kill -TERM "${RUNNER_PID}" 2>/dev/null || true
wait "${RUNNER_PID}" 2>/dev/null || true
RUNNER_PID=""

"${EXP}/.venv/bin/python" - "${OUT}" <<'PY'
import json, sys
from pathlib import Path
out = Path(sys.argv[1])
m = json.loads((out / "metrics.json").read_text())
commands = (out / "px4_commands.log").read_text(errors="replace")
sitl = (out / "sitl.log").read_text(errors="replace")
passed = (
    m["max_altitude_m"] > 1.0
    and m["hover_samples"] > 100
    and not m["nan_detected"]
    and (m["hover_roll_max_abs_deg"] or 999) < 15
    and (m["hover_pitch_max_abs_deg"] or 999) < 15
    and "Armed by internal command" in sitl
    and "Landing detected" in sitl
    and "Critical failure detected" not in sitl
)
text = "PASS\n" if passed else "FAIL\n"
(out / "result.txt").write_text(text)
print(text.strip())
print(json.dumps(m, indent=2))
raise SystemExit(0 if passed else 1)
PY

echo "results=${OUT}"
