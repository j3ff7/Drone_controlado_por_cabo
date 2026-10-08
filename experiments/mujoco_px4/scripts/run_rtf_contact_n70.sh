#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
EXP="${ROOT}/experiments/mujoco_px4"
MODEL="${EXP}/models/x500_tether_n70.xml"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${EXP}/results/${STAMP}_rtf_contact_n70"
STOP_FILE="${OUT}/stop_recorder"
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
  "configuration=N=70,L=2.5,rho=0.06,physics_rate=1000Hz,staged_descent" \
  "command=bash experiments/mujoco_px4/scripts/run_rtf_contact_n70.sh" >"${OUT}/metadata.txt"

bash "${EXP}/scripts/run_sitl.sh" --model "${MODEL}" --physics-rate 1000 \
  >"${OUT}/sitl.log" 2>&1 &
RUNNER_PID=$!
ready=0
for _ in {1..240}; do
  if grep -q "Ready for takeoff" "${OUT}/sitl.log"; then ready=1; break; fi
  kill -0 "${RUNNER_PID}" 2>/dev/null || break
  sleep 1
done
((ready)) || { echo FAIL >"${OUT}/result.txt"; tail -100 "${OUT}/sitl.log"; exit 1; }

"${EXP}/.venv/bin/python" "${EXP}/scripts/record_sidechannel.py" \
  --duration 1800 --stop-file "${STOP_FILE}" --output "${OUT}" \
  >"${OUT}/recorder.log" 2>&1 &
RECORDER_PID=$!
sleep 2

{
  "${EXP}/scripts/px4_cmd.sh" param set MIS_TAKEOFF_ALT 2.0
  "${EXP}/scripts/px4_cmd.sh" commander arm
  sleep 2
  "${EXP}/scripts/px4_cmd.sh" commander takeoff
  "${EXP}/.venv/bin/python" "${EXP}/scripts/wait_altitude.py" \
    --minimum 1.7 --timeout 360
} >"${OUT}/takeoff.log" 2>&1

set +e
"${EXP}/.venv/bin/python" "${EXP}/scripts/staged_descent.py" \
  --drops 0,0.35,0.70,1.05,1.40 --hold-sim 1.5 --timeout-wall 180 \
  --output "${OUT}/stages.json" >"${OUT}/descent.log" 2>&1
PROFILE_STATUS=$?
set -e

"${EXP}/scripts/px4_cmd.sh" commander land >>"${OUT}/takeoff.log" 2>&1 || true
for _ in {1..240}; do
  grep -q "Disarmed by landing" "${OUT}/sitl.log" && break
  sleep 1
done
: >"${STOP_FILE}"
wait "${RECORDER_PID}"; RECORDER_PID=""

set +e
"${EXP}/.venv/bin/python" "${EXP}/scripts/analyze_rtf_contacts.py" \
  "${OUT}/ground_truth.csv" "${OUT}/stages.json" --output "${OUT}/contact_rtf.json" \
  | tee "${OUT}/analysis.log"
ANALYSIS_STATUS=$?
set -e

kill -TERM "${RUNNER_PID}" 2>/dev/null || true
wait "${RUNNER_PID}" 2>/dev/null || true; RUNNER_PID=""
if ((PROFILE_STATUS == 0 && ANALYSIS_STATUS == 0)); then
  echo PASS | tee "${OUT}/result.txt"
else
  echo FAIL | tee "${OUT}/result.txt"
  exit 1
fi
echo "results=${OUT}"
