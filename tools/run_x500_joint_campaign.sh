#!/bin/bash
# Estatico -> vertical -> horizontal (se vertical passar) com o tether ligado ao X500 por juntas fisicas.
# Uso: tools/run_x500_joint_campaign.sh [distancia_m] [rotulo] [args extras do gerador de mundo]
#      ex.: tools/run_x500_joint_campaign.sh 0.5 D0p5_sem_colisao --no-collisions
set -u
cd /home/lima/codes/ic/drone-cabo
D=${1:-0.5}
TAG=${2:-D$(echo $D | tr . p)}
shift $(( $# < 2 ? $# : 2 ))
EXTRA=("$@")
BASE=results/x500_joint/$TAG
export GZ_SIM_RESOURCE_PATH=$PWD/src/pacote_do_drone/models:$PWD/px4/PX4-Autopilot/Tools/simulation/gz/models
W=$PWD/src/pacote_do_drone/worlds/x500_tether_joint.sdf

limpa() {
  pkill -f "px4_sitl_default/bin/px4"; pkill -f "make px4_sitl"; pkill -f "gz sim -s -r -v 3"
  sleep 3
  for p in $(ps -eo pid=,comm= | awk '$2=="px4"||$2=="gz"||$2=="ruby"{print $1}'); do kill -9 $p 2>/dev/null; done
  sleep 2
}
sobe_gz() {
  gz sim -s -r -v 3 $W > $1/gz.log 2>&1 &
  for i in $(seq 1 40); do timeout 4 gz topic -l 2>/dev/null | grep -q "/world/default/clock" && return 0; sleep 1; done
  return 1
}

limpa
mkdir -p $BASE
./tools/generate_x500_tether_world.py --drone-x $D "${EXTRA[@]}" > $BASE/mundo.txt

# ---- estatico (sem PX4) ----
OUT=$BASE/estatico; mkdir -p $OUT
sobe_gz $OUT
timeout 8 gz model --list 2>/dev/null > $OUT/modelos.txt
./tools/record_tether_connection.py --duration 45 --output-dir $OUT --links 10 --segment-length 0.25 --settle 15 > $OUT/record.log 2>&1
echo "abortos=$(grep -cE 'Assertion|Aborted' $OUT/gz.log)" > $OUT/abortos.txt
limpa

voo() {  # $1 = nome, $2 = dx
  local OUT=$BASE/$1; mkdir -p $OUT
  # Sem o mundo vivo o PX4 sobe o proprio `default`, sem o X500, e a missao so expira:
  # o voo nao seria do cenario sob teste. Recusa em vez de gerar um resultado falso.
  if ! sobe_gz $OUT || sleep 5 && ! timeout 4 gz topic -l 2>/dev/null | grep -q "/world/default/clock"; then
    echo "NAO EXECUTADO: o mundo morreu antes do PX4 (ver gz.log)" > $OUT/nao_executado.txt
    echo "abortos=$(grep -cE 'Assertion|Aborted' $OUT/gz.log) failsafe=0" > $OUT/abortos.txt
    limpa
    return 1
  fi
  ( cd px4/PX4-Autopilot && PX4_GZ_MODEL_NAME=x500_tether_attach_0 HEADLESS=1 make px4_sitl gz_x500 2>&1 \
      | stdbuf -o0 tr '\r' '\n' | stdbuf -o0 uniq > ../../$OUT/px4.log ) < /dev/null &
  for i in $(seq 1 60); do grep -q "Ready for takeoff" $OUT/px4.log 2>/dev/null && break; sleep 2; done
  ./tools/record_tether_connection.py --duration 75 --output-dir $OUT --links 10 --segment-length 0.25 --settle 5 > $OUT/record.log 2>&1 &
  local REC=$!; sleep 3
  ./tools/px4_offboard_horizontal_mission.py --output-dir $OUT --dx $2 --altitude 2.0 --rate 20 > $OUT/missao.log 2>&1
  wait $REC
  echo "abortos=$(grep -cE 'Assertion|Aborted' $OUT/gz.log) failsafe=$(grep -ciE 'failsafe' $OUT/px4.log)" > $OUT/abortos.txt
  limpa
}

passou() {  # criterios objetivos: a ferramenta de missao nao detecta simulador morto
  python3 - "$1" <<'PY'
import json, sys, pathlib
d = pathlib.Path(sys.argv[1])
s = json.loads((d / 'conexao_summary.json').read_text())
ab = dict(kv.split('=') for kv in (d / 'abortos.txt').read_text().split())
m = json.loads((d / 'px4_offboard_horizontal_mission.json').read_text())
ok = (s.get('sim_time_covered_s', 0) >= 0.9 * 72 and int(ab['abortos']) == 0 and int(ab.get('failsafe', 0)) == 0
      and s['pivot_gap_m']['max'] < 1e-3 and s['roll_max_abs_deg'] < 30 and s['pitch_max_abs_deg'] < 30
      and not m.get('failed'))
print('PASS' if ok else 'FAIL')
sys.exit(0 if ok else 1)
PY
}

if voo vertical 0.0; then
  passou $BASE/vertical > $BASE/vertical/veredito.txt
else
  echo "FAIL (nao executado: mundo morreu)" > $BASE/vertical/veredito.txt
fi
if [ "$(cat $BASE/vertical/veredito.txt)" = "PASS" ]; then
  if voo horizontal 0.5; then
    passou $BASE/horizontal > $BASE/horizontal/veredito.txt
  else
    echo "FAIL (nao executado: mundo morreu)" > $BASE/horizontal/veredito.txt
  fi
else
  echo "NAO EXECUTADO (vertical falhou)" > $BASE/horizontal_nao_executado.txt
fi
echo DONE > $BASE/.done
