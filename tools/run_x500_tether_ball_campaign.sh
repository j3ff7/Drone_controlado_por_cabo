#!/bin/bash
# PX4 + X500 original + tether por BallJoint (wrapper do launch): estrutura -> estatico ->
# vertical -> horizontal (so se o vertical passar). Resultados em results/x500_tether_ball_shared/.
# Uso: tools/run_x500_tether_ball_campaign.sh [rotulo]
# TIME_SCALE=<s> multiplica as fases da missao e a gravacao. A missao conta tempo de PAREDE e o
# cabo de 72 elos deixa o RTF em ~0,3 (solo) a ~0,5 (voo): com TIME_SCALE=1 o hover de 12 s vira
# ~4-6 s simulados e a subida invade a janela de hover (RMS Z sem sentido). Use TIME_SCALE ~ 1/RTF.
set -u
cd /home/lima/codes/ic/drone-cabo
BASE=results/x500_tether_ball_shared/${1:-baseline}
MODELS=$PWD/build/x500_tether_ball/models
TS=${TIME_SCALE:-1}
sc() { python3 -c "print(round($1*$TS, 1))"; }

limpa() {   # so por nome de executavel: casar padrao na linha de comando mata o proprio shell
  for p in $(ps -eo pid=,comm= | awk '$2=="px4"||$2=="gz"||$2=="ruby"{print $1}'); do kill -INT $p 2>/dev/null; done
  sleep 4
  for p in $(ps -eo pid=,comm= | awk '$2=="px4"||$2=="gz"||$2=="ruby"{print $1}'); do kill -9 $p 2>/dev/null; done
  sleep 2
}

limpa
rm -rf $BASE; mkdir -p $BASE/structure $BASE/static $BASE/vertical $BASE/horizontal
git -C px4/PX4-Autopilot rev-parse HEAD > $BASE/px4_head.txt
echo "TIME_SCALE=$TS" > $BASE/time_scale.txt
git -C px4/PX4-Autopilot status --short | wc -l > $BASE/px4_changes_before.txt

unset GZ_SIM_RESOURCE_PATH PX4_GZ_MODEL_NAME PX4_GZ_MODEL_POSE
export DISPLAY=${DISPLAY:-:1}
( tools/launch_x500_tether_ball.py --headless --model-dir $MODELS 2>&1 \
    | stdbuf -o0 tr '\r' '\n' | stdbuf -o0 uniq > $BASE/px4.log ) < /dev/null &
for i in $(seq 1 90); do grep -q "Ready for takeoff" $BASE/px4.log 2>/dev/null && break; sleep 2; done
sleep 3

# ---- estrutura ----
timeout 8 gz model --list > $BASE/structure/models.txt 2>&1
timeout 15 gz model -m x500_tether_ball_0 > $BASE/structure/model_detail.txt 2>&1
timeout 5 gz topic -l > $BASE/structure/topics.txt 2>&1

# ---- estatico: PX4 de pe, drone pousado ----
tools/record_x500_tether_ball.py --duration 30 --output-dir $BASE/static > $BASE/static/record.log 2>&1
# captura visual: GUI ogre aberta a parte (a GUI ogre2 do PX4 nao aparece a partir de processo em segundo plano)
gz sim -g --render-engine-gui ogre > $BASE/static/gui.log 2>&1 &
GUI=$!
W=""; for i in $(seq 1 30); do W=$(xwininfo -root -tree 2>/dev/null | awk '/"Gazebo Sim"/{print $1; exit}'); [ -n "$W" ] && break; sleep 2; done
if [ -n "$W" ]; then
  sleep 10
  timeout 10 gz service -s /gui/move_to --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 5000 --req 'data: "x500_tether_ball_0"' > /dev/null 2>&1
  sleep 8; import -window "$W" $BASE/static/gazebo.png 2>/dev/null
fi
kill -9 $GUI 2>/dev/null

voo() {  # $1 = fase, $2 = dx
  local OUT=$BASE/$1
  tools/record_x500_tether_ball.py --duration $(sc 75) --output-dir $OUT > $OUT/record.log 2>&1 &
  local REC=$!; sleep 3
  tools/px4_offboard_horizontal_mission.py --output-dir $OUT --dx $2 --altitude 2.0 --rate 20 \
    --prestream $(sc 2) --offboard-settle $(sc 1) --takeoff-hover $(sc 12) --move-hold $(sc 10) \
    --return-hold $(sc 10) --land-stream $(sc 8) > $OUT/mission.log 2>&1
  wait $REC
  tools/analyze_x500_tether_ball_flight.py --run-dir $OUT --px4-log $BASE/px4.log > $OUT/analysis.log 2>&1
  python3 -c "import json,sys; sys.exit(0 if json.load(open('$OUT/flight_metrics.json'))['pass'] else 1)"
}

if voo vertical 0.0; then
  echo PASS > $BASE/vertical/verdict.txt
  if voo horizontal 0.5; then echo PASS > $BASE/horizontal/verdict.txt; else echo FAIL > $BASE/horizontal/verdict.txt; fi
else
  echo FAIL > $BASE/vertical/verdict.txt
  echo "NAO EXECUTADO (vertical falhou)" > $BASE/horizontal/verdict.txt
fi

echo "abortos=$(grep -cE 'Assertion|Aborted' $BASE/px4.log) failsafe=$(grep -ciE 'failsafe' $BASE/px4.log) ready=$(grep -c 'Ready for takeoff' $BASE/px4.log)" > $BASE/status.txt
limpa
git -C px4/PX4-Autopilot status --short | wc -l > $BASE/px4_changes_after.txt
echo DONE > $BASE/.done
