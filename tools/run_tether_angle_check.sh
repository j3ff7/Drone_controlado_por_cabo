#!/bin/bash
# Conexao por plugin de forca (baseline das rodadas A) com a medicao nova de tangente e
# angulos junto ao UAV: estatico (PX4 de pe, sem voo) e voo vertical curto.
# Uso: tools/run_tether_angle_check.sh [rotulo]        (resultados em results/angles/<rotulo>)
set -u
cd /home/lima/codes/ic/drone-cabo
BASE=results/angles/${1:-plugin_only}
N=5; L=2.5; SEG=0.5

limpa() {   # so por nome de executavel: casar padroes na linha de comando mata o proprio shell
  for p in $(ps -eo pid=,comm= | awk '$2=="px4"||$2=="gz"||$2=="ruby"{print $1}'); do kill -INT $p 2>/dev/null; done
  sleep 4
  for p in $(ps -eo pid=,comm= | awk '$2=="px4"||$2=="gz"||$2=="ruby"{print $1}'); do kill -9 $p 2>/dev/null; done
  sleep 2
}

limpa
rm -rf $BASE; mkdir -p $BASE/static $BASE/vertical
./tools/generate_tether_anchor_chain.py --links $N --length $L --rho 0.06 --radius 0.003 \
  --initial-axis folded_ground --force-constraint --stiffness 5 --damping 0.5 --max-force 3 \
  --no-reel-actuator > $BASE/model.txt
export GZ_SIM_RESOURCE_PATH=$PWD/src/pacote_do_drone/models GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/build/gz_plugins
( cd px4/PX4-Autopilot && PX4_GZ_MODEL=x500_tether_attach HEADLESS=1 make px4_sitl gz_x500 2>&1 \
    | stdbuf -o0 tr '\r' '\n' | stdbuf -o0 uniq > ../../$BASE/px4.log ) < /dev/null &
for i in $(seq 1 75); do grep -q "Ready for takeoff" $BASE/px4.log 2>/dev/null && break; sleep 2; done
timeout 15 gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
  --timeout 10000 --req "sdf_filename: \"$PWD/src/pacote_do_drone/models/tether_anchor_chain/model.sdf\" name: \"tether_anchor_chain\" allow_renaming: false pose: {position: {z: 0}}" > $BASE/spawn.txt
timeout 8 gz model --list 2>/dev/null > $BASE/models.txt
sleep 5

# ---- estatico: PX4 de pe, drone pousado ----
./tools/record_tether_timeseries.py --duration 30 --output-dir $BASE/static --prefix plugin > $BASE/static/ts.log 2>&1 &
R0=$!
./tools/record_tether_connection.py --duration 30 --output-dir $BASE/static --prefix connection --links $N --segment-length $SEG --settle 5 > $BASE/static/pose.log 2>&1
wait $R0   # nunca `wait` sem PID: ele esperaria tambem o PX4, que nao termina

# ---- vertical curto ----
./tools/record_tether_timeseries.py --duration 70 --output-dir $BASE/vertical --prefix plugin > $BASE/vertical/ts.log 2>&1 &
R1=$!
./tools/record_tether_connection.py --duration 70 --output-dir $BASE/vertical --prefix connection --links $N --segment-length $SEG --settle 5 > $BASE/vertical/pose.log 2>&1 &
R2=$!; sleep 3
./tools/px4_offboard_horizontal_mission.py --output-dir $BASE/vertical --dx 0.0 --altitude 2.0 --rate 20 > $BASE/vertical/mission.log 2>&1
wait $R1 $R2

echo "abortos=$(grep -cE 'Assertion|Aborted' $BASE/px4.log) failsafe=$(grep -ciE 'failsafe' $BASE/px4.log)" > $BASE/aborts.txt
limpa
echo DONE > $BASE/.done
