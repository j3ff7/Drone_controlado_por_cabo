#!/bin/bash
# Conexao por plugin de forca + medicao angular no frame do drone + colisao tether-solo.
#
#   tools/run_tether_angle_collision_campaign.sh collision  [rotulo] [args do gerador...]   C0 -> C1 -> C2
#   tools/run_tether_angle_collision_campaign.sh integrated [rotulo] [args do gerador...]  S0 -> S1 -> S2
#
# Cabo da baseline de forca (N=5, L=2,5 m, folded_ground, K=5, C=0,5, Fmax=3, reel sem atuador)
# COM colisao nos elos. O modelo e gerado em results/ (o arquivo versionado nao e tocado). O X500
# e spawnado pelo PX4 com PX4_GZ_MODEL_POSE=0,0,0.24: sem isso o PX4 spawna em z=0, o que
# sobrescreve a <pose>0 0 .24 do modelo e deixa o drone afundado com o attach abaixo do solo.
# Voos com altitude relativa ao ponto de decolagem (--relative-altitude): a origem do EKF ja
# nasceu 1,14 m deslocada, o que mandava o drone alem do alcance do cabo (2,5 m).
# Cada etapa so roda se a anterior passou. Resultados em results/tether_angles/<modo>/<rotulo>/.
set -u
cd /home/lima/codes/ic/drone-cabo
MODE=${1:?modo: collision | integrated}; LABEL=${2:-baseline}; shift 2 2>/dev/null || shift $#
case "$MODE" in   # nomes antigos aceitos como alias
  colisao) MODE=collision ;;
  integrado) MODE=integrated ;;
  collision|integrated) ;;
  *) echo "modo invalido: $MODE (use collision ou integrated)" >&2; exit 2 ;;
esac
EXTRA_GEN=("$@")
BASE=results/tether_angles/$MODE/$LABEL
N=5; L=2.5; SEG=0.5; RADIUS=0.003
# raio da colisao dos elos: o do gerador (0,75 * raio visual) ou o passado em --collision-radius
COLLISION_RADIUS=$(python3 -c "print(0.75 * $RADIUS)")
prev=""
for a in "${EXTRA_GEN[@]}"; do [ "$prev" = --collision-radius ] && COLLISION_RADIUS=$a; prev=$a; done
DRONE_POSE=${DRONE_POSE:-0,0,0.24,0,0,0}
# Filtro de colisao cabo x drone (opcional): DRONE_MASK=1 CABLE_MASK=2 gera uma copia do
# x500_tether_attach com <collide_bitmask> 1 (x500_tether_attach_mask, em $BASE/drone_models)
# e o cabo com 2; o solo fica no padrao 65535 e colide com os dois.
DRONE_MASK=${DRONE_MASK:-}; CABLE_MASK=${CABLE_MASK:-}
# Passo de fisica (opcional): STEP=0.002 serve uma COPIA do mundo default do PX4 (gerada em
# $BASE/world) com max_step_size = STEP e real_time_update_rate = 1/STEP. O servidor sobe antes
# do PX4, que se liga a um mundo ja rodando (px4-rc.simulator); o arquivo do PX4 nao e editado.
STEP=${STEP:-}
DRONE_MODEL=x500_tether_attach
[ -n "$DRONE_MASK" ] && DRONE_MODEL=x500_tether_attach_mask

limpa() {   # so por nome de executavel: casar padrao na linha de comando mata o proprio shell
  for p in $(ps -eo pid=,comm= | awk '$2=="px4"||$2=="gz"||$2=="ruby"{print $1}'); do kill -INT $p 2>/dev/null; done
  sleep 4
  for p in $(ps -eo pid=,comm= | awk '$2=="px4"||$2=="gz"||$2=="ruby"{print $1}'); do kill -9 $p 2>/dev/null; done
  sleep 2
}

limpa
rm -rf $BASE; mkdir -p $BASE/model
git -C px4/PX4-Autopilot rev-parse HEAD > $BASE/px4_head.txt
git -C px4/PX4-Autopilot status --short | wc -l > $BASE/px4_changes_before.txt
GEN_ARGS=(--links $N --length $L --rho 0.06 --radius $RADIUS --initial-axis folded_ground
          --force-constraint --stiffness 5 --damping 0.5 --max-force 3 --no-reel-actuator
          --link-collisions --output-dir $BASE/model --drone-model ${DRONE_MODEL}_0)
[ -n "$CABLE_MASK" ] && GEN_ARGS+=(--collide-bitmask $CABLE_MASK)
if [ -n "$DRONE_MASK" ]; then
  mkdir -p $BASE/drone_models
  ./tools/generate_x500_tether_attach.py --model-name $DRONE_MODEL --collide-bitmask $DRONE_MASK \
    --output-dir $BASE/drone_models/$DRONE_MODEL > $BASE/drone_model.txt || { echo "gerador do drone falhou" > $BASE/verdict.txt; exit 1; }
fi
GEN_ARGS+=("${EXTRA_GEN[@]}")
./tools/generate_tether_anchor_chain.py "${GEN_ARGS[@]}" > $BASE/model.txt || { echo "gerador falhou" > $BASE/verdict.txt; exit 1; }
printf '%s\n' "${GEN_ARGS[@]}" > $BASE/generator_args.txt
echo "collision_radius=$COLLISION_RADIUS drone_pose=$DRONE_POSE drone_model=$DRONE_MODEL drone_mask=${DRONE_MASK:-65535} cable_mask=${CABLE_MASK:-65535} step=${STEP:-0.004}" >> $BASE/model.txt

export GZ_SIM_RESOURCE_PATH=$PWD/src/pacote_do_drone/models GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/build/gz_plugins
[ -n "$DRONE_MASK" ] && export GZ_SIM_RESOURCE_PATH=$PWD/$BASE/drone_models:$GZ_SIM_RESOURCE_PATH
unset PX4_GZ_MODEL_NAME
LOGS="$BASE/px4.log"
if [ -n "$STEP" ]; then
  mkdir -p $BASE/world
  ./tools/generate_px4_world_step.py --step "$STEP" --output $BASE/world/default.sdf > $BASE/world.txt
  ( GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$PWD/px4/PX4-Autopilot/Tools/simulation/gz/models:$PWD/px4/PX4-Autopilot/Tools/simulation/gz/worlds \
      gz sim --verbose=1 -r -s $BASE/world/default.sdf > $BASE/gz.log 2>&1 ) < /dev/null &
  for i in $(seq 1 30); do timeout 5 gz topic -l 2>/dev/null | grep -q "/world/default/clock" && break; sleep 1; done
  LOGS="$LOGS $BASE/gz.log"
fi
( cd px4/PX4-Autopilot && PX4_GZ_MODEL=$DRONE_MODEL PX4_GZ_MODEL_POSE=$DRONE_POSE HEADLESS=1 \
    make px4_sitl gz_x500 2>&1 | stdbuf -o0 tr '\r' '\n' | stdbuf -o0 uniq > ../../$BASE/px4.log ) < /dev/null &
for i in $(seq 1 90); do grep -q "Ready for takeoff" $BASE/px4.log 2>/dev/null && break; sleep 2; done
timeout 15 gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
  --timeout 10000 --req "sdf_filename: \"$PWD/$BASE/model/model.sdf\" name: \"tether_anchor_chain\" allow_renaming: false pose: {position: {z: 0}}" > $BASE/spawn.txt 2>&1
timeout 8 gz model --list > $BASE/models.txt 2>&1
sleep 5

# gravadores: topicos do plugin + poses com 3 janelas de tangente (T2) + penetracao no solo
grava() {  # $1 = dir, $2 = duracao
  mkdir -p $1
  ./tools/record_tether_timeseries.py --duration $2 --output-dir $1 --prefix plugin > $1/ts.log 2>&1 &
  RECS="$!"
  for w in 0.15 0.5 1.0; do
    ./tools/record_tether_connection.py --duration $2 --output-dir $1 --prefix connection_w$w --links $N \
      --segment-length $SEG --settle 5 --tangent-window $w --collision-radius $COLLISION_RADIUS \
      --drone-model ${DRONE_MODEL}_0 > $1/pose_w$w.log 2>&1 &
    RECS="$RECS $!"
  done
}

estatico() {  # $1 = dir
  grava $BASE/$1 30
  wait $RECS
  ./tools/analyze_tether_angle_flight.py --run-dir $BASE/$1 --px4-log $LOGS --static > $BASE/$1/analysis.log 2>&1
}

voo() {  # $1 = dir, $2 = duracao da gravacao, demais = argumentos da missao
  local D=$1 T=$2; shift 2
  grava $BASE/$D $T
  sleep 3
  ./tools/px4_offboard_horizontal_mission.py --output-dir $BASE/$D --rate 20 "$@" > $BASE/$D/mission.log 2>&1
  wait $RECS
  ./tools/analyze_tether_angle_flight.py --run-dir $BASE/$D --px4-log $LOGS > $BASE/$D/analysis.log 2>&1
}

passou() { python3 -c "import json,sys; sys.exit(0 if json.load(open('$BASE/$1/metrics.json'))['pass'] else 1)" 2>/dev/null; }

if [ "$MODE" = collision ]; then
  STAGES=(C0_rest C1_drag C2_landing)
  estatico C0_rest
  if passou C0_rest; then
    # arraste lento: 1 m a 0,8 m acima do ponto de decolagem, rampa de 0,1 m/s (x local do PX4 =
    # norte = +y do Gazebo). Com 0,8 m parte do cabo continua no solo e e arrastada.
    voo C1_drag 80 --dx 1.0 --altitude 0.8 --relative-altitude --xy-speed 0.1 --takeoff-hover 12 --move-hold 18 --return-hold 18 --land-stream 10
  fi
  if passou C1_drag; then
    voo C2_landing 70 --dx 0.0 --altitude 1.8 --relative-altitude
  fi
else
  STAGES=(S0_static S1_vertical S2_horizontal)
  estatico S0_static
  if passou S0_static; then
    voo S1_vertical 70 --dx 0.0 --altitude 1.8 --relative-altitude
  fi
  if passou S1_vertical; then
    voo S2_horizontal 70 --dx 0.5 --altitude 1.8 --relative-altitude
  fi
fi

for s in "${STAGES[@]}"; do
  if [ -f $BASE/$s/metrics.json ]; then
    passou $s && echo "$s PASS" || echo "$s FAIL"
  else
    echo "$s NAO EXECUTADO"
  fi
done > $BASE/verdict.txt
echo "abortos=$(cat $LOGS | grep -cE 'Assertion|Aborted|assertion') failsafe=$(cat $LOGS | grep -ciE 'failsafe') passo=${STEP:-0.004}" > $BASE/status.txt
grep -m1 -o "max_step_size>[^<]*" ${BASE}/world/default.sdf 2>/dev/null >> $BASE/status.txt
limpa
git -C px4/PX4-Autopilot status --short | wc -l > $BASE/px4_changes_after.txt
echo DONE > $BASE/.done
