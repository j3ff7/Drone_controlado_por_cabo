#!/bin/bash
# T0/T1 — bancada de compensacao de atitude, sem PX4.
#
# Um corpo ESTATICO (`attitude_body`, link `tether_attach_link` na origem do corpo) faz o papel do
# drone. O cabo da baseline de forca (N=5, L=2,5 m, constraint K=5, C=0,5, Fmax=3) nasce esticado
# da guia da estacao ate o corpo e assenta pendurado. Cada caso roda uma sessao nova do Gazebo com o
# corpo numa atitude fixa. Como o ponto de conexao e a origem do corpo, girar o corpo NAO muda a
# geometria do cabo: t_hat_world deve ficar igual entre os casos e t_hat_body deve ser R_BW t_hat_world.
#
# Uso: tools/run_tether_attitude_bench.sh [rotulo] [casos...]
#      caso = nome:roll:pitch:yaw (graus); padrao = nivelado, roll +-15, pitch +-15, yaw 45, combinado
# Resultados: results/tether_angles/bench/<rotulo>/<caso>/
set -u
cd /home/lima/codes/ic/drone-cabo
LABEL=${1:-t1}; shift || true
CASES=${*:-"level:0:0:0 roll_p15:15:0:0 roll_m15:-15:0:0 pitch_p15:0:15:0 pitch_m15:0:-15:0 yaw_45:0:0:45 combined:10:-10:30"}
BASE=results/tether_angles/bench/$LABEL
BODY_X=1.5; BODY_Z=1.2          # corpo a 1,5 m da estacao e 1,2 m de altura: cabo inclinado
SETTLE=${SETTLE:-15}; RECORD=${RECORD:-10}

limpa() {   # so por nome de executavel: casar padrao na linha de comando mata o proprio shell
  for p in $(ps -eo pid=,comm= | awk '$2=="gz"||$2=="ruby"{print $1}'); do kill -INT $p 2>/dev/null; done
  sleep 3
  for p in $(ps -eo pid=,comm= | awk '$2=="gz"||$2=="ruby"{print $1}'); do kill -9 $p 2>/dev/null; done
  sleep 1
}

limpa
rm -rf $BASE; mkdir -p $BASE/model
# alvo do taut relativo ao tether_exit_point (z = 0,19 m)
./tools/generate_tether_anchor_chain.py --links 5 --length 2.5 --rho 0.06 --radius 0.003 \
  --initial-axis taut --taut-target "$BODY_X 0 $(python3 -c "print($BODY_Z - 0.19)")" \
  --force-constraint --stiffness 5 --damping 0.5 --max-force 3 --no-reel-actuator \
  --drone-model attitude_body --drone-link tether_attach_link \
  --output-dir $BASE/model > $BASE/model.txt
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/build/gz_plugins
export GZ_SIM_RESOURCE_PATH=$PWD/src/pacote_do_drone/models

for spec in $CASES; do
  IFS=: read -r NAME R P Y <<< "$spec"
  OUT=$BASE/$NAME; mkdir -p $OUT
  RPY=$(python3 -c "import math; print(' '.join(f'{math.radians(float(v)):.9f}' for v in ('$R','$P','$Y')))")
  cat > $OUT/world.sdf <<EOF
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="default">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <plugin name="gz::sim::systems::Physics" filename="gz-sim-physics-system"/>
    <plugin name="gz::sim::systems::UserCommands" filename="gz-sim-user-commands-system"/>
    <plugin name="gz::sim::systems::SceneBroadcaster" filename="gz-sim-scene-broadcaster-system"/>
    <gravity>0 0 -9.8</gravity>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>50 50</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>50 50</size></plane></geometry></visual>
      </link>
    </model>
    <model name="attitude_body">
      <static>true</static>
      <pose>$BODY_X 0 $BODY_Z $RPY</pose>
      <link name="tether_attach_link">
        <visual name="corpo"><geometry><box><size>0.30 0.20 0.04</size></box></geometry></visual>
        <visual name="nariz"><pose>0.2 0 0 0 0 0</pose><geometry><box><size>0.10 0.03 0.03</size></box></geometry></visual>
      </link>
    </model>
    <include>
      <uri>file://$PWD/$BASE/model</uri>
    </include>
  </world>
</sdf>
EOF
  echo "$NAME roll=$R pitch=$P yaw=$Y" > $OUT/case.txt
  gz sim -s -r $OUT/world.sdf > $OUT/gz.log 2>&1 &
  sleep $SETTLE
  ./tools/record_tether_timeseries.py --duration $RECORD --output-dir $OUT --prefix plugin > $OUT/ts.log 2>&1 &
  R0=$!
  ./tools/record_tether_connection.py --duration $RECORD --output-dir $OUT --prefix connection --links 5 --segment-length 0.5 \
    --drone-model attitude_body --settle 0 --tangent-window 0.15 --collision-radius 0 > $OUT/pose.log 2>&1
  wait $R0
  grep -cE "Assertion|Aborted" $OUT/gz.log > $OUT/aborts.txt
  limpa
done
echo DONE > $BASE/.done
