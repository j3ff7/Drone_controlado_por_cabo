import json
import math
from pathlib import Path

# ============================================================
# CAMINHOS E PASTAS
# ============================================================

raiz_pacote = Path(__file__).resolve().parent
caminho_json = raiz_pacote / 'parameters' / 'tether_parameters.json'

pasta_models = raiz_pacote / 'models'
pasta_cabo = pasta_models / 'cabo'
pasta_cabo.mkdir(parents=True, exist_ok=True)

caminho_sdf = pasta_cabo / 'model.sdf'
caminho_config = pasta_cabo / 'model.config'

# ============================================================
# FUNÇÕES AUXILIARES (geometria)
# ============================================================

def clamp_min(valor, minimo):
    return max(float(valor), minimo)


def dist3(p, q):
    return math.sqrt(
        (p[0] - q[0]) ** 2 +
        (p[1] - q[1]) ** 2 +
        (p[2] - q[2]) ** 2
    )


def norm3(v):
    n = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    if n < 1e-12:
        return (1.0, 0.0, 0.0)
    return (v[0] / n, v[1] / n, v[2] / n)


def yaw_pitch_do_segmento(p0, p1):
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]

    yaw = math.atan2(dy, dx)
    pitch = -math.atan2(dz, math.hypot(dx, dy))

    return yaw, pitch


# ============================================================
# LER PARÂMETROS
# ============================================================

with open(caminho_json, 'r') as f:
    params = json.load(f)

num_links = max(3, int(params.get("num_links", 50)))

length = clamp_min(params.get("length", 0.05), 0.01)
radius = clamp_min(params.get("radius", 0.003), 0.001)

densidade_linear = float(params.get("densidade_linear", 0.04))

# Ponto de ancoragem (origem do cabo dentro do proprio model)
ancora_x = float(params.get("ancora_x", 0.2))
ancora_y = float(params.get("ancora_y", 0.0))
ancora_z = float(params.get("ancora_z", 0.0))

# Ponto alvo da ponta do cabo, relativo a ancora (define so a DIREÇÃO;
# quem manda ate onde a ponta chega e num_links * length)
cabo_fim_x = float(params.get("cabo_fim_x", 1.0))
cabo_fim_y = float(params.get("cabo_fim_y", 0.0))
cabo_fim_z = float(params.get("cabo_fim_z", 0.2))

damping_junta = float(params.get("damping_junta", 0.5))
friction_junta = float(params.get("friction_junta", 0.1))
limite_junta_deg = float(params.get("limite_junta_deg", 30.0))

# ============================================================
# GERAÇÃO DOS PONTOS -- CABO SEMPRE RETO/ESTICADO
# ============================================================

comprimento_total = num_links * length

P0 = (ancora_x, ancora_y, ancora_z)
P3_original = (cabo_fim_x, cabo_fim_y, cabo_fim_z)

direcao = norm3((
    P3_original[0] - P0[0],
    P3_original[1] - P0[1],
    P3_original[2] - P0[2]
))

pontos_cabo = [
    (
        P0[0] + direcao[0] * i * length,
        P0[1] + direcao[1] * i * length,
        P0[2] + direcao[2] * i * length,
    )
    for i in range(num_links + 1)
]

p_final = pontos_cabo[-1]

comprimentos_reais = [dist3(pontos_cabo[i - 1], pontos_cabo[i]) for i in range(1, len(pontos_cabo))]

print("============================================================")
print("GERAÇÃO DO CABO (reto/esticado, sem sag)")
print("============================================================")
print(f"Número de elos: {num_links}")
print(f"Comprimento de cada elo: {length:.6f} m")
print(f"Comprimento total: {comprimento_total:.6f} m")
print(f"Direção: ({direcao[0]:.4f}, {direcao[1]:.4f}, {direcao[2]:.4f})")
print(f"Menor elo real: {min(comprimentos_reais):.6f} m")
print(f"Maior elo real: {max(comprimentos_reais):.6f} m")
print(f"Ponto final: ({p_final[0]:.4f}, {p_final[1]:.4f}, {p_final[2]:.4f})")
print("============================================================")

# ============================================================
# INÉRCIAS
# ============================================================

limite_inercia_minima = 1e-5
massa_raiz = 0.02
raio_raiz_visual = 0.005
ixx_raiz = max((2.0 / 5.0) * massa_raiz * raio_raiz_visual ** 2, limite_inercia_minima)

massa_ponta = 0.005
raio_ponta = max(2.5 * radius, 0.006)
ixx_ponta = max((2.0 / 5.0) * massa_ponta * raio_ponta ** 2, limite_inercia_minima)

# ============================================================
# GERAÇÃO DO model.sdf (cadeia de links + joints universal)
# ============================================================

limite_rad = math.radians(limite_junta_deg)
limite_xml = f"""
        <limit>
          <lower>{-limite_rad:.6f}</lower>
          <upper>{limite_rad:.6f}</upper>
        </limit>"""

sdf = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="cabo">
    <self_collide>false</self_collide>

    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"/>

    <link name="raiz_cabo">
      <pose>0 0 0 0 0 0</pose>
      <inertial>
        <mass>{massa_raiz}</mass>
        <inertia>
          <ixx>{ixx_raiz}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{ixx_raiz}</iyy><iyz>0</iyz><izz>{ixx_raiz}</izz>
        </inertia>
      </inertial>
      <visual name="visual_raiz">
        <geometry><sphere><radius>{raio_raiz_visual}</radius></sphere></geometry>
        <material>
          <ambient>0.1 0.1 0.1 1</ambient>
          <diffuse>0.1 0.1 0.1 1</diffuse>
        </material>
      </visual>
    </link>
"""

parent_link = "raiz_cabo"
for i in range(1, num_links + 1):
    nome_elo = f"segment_{i}"
    p_atual = pontos_cabo[i - 1]
    p_prox = pontos_cabo[i]
    seg_len = dist3(p_atual, p_prox)

    yaw, pitch = yaw_pitch_do_segmento(p_atual, p_prox)
    pose_x = p_atual[0] - ancora_x
    pose_y = p_atual[1] - ancora_y
    pose_z = p_atual[2] - ancora_z

    mass_seg = max(densidade_linear * seg_len, 0.001)
    ixx_seg = max(0.5 * mass_seg * radius ** 2, limite_inercia_minima)
    iyy_zz_seg = max((1.0 / 12.0) * mass_seg * (3 * radius ** 2 + seg_len ** 2), limite_inercia_minima)

    collision_radius_seg = 0.50 * radius
    collision_length_seg = max(0.001, 0.60 * seg_len)

    dynamics_xml = f"""
      <dynamics>
        <damping>{damping_junta}</damping>
        <friction>{friction_junta}</friction>
      </dynamics>"""

    sensor_xml = ""
    if i == 1:
        sensor_xml = """
      <sensor name="sensor_tensao_ancora" type="force_torque">
        <always_on>true</always_on>
        <update_rate>50</update_rate>
        <topic>/cabo/tensao_ancora</topic>
      </sensor>"""

    sdf += f"""
      <link name="{nome_elo}">
        <pose>{pose_x:.6f} {pose_y:.6f} {pose_z:.6f} 0 {pitch:.6f} {yaw:.6f}</pose>
        <enable_wind>false</enable_wind>
        <self_collide>false</self_collide>

        <visual name="visual">
          <pose>{seg_len / 2:.6f} 0 0 0 1.5708 0</pose>
          <geometry><cylinder><radius>{radius}</radius><length>{seg_len:.6f}</length></cylinder></geometry>
          <material>
            <ambient>0 0 0 1</ambient>
            <diffuse>0 0 0 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <pose>{seg_len / 2:.6f} 0 0 0 1.5708 0</pose>
          <geometry><cylinder><radius>{collision_radius_seg}</radius><length>{collision_length_seg:.6f}</length></cylinder></geometry>
        </collision>

        <inertial>
          <pose>{seg_len / 2:.6f} 0 0 0 0 0</pose>
          <mass>{mass_seg}</mass>
          <inertia>
            <ixx>{ixx_seg}</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>{iyy_zz_seg}</iyy><iyz>0</iyz><izz>{iyy_zz_seg}</izz>
          </inertia>
        </inertial>
      </link>

      <joint name="joint_{i}" type="universal">
        <parent>{parent_link}</parent>
        <child>{nome_elo}</child>
        <pose>0 0 0 0 0 0</pose>
        <axis>
          <xyz>0 1 0</xyz>{limite_xml}
        </axis>
        <axis2>
          <xyz>0 0 1</xyz>{limite_xml}
        </axis2>
        {dynamics_xml}
    {sensor_xml}
      </joint>
  """
    parent_link = nome_elo

# ---- elo extra: só a esfera na ponta ----
ponta_x = p_final[0] - ancora_x
ponta_y = p_final[1] - ancora_y
ponta_z = p_final[2] - ancora_z

sdf += f"""
      <link name="ponta_cabo">
        <pose>{ponta_x:.6f} {ponta_y:.6f} {ponta_z:.6f} 0 0 0</pose>
        <enable_wind>false</enable_wind>
        <self_collide>false</self_collide>

        <visual name="visual_ponta_vermelha">
          <geometry><sphere><radius>{raio_ponta}</radius></sphere></geometry>
          <material>
            <ambient>0.8 0.1 0.1 1</ambient>
            <diffuse>0.8 0.1 0.1 1</diffuse>
          </material>
        </visual>
        <collision name="collision_ponta">
          <geometry><sphere><radius>{raio_ponta}</radius></sphere></geometry>
        </collision>

        <inertial>
          <mass>{massa_ponta}</mass>
          <inertia>
            <ixx>{ixx_ponta}</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>{ixx_ponta}</iyy><iyz>0</iyz><izz>{ixx_ponta}</izz>
          </inertia>
        </inertial>
      </link>

      <joint name="joint_ponta" type="universal">
        <parent>{parent_link}</parent>
        <child>ponta_cabo</child>
        <pose>0 0 0 0 0 0</pose>
        <axis>
          <xyz>0 1 0</xyz>{limite_xml}
        </axis>
        <axis2>
          <xyz>0 0 1</xyz>{limite_xml}
        </axis2>
        {dynamics_xml}
        <sensor name="sensor_tensao_ponta" type="force_torque">
          <always_on>true</always_on>
          <update_rate>50</update_rate>
          <topic>/cabo/tensao_ponta</topic>
        </sensor>
      </joint>
  """

sdf += """
  </model>
</sdf>
"""

with open(caminho_sdf, "w") as f:
    f.write(sdf)
print(f"✓ model.sdf gerado em: {caminho_sdf}")

# ============================================================
# GERAÇÃO DO model.config (metadado exigido pelo esquema model://)
# ============================================================

config = """<?xml version="1.0"?>
<model>
  <name>cabo</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <description>Cabo reto/esticado (cadeia de links rigidos com joints universal), gerado dinamicamente por build_tether.py</description>
</model>
"""

with open(caminho_config, "w") as f:
    f.write(config)
print(f"✓ model.config gerado em: {caminho_config}")

print()
print(f"Para incluir em qualquer world.sdf, use: <uri>model://cabo</uri>")
print(f"Garanta que '{pasta_models}' esteja no GZ_SIM_RESOURCE_PATH.")