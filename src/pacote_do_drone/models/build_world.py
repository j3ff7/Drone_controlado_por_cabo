import json
import math
from pathlib import Path

# ============================================================
# CAMINHOS E PASTAS
# ============================================================

raiz_pacote = Path(__file__).resolve().parent.parent
caminho_json = raiz_pacote /'tether_package' / 'parameters' / 'tether_parameters.json'
pasta_models = raiz_pacote / 'models'
pasta_worlds = raiz_pacote / 'worlds'
caminho_sdf = pasta_models / 'cabo.sdf'
caminho_world = pasta_worlds / 'my_world.sdf'

pasta_models.mkdir(exist_ok=True)
pasta_worlds.mkdir(exist_ok=True)

# ============================================================
# FUNÇÕES AUXILIARES
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


def lerp(p, q, a):
    return (
        p[0] + (q[0] - p[0]) * a,
        p[1] + (q[1] - p[1]) * a,
        p[2] + (q[2] - p[2]) * a
    )


def calcular_bezier(p0, p1, p2, p3, t):
    u = 1.0 - t
    tt = t * t
    uu = u * u
    uuu = uu * u
    ttt = tt * t

    px = uuu * p0[0] + 3 * uu * t * p1[0] + 3 * u * tt * p2[0] + ttt * p3[0]
    py = uuu * p0[1] + 3 * uu * t * p1[1] + 3 * u * tt * p2[1] + ttt * p3[1]
    pz = uuu * p0[2] + 3 * uu * t * p1[2] + 3 * u * tt * p2[2] + ttt * p3[2]

    return (px, py, pz)


def yaw_pitch_do_segmento(p0, p1):
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]

    yaw = math.atan2(dy, dx)
    pitch = -math.atan2(dz, math.hypot(dx, dy))

    return yaw, pitch


def rotacionar_offset_yaw(x, y, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)

    return (
        c * x - s * y,
        s * x + c * y
    )


def gerar_pontos_bezier(P0, P1, P2, P3, num_amostras):
    curva = []
    for k in range(num_amostras + 1):
        t = k / num_amostras
        curva.append(calcular_bezier(P0, P1, P2, P3, t))
    cum = [0.0]
    for k in range(1, len(curva)):
        cum.append(cum[-1] + dist3(curva[k - 1], curva[k]))
    return curva, cum


def comprimento_bezier(P0, P1, P2, P3, num_amostras):
    _, cum = gerar_pontos_bezier(P0, P1, P2, P3, num_amostras)
    return cum[-1]


def montar_bezier_com_sag(P0, P3, sag):
    dx = P3[0] - P0[0]
    dy = P3[1] - P0[1]
    dz = P3[2] - P0[2]

    P1 = (
        P0[0] + 0.333 * dx,
        P0[1] + 0.333 * dy,
        P0[2] + 0.333 * dz - sag
    )
    P2 = (
        P0[0] + 0.666 * dx,
        P0[1] + 0.666 * dy,
        P0[2] + 0.666 * dz - sag
    )
    return P1, P2


def reamostrar_curva_por_comprimento(curva, cum, num_links, length, P3):
    pontos = []
    j = 1
    for i in range(num_links + 1):
        if i == num_links:
            pontos.append(P3)   # força o último ponto exato
            continue

        alvo_s = i * length
        while j < len(cum) - 1 and cum[j] < alvo_s:
            j += 1
        s0 = cum[j - 1]
        s1 = cum[j]
        if abs(s1 - s0) < 1e-12:
            a = 0.0
        else:
            a = (alvo_s - s0) / (s1 - s0)
        pontos.append(lerp(curva[j - 1], curva[j], a))
    return pontos


# ============================================================
# LER PARÂMETROS
# ============================================================

with open(caminho_json, 'r') as f:
    params = json.load(f)

num_links = max(3, int(params.get("num_links", 50)))

length = clamp_min(params.get("length", 0.05), 0.01)
radius = clamp_min(params.get("radius", 0.003), 0.001)

densidade_linear = float(params.get("densidade_linear", 0.04))

# Ponto alvo da ponta do cabo (substitui os antigos z_alvo_cabo, comprimento_curva_cabo, etc.)
cabo_fim_x = float(params.get("cabo_fim_x", 1.0))
cabo_fim_y = float(params.get("cabo_fim_y", 0.18))   # pode manter o mesmo y da âncora se quiseres
cabo_fim_z = float(params.get("cabo_fim_z", 0.08))   # altura desejada da ponta

# ============================================================
# CONFIGURAÇÕES FÍSICAS DO CABO
# ============================================================

comprimento_total = num_links * length

ancora_x = 0.0
ancora_y = 0.18
ancora_z = 0.335

offset_conexao_drone_x = float(params.get("offset_conexao_drone_x", 0.0))
offset_conexao_drone_y = float(params.get("offset_conexao_drone_y", 0.0))
offset_conexao_drone_z = float(params.get("offset_conexao_drone_z", -0.01))

# Amortecimento e atrito elevados (como no segundo código que funcionou)
damping_junta = float(params.get("damping_junta", 0.5))
friction_junta = float(params.get("friction_junta", 0.1))
limite_junta = float(params.get("limite_junta", 2.0))   # mantido mas não utilizado diretamente agora
limite_junta_deg = float(params.get("limite_junta_deg", 30.0))  # limite em graus para juntas universal

z_minimo_chao = max(radius + 0.02, 0.03)

# ============================================================
# GERAÇÃO DA BÉZIER ÚNICA ATÉ O PONTO ALVO
# ============================================================

P0 = (ancora_x, ancora_y, ancora_z)
P3_original = (cabo_fim_x, cabo_fim_y, cabo_fim_z)

num_amostras = max(4000, num_links * 250)

dist_reta_original = dist3(P0, P3_original)

# Direcção horizontal (para eventual alongamento)
dx_xy = cabo_fim_x - ancora_x
dy_xy = cabo_fim_y - ancora_y
dist_xy_original = math.hypot(dx_xy, dy_xy)
if dist_xy_original < 1e-9:
    dir_x, dir_y = 1.0, 0.0
else:
    dir_x, dir_y = dx_xy / dist_xy_original, dy_xy / dist_xy_original

# Altura base para cálculo do sag máximo (não pode descer abaixo do chão)
z_base = min(ancora_z, cabo_fim_z)
sag_maximo = max(0.0, z_base - z_minimo_chao)

# Caso 1: alvo está mais longe que o cabo → esticamos na direcção do alvo
if dist_reta_original > comprimento_total:
    direcao = norm3((P3_original[0]-P0[0], P3_original[1]-P0[1], P3_original[2]-P0[2]))
    P3 = (P0[0] + direcao[0]*comprimento_total,
          P0[1] + direcao[1]*comprimento_total,
          P0[2] + direcao[2]*comprimento_total)
    sag_final = 0.0
    print("⚠️ Alvo fora do alcance. Cabo esticado na direção do alvo.")
else:
    # Tentamos manter o alvo e ajustar a barriga (sag) para consumir o comprimento
    P1_max, P2_max = montar_bezier_com_sag(P0, P3_original, sag_maximo)
    comprimento_com_sag_maximo = comprimento_bezier(P0, P1_max, P2_max, P3_original, num_amostras)

    if comprimento_com_sag_maximo >= comprimento_total:
        # A curva com sag máximo já é maior que o cabo; podemos manter o alvo
        P3 = P3_original
        baixo, alto = 0.0, sag_maximo
        for _ in range(60):
            meio = 0.5*(baixo+alto)
            P1_t, P2_t = montar_bezier_com_sag(P0, P3, meio)
            comp_t = comprimento_bezier(P0, P1_t, P2_t, P3, num_amostras)
            if comp_t < comprimento_total:
                baixo = meio
            else:
                alto = meio
        sag_final = 0.5*(baixo+alto)
        print("✓ Alvo mantido. Sag ajustado.")
    else:
        # Mesmo com sag máximo a curva não atinge o comprimento do cabo → alongar horizontalmente
        sag_final = sag_maximo
        baixo = dist_xy_original
        alto = max(dist_xy_original + 0.10, comprimento_total)
        # Encontrar um limite superior que ultrapasse o comprimento
        while True:
            P3_test = (ancora_x + dir_x*alto, ancora_y + dir_y*alto, cabo_fim_z)
            P1_t, P2_t = montar_bezier_com_sag(P0, P3_test, sag_final)
            comp_t = comprimento_bezier(P0, P1_t, P2_t, P3_test, num_amostras)
            if comp_t >= comprimento_total:
                break
            alto *= 1.5
        # Refinamento binário
        for _ in range(70):
            meio = 0.5*(baixo+alto)
            P3_test = (ancora_x + dir_x*meio, ancora_y + dir_y*meio, cabo_fim_z)
            P1_t, P2_t = montar_bezier_com_sag(P0, P3_test, sag_final)
            comp_t = comprimento_bezier(P0, P1_t, P2_t, P3_test, num_amostras)
            if comp_t < comprimento_total:
                baixo = meio
            else:
                alto = meio
        dist_xy_final = 0.5*(baixo+alto)
        P3 = (ancora_x + dir_x*dist_xy_final, ancora_y + dir_y*dist_xy_final, cabo_fim_z)
        print("⚠️ Cabo longo demais para o alvo sem bater no chão. Ponto final alongado horizontalmente.")

P1, P2 = montar_bezier_com_sag(P0, P3, sag_final)
curva, cum = gerar_pontos_bezier(P0, P1, P2, P3, num_amostras)
comprimento_curva = cum[-1]

pontos_cabo = reamostrar_curva_por_comprimento(curva, cum, num_links, length, P3)

p_final = pontos_cabo[-1]

# ============================================================
# DIAGNÓSTICO DOS ELOS
# ============================================================

comprimentos_reais = [dist3(pontos_cabo[i-1], pontos_cabo[i]) for i in range(1, len(pontos_cabo))]

print("============================================================")
print("GERAÇÃO DO CABO (Bézier única)")
print("============================================================")
print(f"Número de elos: {num_links}")
print(f"Comprimento nominal de cada elo: {length:.6f} m")
print(f"Comprimento total nominal: {comprimento_total:.6f} m")
print(f"Comprimento da Bézier: {comprimento_curva:.6f} m")
print(f"Sag utilizado: {sag_final:.4f} m")
print(f"Menor elo real: {min(comprimentos_reais):.6f} m")
print(f"Maior elo real: {max(comprimentos_reais):.6f} m")
print(f"Ponto final original: ({cabo_fim_x:.4f}, {cabo_fim_y:.4f}, {cabo_fim_z:.4f})")
print(f"Ponto final usado:     ({P3[0]:.4f}, {P3[1]:.4f}, {P3[2]:.4f})")
print("============================================================")

# ============================================================
# SPAWN DO DRONE
# ============================================================

yaw_base = math.atan2(P3[1] - ancora_y, P3[0] - ancora_x)
offset_world_x, offset_world_y = rotacionar_offset_yaw(
    offset_conexao_drone_x, offset_conexao_drone_y, yaw_base
)

drone_spawn_x = p_final[0] - offset_world_x
drone_spawn_y = p_final[1] - offset_world_y
drone_spawn_z = p_final[2] - offset_conexao_drone_z

print(f"Ponta real do cabo: ({p_final[0]:.4f}, {p_final[1]:.4f}, {p_final[2]:.4f})")
print(f"Spawn drone:        ({drone_spawn_x:.4f}, {drone_spawn_y:.4f}, {drone_spawn_z:.4f})")
print(f"Yaw sugerido:       {yaw_base:.6f} rad")

# ============================================================
# INÉRCIAS (iguais ao código anterior)
# ============================================================

limite_inercia_minima = 1e-5
massa_raiz = 0.02
raio_raiz_visual = 0.005
ixx_raiz = max((2.0/5.0)*massa_raiz*raio_raiz_visual**2, limite_inercia_minima)

massa_ponta = 0.005   # massa maior para estabilidade
raio_ponta = max(2.5*radius, 0.006)
ixx_ponta = max((2.0/5.0)*massa_ponta*raio_ponta**2, limite_inercia_minima)

# ============================================================
# GERAÇÃO DO cabo.sdf  (agora com juntas "universal" e limites)
# ============================================================

# Converte o limite de graus para radianos
limite_rad = math.radians(limite_junta_deg)
limite_xml = f"""
        <limit>
          <lower>{-limite_rad:.6f}</lower>
          <upper>{limite_rad:.6f}</upper>
        </limit>"""

sdf = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="cabo_flexivel">
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
    ixx_seg = max(0.5 * mass_seg * radius**2, limite_inercia_minima)
    iyy_zz_seg = max((1.0/12.0) * mass_seg * (3*radius**2 + seg_len**2), limite_inercia_minima)

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
      <sensor name="sensor_tensao_carretel" type="force_torque">
        <always_on>true</always_on>
        <update_rate>50</update_rate>
        <topic>/cabo/tensao_carretel</topic>
      </sensor>"""

    sdf += f"""
      <link name="{nome_elo}">
        <pose>{pose_x:.6f} {pose_y:.6f} {pose_z:.6f} 0 {pitch:.6f} {yaw:.6f}</pose>
        <enable_wind>false</enable_wind>
        <self_collide>false</self_collide>

        <visual name="visual">
          <pose>{seg_len/2:.6f} 0 0 0 1.5708 0</pose>
          <geometry><cylinder><radius>{radius}</radius><length>{seg_len:.6f}</length></cylinder></geometry>
          <material>
            <ambient>0 0 0 1</ambient>
            <diffuse>0 0 0 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <pose>{seg_len/2:.6f} 0 0 0 1.5708 0</pose>
          <geometry><cylinder><radius>{collision_radius_seg}</radius><length>{collision_length_seg:.6f}</length></cylinder></geometry>
        </collision>

        <inertial>
          <pose>{seg_len/2:.6f} 0 0 0 0 0</pose>
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

# ---- elo extra: só a esfera, encaixada exatamente no fim do último cilindro ----
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
        <sensor name="sensor_tensao_drone" type="force_torque">
          <always_on>true</always_on>
          <update_rate>50</update_rate>
          <topic>/cabo/tensao_drone</topic>
        </sensor>
      </joint>
  """

sdf += """
  </model>
</sdf>
"""




with open(caminho_sdf, "w") as f:
    f.write(sdf)
print("✓ cabo.sdf gerado com sucesso!")

# ============================================================
# GERAR my_world.sdf (inalterado, exceto a junta da âncora permanece "ball")
# ============================================================

world = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="mundo_ic">
    <gravity>0 0 -9.81</gravity>

    <physics name="physics" type="ode">
      <max_step_size>0.0005</max_step_size>
      <real_time_update_rate>2000</real_time_update_rate>
      <real_time_factor>1.0</real_time_factor>
      <ode>
        <solver>
          <type>quick</type>
          <iters>800</iters>
          <sor>1.0</sor>
        </solver>
        <constraints>
          <cfm>1e-6</cfm>
          <erp>0.9</erp>
          <contact_max_correcting_vel>0.5</contact_max_correcting_vel>
          <contact_surface_layer>0.01</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <plugin filename="gz-sim-physics-system"           name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"     name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-forcetorque-system"       name="gz::sim::systems::ForceTorque"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <include>
      <uri>model://carretel</uri>
      <name>meu_carretel</name>
      <pose>0 0 0 0 0 0</pose>
    </include>

    <include>
      <uri>model://cabo.sdf</uri>
      <name>cabo_dinamico</name>
      <pose>{ancora_x} {ancora_y} {ancora_z} 0 0 0</pose>
      <static>false</static>
    </include>

    <joint name="ancora_carretel_cabo" type="ball">
      <parent>meu_carretel::cilindro_carretel</parent>
      <child>cabo_dinamico::raiz_cabo</child>
      <pose>0 0 0 0 0 0</pose>
    </joint>
    
    <include>
      <uri>model://meu_drone</uri>
      <name>meu_drone</name>
      <pose>{drone_spawn_x:.6f} {drone_spawn_y:.6f} {drone_spawn_z:.6f} 0 0 {yaw_base:.6f}</pose>
    </include>

    <joint name="cabo_drone_joint" type="ball">
      <parent>cabo_dinamico::ponta_cabo</parent>
      <child>meu_drone::base_link</child>
    </joint>
  </world>
</sdf>
"""

with open(caminho_world, "w") as f:
    f.write(world)
print("✓ my_world.sdf gerado com sucesso!")