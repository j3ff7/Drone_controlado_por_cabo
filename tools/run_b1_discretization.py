#!/usr/bin/env python3
"""Sweep de discretizacao/escalabilidade do tether (B1, B2, C1-C4).

Para cada N: regenera o modelo, sobe PX4/Gazebo headless, insere o cabo, deixa
assentar, mede o estado estatico e encerra. A forma inicial padrao e `taut` (arco de
comprimento L e corda fixa), a mesma familia geometrica para qualquer N -- sem isso o
sweep misturaria condicao inicial com erro de discretizacao. `--initial-axis x` troca o
arco por um cabo reto: e a unica classe que o gz-physics constroi acima de N ~ 34 (C2.3),
e por isso a unica via para medir discretizacoes finas.
"""
import sys

sys.path = ['/usr/lib/python3/dist-packages'] + [p for p in sys.path if 'local/lib' not in p]

import argparse  # noqa: E402
import csv  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
import os  # noqa: E402
import subprocess  # noqa: E402
import time  # noqa: E402
from pathlib import Path  # noqa: E402

import numpy as np  # noqa: E402


ROOT = Path(__file__).resolve().parents[1]
PX4 = ROOT / 'px4' / 'PX4-Autopilot'
MODELS = ROOT / 'src' / 'pacote_do_drone' / 'models'
PLUGINS = ROOT / 'build' / 'gz_plugins'
TETHER_SDF = MODELS / 'tether_anchor_chain' / 'model.sdf'


def sh(cmd, **kwargs):
    return subprocess.run(cmd, shell=isinstance(cmd, str), capture_output=True, text=True, **kwargs)


class Timeout(Exception):
    pass


def sh_timeout(cmd, seconds):
    """`gz topic -e -n 1` NAO retorna se o servidor morreu: fica esperando a mensagem.

    Sem teto, um aborto durante a construcao do modelo trava o sweep inteiro no
    primeiro `sim_time_now()` (foi o que aconteceu em C2/N=40).
    """
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=seconds)
    except subprocess.TimeoutExpired:
        raise Timeout(' '.join(cmd))


def sim_processes():
    """Processos reais do simulador, por nome do executavel.

    Nao usar `pgrep -f 'gz sim'`: o padrao casa com a propria linha de comando de
    qualquer shell que o contenha, o wait loop nunca esvazia e o PX4 da iteracao
    anterior sobrevive para colidir com a proxima.
    """
    listing = sh(['ps', '-eo', 'pid=,comm=']).stdout.splitlines()
    return [line.split()[0] for line in listing
            if line.split()[1:] and line.split()[1] in ('px4', 'gz', 'ruby')]


def kill_sim():
    for pattern in ('px4_sitl_default/bin/px4',
                    'Tools/simulation/gz/worlds/default.sdf',
                    'make px4_sitl gz_x500'):
        sh(['pkill', '-KILL', '-f', pattern])
    for pid in sim_processes():
        sh(['kill', '-KILL', pid])
    for _ in range(20):
        time.sleep(1)
        if not sim_processes():
            return True
    return False


def drone_distance(length, ratio, override):
    """Distancia estacao->UAV. Escala com L para manter a folga inicial constante."""
    return override if override is not None else ratio * length


def generate(n_links, length, rho, radius, drone_x, exit_z, attach_z, collisions,
             initial_axis='taut', joint_type='ball', universal_roll=0.0):
    # Alvo do cabo = attach do UAV, relativo ao tether_exit_point.
    target = f'{drone_x} 0 {attach_z - exit_z}'
    cmd = [str(ROOT / 'tools' / 'generate_tether_anchor_chain.py'),
           '--links', str(n_links), '--length', str(length), '--rho', str(rho),
           '--radius', str(radius), '--initial-axis', initial_axis,
           '--force-constraint', '--stiffness', '5', '--damping', '0.5', '--max-force', '3']
    if initial_axis == 'taut':
        cmd += ['--taut-target', target]
    cmd += ['--joint-type', joint_type, '--universal-roll', str(universal_roll)]
    if collisions:
        cmd.append('--link-collisions')
    result = sh(cmd, cwd=ROOT)
    if result.returncode != 0:
        raise SystemExit(f'gerador falhou para N={n_links}: {result.stderr}')
    return {line.split('=', 1)[0]: line.split('=', 1)[1]
            for line in result.stdout.splitlines() if '=' in line}


def launch(log_path, drone_x, drone_y=0.0):
    env = {**os.environ,
           'GZ_SIM_RESOURCE_PATH': f'{MODELS}:{os.environ.get("GZ_SIM_RESOURCE_PATH", "")}',
           'GZ_SIM_SYSTEM_PLUGIN_PATH': f'{PLUGINS}:{os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")}',
           'HEADLESS': '1', 'PX4_GZ_MODEL': 'x500_tether_attach',
           # Afasta o UAV da estacao para o cabo nascer esticado em vez de em laco.
           # drone_y != 0 desloca o UAV lateralmente sem mexer no alvo `taut`: e a
           # perturbacao lateral controlada (usada em C3).
           'PX4_GZ_MODEL_POSE': f'{drone_x},{drone_y},0,0,0,0'}
    handle = open(log_path, 'w')
    proc = subprocess.Popen(
        f"make px4_sitl gz_x500 2>&1 | stdbuf -o0 tr '\\r' '\\n' | stdbuf -o0 uniq",
        shell=True, cwd=PX4, env=env, stdout=handle, stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL)
    deadline = time.time() + 150
    while time.time() < deadline:
        if '/world/default/stats' in sh(['gz', 'topic', '-l']).stdout:
            return proc, handle
        time.sleep(3)
    return None, handle


def spawn_tether():
    req = (f'sdf_filename: "{TETHER_SDF}" name: "tether_anchor_chain" '
           'allow_renaming: false pose: {position: {z: 0}}')
    return 'data: true' in sh(
        ['gz', 'service', '-s', '/world/default/create',
         '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '5000', '--req', req]).stdout


def sim_time_now(timeout=10.0):
    try:
        result = sh_timeout(['gz', 'topic', '-e', '-t', '/stats', '-n', '1'], timeout)
    except Timeout:
        return None
    sec = nsec = None
    for line in result.stdout.splitlines():
        stripped = line.strip()
        if sec is None and stripped.startswith('sec:'):
            sec = int(stripped.split(':')[1])
        elif sec is not None and nsec is None and stripped.startswith('nsec:'):
            nsec = int(stripped.split(':')[1])
            break
    if sec is None:
        return None
    return sec + (nsec or 0) * 1e-9


def is_server_command_line(line):
    """A linha de comando e a do servidor gz (e nao a de uma ferramenta auxiliar)?"""
    return ' sim ' in line and ' -s ' in line and '.sdf' in line and 'ps -eo' not in line


def sim_alive():
    """O SERVIDOR gz esta de pe?

    Nao usar `comm`: o executavel do `gz` e um wrapper ruby, entao o nome do processo
    do servidor e `ruby` nesta instalacao — e `ruby`/`gz-transport-to` tambem aparecem
    para cada `gz topic` auxiliar, o que daria falso positivo. A linha de comando do
    servidor e inconfundivel (`gz sim ... -s ... .sdf`) e nenhuma ferramenta deste repo
    a contem, entao nao ha o risco de casar com a propria linha de comando que derrubou
    o `pgrep -f` em B1.
    """
    return any(is_server_command_line(line)
               for line in sh(['ps', '-eo', 'args=']).stdout.splitlines())


def settle_sim_seconds(target_sim, wall_cap):
    """Espera `target_sim` segundos de tempo SIMULADO, com teto de wall-clock.

    Dormir em wall-clock deixaria as configuracoes lentas assentarem menos que as
    rapidas, e a comparacao entre N deixaria de ser justa.
    """
    start_wall = time.time()
    if not sim_alive():
        # Aborto na construcao do modelo: nao ha relogio para esperar.
        return {'settled_sim_s': None, 'settle_wall_s': 0.0, 'settle_capped': False,
                'sim_died_during_settle': True}
    start_sim = sim_time_now()
    if start_sim is None:
        time.sleep(min(target_sim, wall_cap))
        return {'settled_sim_s': None, 'settle_wall_s': time.time() - start_wall,
                'settle_capped': False}
    while time.time() - start_wall < wall_cap:
        if not sim_alive():
            # O gz sim morreu (aborto do DART): nao adianta esperar o teto inteiro.
            return {'settled_sim_s': None, 'settle_wall_s': time.time() - start_wall,
                    'settle_capped': False, 'sim_died_during_settle': True}
        now = sim_time_now()
        if now is None:
            break
        if now - start_sim >= target_sim:
            return {'settled_sim_s': now - start_sim,
                    'settle_wall_s': time.time() - start_wall, 'settle_capped': False}
        time.sleep(1.0)
    final = sim_time_now()
    return {'settled_sim_s': (final - start_sim) if final is not None else None,
            'settle_wall_s': time.time() - start_wall, 'settle_capped': True}


def record(out_dir, prefix, duration):
    sh([str(ROOT / 'tools' / 'record_tether_timeseries.py'),
        '--duration', str(duration), '--output-dir', str(out_dir), '--prefix', prefix],
       cwd=ROOT)


def column(path, key):
    if not Path(path).exists():
        return np.array([])
    with Path(path).open() as handle:
        rows = list(csv.DictReader(handle))
    values = []
    for row in rows:
        try:
            values.append(float(row[key]))
        except (TypeError, ValueError):
            values.append(math.nan)
    return np.array(values)


def summarize(out_dir, prefix, log_path):
    stats_csv = out_dir / f'{prefix}_stats.csv'
    tension_csv = out_dir / f'{prefix}_tensao.csv'
    world_csv = out_dir / f'{prefix}_world_stats.csv'

    error = column(stats_csv, 'x')
    force = column(stats_csv, 'y')
    sat = column(stats_csv, 'z')
    t_full = column(tension_csv, 'x')
    t_qs = column(tension_csv, 'y')
    t_flag = column(tension_csv, 'z')
    rtf = column(world_csv, 'rtf')

    def stat(values):
        finite = values[np.isfinite(values)] if values.size else values
        if not finite.size:
            return None
        return {'mean': float(np.mean(finite)), 'rms': float(np.sqrt(np.mean(finite ** 2))),
                'max': float(np.max(finite)), 'min': float(np.min(finite)),
                'std': float(np.std(finite)), 'samples': int(finite.size),
                'nan': int(values.size - finite.size)}

    log_text = Path(log_path).read_text(errors='replace') if Path(log_path).exists() else ''
    aborted = any(token in log_text for token in ('verifyTransform', 'Aborted', 'Assertion'))

    finite_error = error[np.isfinite(error)] if error.size else error
    diverged = bool(finite_error.size and np.max(np.abs(finite_error)) > 5.0)

    return {
        'constraint_error_m': stat(error),
        'force_uav_n': stat(force),
        'saturation_fraction': (float(np.mean(sat[np.isfinite(sat)] >= 0.5))
                                if sat.size and np.isfinite(sat).any() else None),
        'T_est_full_n': stat(t_full),
        'T_est_quasi_static_n': stat(t_qs),
        'T_est_availability': (float(np.mean(t_flag[np.isfinite(t_flag)] >= 0.5))
                               if t_flag.size and np.isfinite(t_flag).any() else None),
        'rtf': stat(rtf),
        'wall_per_sim_s': (1.0 / float(np.mean(rtf[np.isfinite(rtf)]))
                           if rtf.size and np.isfinite(rtf).any()
                           and np.mean(rtf[np.isfinite(rtf)]) > 0 else None),
        'dart_abort': aborted,
        'diverged': diverged,
        'stable': bool(not aborted and not diverged and error.size > 0),
    }


def write_report(out_root, report):
    payload = json.dumps(report, indent=2)
    (out_root / 'sweep.json').write_text(payload)
    (out_root / 'b1_discretization.json').write_text(payload)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', default='results/b1')
    parser.add_argument('--links', type=int, nargs='+', default=[5, 10, 20, 25])
    parser.add_argument('--length', type=float, default=2.5)
    parser.add_argument('--rho', type=float, default=0.06)
    parser.add_argument('--radius', type=float, default=0.003)
    parser.add_argument('--settle', type=float, default=25.0)
    parser.add_argument('--measure', type=float, default=20.0)
    parser.add_argument('--drone-x', type=float, default=None,
                        help='distancia estacao->UAV [m]; escolhida para o cabo nascer '
                             'esticado (4,7%% de folga) com tensao bem abaixo de Fmax')
    parser.add_argument('--exit-z', type=float, default=0.19)
    parser.add_argument('--attach-z', type=float, default=0.107)
    parser.add_argument('--collisions', action='store_true',
                        help='B1.2: habilita colisoes dos segmentos do cabo')
    parser.add_argument('--taut-ratio', type=float, default=0.953,
                        help='D/L: mantem a folga inicial constante ao variar L')
    parser.add_argument('--settle-wall-cap', type=float, default=180.0,
                        help='teto de wall-clock para o assentamento [s]')
    parser.add_argument('--drone-y', type=float, default=0.0,
                        help='deslocamento lateral do UAV [m]; o alvo taut continua '
                             'em y=0, entao o cabo nasce puxado de lado')
    parser.add_argument('--initial-axis', choices=('taut', 'x'), default='taut',
                        help='taut: arco N-independente de B1/B2. x: cabo reto em +x, '
                             'unica classe que constroi acima de N~34 (ver C2.3)')
    parser.add_argument('--joint-type', choices=('ball', 'universal'), default='ball')
    parser.add_argument('--universal-roll', type=float, default=0.0)
    parser.add_argument('--transient', type=float, default=0.0,
                        help='grava `transitorio` LOGO APOS o spawn, antes do '
                             'assentamento [s]; 0 desliga. A perturbacao lateral e uma '
                             'condicao inicial, entao o transitorio so existe aqui')
    parser.add_argument('--shape', action='store_true',
                        help='captura a forma do cabo (poses dos elos) ao fim do assentamento')
    args = parser.parse_args()

    out_root = Path(args.output_dir)
    out_root.mkdir(parents=True, exist_ok=True)
    report = {'length_m': args.length, 'rho_linear_kg_m': args.rho,
              'initial_shape': args.initial_axis, 'drone_x_m': args.drone_x,
              'segment_collisions': bool(args.collisions),
              'drone_y_m': args.drone_y, 'configs': []}

    for n_links in args.links:
        print(f'--- N={n_links} ---', flush=True)
        if not kill_sim():
            raise SystemExit('nao foi possivel encerrar o simulador anterior; '
                             'seguir contaminaria a medicao')
        distance = drone_distance(args.length, args.taut_ratio, args.drone_x)
        params = generate(n_links, args.length, args.rho, args.radius,
                          distance, args.exit_z, args.attach_z, args.collisions,
                          args.initial_axis, args.joint_type, args.universal_roll)
        out_dir = out_root / f'n{n_links:02d}'
        out_dir.mkdir(parents=True, exist_ok=True)
        log_path = out_dir / 'px4.log'

        entry = {'n_links': n_links,
                 'length_m': args.length,
                 'initial_shape': args.initial_axis,
                 'joint_type': args.joint_type,
                 'universal_roll_deg': args.universal_roll,
                 'drone_x_m': distance,
                 'drone_y_m': args.drone_y,
                 'segment_length_m': args.length / n_links,
                 'link_mass_kg': args.rho * args.length / n_links,
                 'generator': {k: params.get(k) for k in
                               ('massa_total', 'massa_por_link', 'comprimento_por_link')}}

        proc, handle = launch(log_path, distance, args.drone_y)
        if proc is None:
            entry.update({'launched': False, 'stable': False})
            report['configs'].append(entry)
            write_report(out_root, report)
            handle.close()
            kill_sim()
            continue

        entry['launched'] = True
        if not spawn_tether():
            entry.update({'spawned': False, 'stable': False})
            report['configs'].append(entry)
            write_report(out_root, report)
            handle.close()
            kill_sim()
            continue
        entry['spawned'] = True

        if args.transient > 0.0 and sim_alive():
            record(out_dir, 'transitorio', args.transient)
            entry['transient'] = summarize(out_dir, 'transitorio', log_path)
        entry.update(settle_sim_seconds(args.settle, args.settle_wall_cap))
        if args.shape and sim_alive():
            shape = sh([str(ROOT / 'tools' / 'capture_tether_shape.py'),
                        '--links', str(n_links), '--exit-z', str(args.exit_z),
                        '--output', str(out_dir / 'forma.json')], cwd=ROOT)
            entry['shape_captured'] = shape.returncode == 0
        if sim_alive():
            record(out_dir, 'estatico', args.measure)
        handle.close()
        entry.update(summarize(out_dir, 'estatico', log_path))
        report['configs'].append(entry)
        write_report(out_root, report)

        e = entry.get('constraint_error_m') or {}
        f = entry.get('force_uav_n') or {}
        t = entry.get('T_est_quasi_static_n') or {}
        r = entry.get('rtf') or {}
        print(f"  |e| RMS={e.get('rms')} |F| RMS={f.get('rms')} T_est={t.get('mean')} "
              f"RTF={r.get('mean')} estavel={entry.get('stable')}", flush=True)
        kill_sim()

    write_report(out_root, report)

    header = ('| N | l [m] | m_link [kg] | RTF medio | RTF p05 | |e| RMS [m] | |F| RMS [N] '
              '| T_est [N] | saturacao | estavel |')
    lines = [header, '| ' + ' | '.join(['---'] * 10) + ' |']
    for entry in report['configs']:
        e = entry.get('constraint_error_m') or {}
        f = entry.get('force_uav_n') or {}
        t = entry.get('T_est_quasi_static_n') or {}
        r = entry.get('rtf') or {}
        rtfs = column(out_root / f"n{entry['n_links']:02d}" / 'estatico_world_stats.csv', 'rtf')
        rtfs = rtfs[np.isfinite(rtfs)]
        p05 = f'{np.percentile(rtfs, 5):.4f}' if rtfs.size else 'N/D'
        fmt = lambda v, n=4: ('N/D' if v is None else f'{v:.{n}f}')  # noqa: E731
        lines.append(
            f"| {entry['n_links']} | {entry['segment_length_m']:.4f} | {entry['link_mass_kg']:.4f} "
            f"| {fmt(r.get('mean'))} | {p05} | {fmt(e.get('rms'))} | {fmt(f.get('rms'))} "
            f"| {fmt(t.get('mean'))} | {fmt(entry.get('saturation_fraction'))} "
            f"| {'sim' if entry.get('stable') else 'NAO'} |")
    table = '\n'.join(lines)
    (out_root / 'sweep.md').write_text(table + '\n')
    (out_root / 'b1_discretization.md').write_text(table + '\n')
    print()
    print(table)


if __name__ == '__main__':
    main()
