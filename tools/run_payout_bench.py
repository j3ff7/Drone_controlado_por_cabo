#!/usr/bin/env python3
"""Bancada de payout/retraction (A7): casos P0 a P4, sem PX4 e sem UAV.

Comanda torque no reel e mede o comprimento efetivo do tether contra a referencia
`L_esperado = L_nominal + R_eff * theta`, publicada pelo mesmo plugin no mesmo instante.
"""
import sys

sys.path = ['/usr/lib/python3/dist-packages'] + [p for p in sys.path if 'local/lib' not in p]

import argparse  # noqa: E402
import csv  # noqa: E402
import importlib.util  # noqa: E402
import json  # noqa: E402
import os  # noqa: E402
import signal  # noqa: E402
import subprocess  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402
from pathlib import Path  # noqa: E402

import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT / 'src' / 'pacote_do_drone' / 'worlds' / 'payout_bench.sdf'
MODELS = ROOT / 'src' / 'pacote_do_drone' / 'models'
PLUGINS = ROOT / 'build' / 'gz_plugins'
PAYOUT_TOPIC = '/cabo/tms/payout'
REF_TOPIC = '/cabo/tms/payout_ref'
TENSION_TOPIC = '/cabo/estacao/tensao'
COMMAND_TOPIC = '/cabo/tms/reel_cmd'


def _load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'tools' / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


BENCH_IO = _load('reel_bench_io')
REC = _load('record_tether_timeseries')


class Capture(threading.Thread):
    def __init__(self, topic, parser_factory, stop_event):
        super().__init__(daemon=True)
        self.topic, self.stop_event = topic, stop_event
        self.parser = parser_factory()
        self.samples, self.process = [], None

    def run(self):
        self.process = subprocess.Popen(
            ['gz', 'topic', '-e', '-t', self.topic],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
        for line in self.process.stdout:
            sample = self.parser.feed_line(line)
            if sample is not None:
                self.samples.append((time.time(), sample))
            if self.stop_event.is_set():
                break

    def terminate(self):
        if self.process and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()


def publish_command(value):
    subprocess.run(['gz', 'topic', '-t', COMMAND_TOPIC, '-m', 'gz.msgs.Double',
                    '-p', f'data: {value}'],
                   check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def run_case(case, out_dir):
    env = {**os.environ,
           'GZ_SIM_RESOURCE_PATH': f'{MODELS}:{os.environ.get("GZ_SIM_RESOURCE_PATH", "")}',
           'GZ_SIM_SYSTEM_PLUGIN_PATH': f'{PLUGINS}:{os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")}'}
    sim = subprocess.Popen(['gz', 'sim', '-s', '-r', str(WORLD)],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)

    deadline, ready = time.time() + 30.0, False
    while time.time() < deadline and not ready:
        listing = subprocess.run(['gz', 'topic', '-l'], capture_output=True, text=True)
        ready = PAYOUT_TOPIC in listing.stdout and REF_TOPIC in listing.stdout
        if sim.poll() is not None:
            break
        if not ready:
            time.sleep(0.5)

    crashed = sim.poll() is not None
    stop = threading.Event()
    caps = {'payout': Capture(PAYOUT_TOPIC, BENCH_IO.StampedVector3dParser, stop),
            'ref': Capture(REF_TOPIC, BENCH_IO.StampedVector3dParser, stop),
            'tension': Capture(TENSION_TOPIC, REC.Vector3dStreamParser, stop)}
    marks = []
    if not crashed:
        for cap in caps.values():
            cap.start()
        time.sleep(1.0)
        for duration, command in case['sequence']:
            publish_command(command)
            start = time.time()
            time.sleep(duration)
            marks.append({'command_nm': command, 'wall_start': start, 'wall_end': time.time()})
        publish_command(0.0)
        stop.set()
        for cap in caps.values():
            cap.terminate()
        for cap in caps.values():
            cap.join(timeout=5)

    sim.terminate()
    try:
        sim.communicate(timeout=10)
    except subprocess.TimeoutExpired:
        sim.kill()
        sim.communicate()

    result = {'case': case['name'], 'description': case['description'], 'crashed': bool(crashed)}
    if crashed or not caps['payout'].samples or not caps['ref'].samples:
        result.update({'samples': 0, 'valid': False})
        return result

    t = np.array([s[0] for _, s in caps['payout'].samples])
    wall = np.array([w for w, _ in caps['payout'].samples])
    L = np.array([s[1] for _, s in caps['payout'].samples])
    L_dot = np.array([s[2] for _, s in caps['payout'].samples])
    flag = np.array([s[3] for _, s in caps['payout'].samples])

    t_ref = np.array([s[0] for _, s in caps['ref'].samples])
    theta = np.array([s[1] for _, s in caps['ref'].samples])
    L_exp = np.array([s[2] for _, s in caps['ref'].samples])
    r_eff = float(np.median([s[3] for _, s in caps['ref'].samples]))
    # Os dois topicos saem do mesmo PreUpdate: alinhar pelo tempo simulado carimbado.
    theta_on_L = np.interp(t, t_ref, theta)
    L_exp_on_L = np.interp(t, t_ref, L_exp)
    error = L - L_exp_on_L

    tension = np.array([s[1] for _, s in caps['tension'].samples]) if caps['tension'].samples else None
    t_wall_tension = np.array([w for w, _ in caps['tension'].samples]) if caps['tension'].samples else None

    per_mark = []
    for mark in marks:
        m = (wall >= mark['wall_start']) & (wall <= mark['wall_end'])
        if np.count_nonzero(m) < 20:
            per_mark.append({**{k: mark[k] for k in ('command_nm',)}, 'valid': False})
            continue
        seg = {
            'command_nm': mark['command_nm'],
            'samples': int(np.count_nonzero(m)),
            'valid': True,
            'theta_start': float(theta_on_L[m][0]), 'theta_end': float(theta_on_L[m][-1]),
            'delta_theta': float(theta_on_L[m][-1] - theta_on_L[m][0]),
            'L_start': float(L[m][0]), 'L_end': float(L[m][-1]),
            'delta_L_measured': float(L[m][-1] - L[m][0]),
            'delta_L_expected': float(r_eff * (theta_on_L[m][-1] - theta_on_L[m][0])),
            'L_dot_mean': float(np.mean(L_dot[m])),
            'L_dot_max_abs': float(np.max(np.abs(L_dot[m]))),
            'monotonic_increasing': bool(np.all(np.diff(L[m]) >= -1e-9)),
            'monotonic_decreasing': bool(np.all(np.diff(L[m]) <= 1e-9)),
            'limit_flags': sorted({float(v) for v in flag[m]}),
        }
        seg['delta_L_error'] = seg['delta_L_measured'] - seg['delta_L_expected']
        if tension is not None and t_wall_tension is not None and t_wall_tension.size:
            mt = (t_wall_tension >= mark['wall_start']) & (t_wall_tension <= mark['wall_end'])
            finite = tension[mt][np.isfinite(tension[mt])] if np.any(mt) else np.array([])
            seg['T_est_mean'] = float(np.mean(finite)) if finite.size else None
        per_mark.append(seg)

    finite_err = error[np.isfinite(error)]
    result.update({
        'samples': int(len(t)), 'valid': True,
        'r_eff_m': r_eff,
        'L_range_m': [float(L.min()), float(L.max())],
        'L_dot_max_abs': float(np.max(np.abs(L_dot))),
        'theta_range_rad': [float(theta_on_L.min()), float(theta_on_L.max())],
        'tracking_error_m': {
            'mean_abs': float(np.mean(np.abs(finite_err))) if finite_err.size else None,
            'max_abs': float(np.max(np.abs(finite_err))) if finite_err.size else None,
        },
        'nan_L': int(np.count_nonzero(~np.isfinite(L))),
        'limit_flags_seen': sorted({float(v) for v in flag}),
        'max_step_jump_m': float(np.max(np.abs(np.diff(L)))) if L.size > 1 else None,
        'segments': per_mark,
    })

    slug = case['name'].lower()
    with (out_dir / f'{slug}.csv').open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['t_sim', 'theta', 'L', 'L_dot', 'L_expected', 'error', 'limit_flag'])
        writer.writerows(zip(t, theta_on_L, L, L_dot, L_exp_on_L, error, flag))

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(t, L, lw=1.1, label='L medido')
    axes[0].plot(t, L_exp_on_L, lw=0.8, ls='--', label='L esperado = L0 + R_eff*theta')
    axes[0].set_ylabel('L [m]')
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.3)
    axes[1].plot(t, L_dot, lw=0.9, color='tab:orange')
    axes[1].set_ylabel('L_dot [m/s]')
    axes[1].grid(alpha=0.3)
    axes[2].plot(t, theta_on_L, lw=0.9, color='tab:purple')
    axes[2].set_ylabel('theta [rad]')
    axes[2].set_xlabel('t_sim [s]')
    axes[2].grid(alpha=0.3)
    fig.suptitle(f"A7 {case['name']} - {case['description']}")
    fig.tight_layout()
    fig.savefig(out_dir / 'plots' / f'{slug}.png', dpi=110)
    plt.close(fig)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', default='results/a7')
    parser.add_argument('--only', nargs='+', default=None,
                        help='executa apenas estes casos, ex: --only P0 P1')
    parser.add_argument('--cmd', type=float, default=0.02,
                        help='torque de comando do reel nos casos de payout [N.m]')
    args = parser.parse_args()

    out_dir = Path(args.output_dir)
    (out_dir / 'plots').mkdir(parents=True, exist_ok=True)
    c = args.cmd

    cases = [
        {'name': 'P0', 'description': 'reel parado, comprimento constante',
         'sequence': [(8.0, 0.0)]},
        {'name': 'P1', 'description': 'payout: comando positivo libera cabo',
         'sequence': [(2.0, 0.0), (8.0, c)]},
        {'name': 'P2', 'description': 'retraction: libera e depois recolhe',
         'sequence': [(6.0, c), (2.0, 0.0), (8.0, -c)]},
        {'name': 'P3', 'description': 'reversao payout -> parada -> retraction',
         'sequence': [(5.0, c), (4.0, 0.0), (5.0, -c), (3.0, 0.0)]},
        {'name': 'P4', 'description': 'limites L_max e L_min',
         'sequence': [(14.0, 4 * c), (4.0, 0.0), (16.0, -4 * c)]},
    ]

    report = {'strategy': 'junta prismatica na guia; L = L_nominal + s',
              'command_topic': COMMAND_TOPIC, 'cases': []}
    if args.only:
        cases = [c for c in cases if c['name'] in args.only]
    for case in cases:
        result = run_case(case, out_dir)
        report['cases'].append(result)
        segs = ' | '.join(
            f"cmd={s['command_nm']:+.3f} dTheta={s['delta_theta']:+.3f} "
            f"dL={s['delta_L_measured']:+.4f} esp={s['delta_L_expected']:+.4f} "
            f"err={s['delta_L_error']:+.2e}"
            for s in result.get('segments', []) if s.get('valid'))
        print(f"{result['case']:3s} crash={result['crashed']} "
              f"L={result.get('L_range_m')} flags={result.get('limit_flags_seen')} :: {segs}")

    report['summary'] = {
        'cases_run': len(report['cases']),
        'cases_valid': sum(1 for x in report['cases'] if x.get('valid')),
        'any_crash': any(x['crashed'] for x in report['cases']),
        'any_nan': any(x.get('nan_L', 0) for x in report['cases']),
        'max_tracking_error_m': max(
            (x['tracking_error_m']['max_abs'] for x in report['cases']
             if x.get('valid') and x['tracking_error_m']['max_abs'] is not None), default=None),
        'max_step_jump_m': max((x['max_step_jump_m'] for x in report['cases']
                                if x.get('max_step_jump_m') is not None), default=None),
    }
    (out_dir / 'payout_bench.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report['summary'], indent=2))


if __name__ == '__main__':
    main()
