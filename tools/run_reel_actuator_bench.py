#!/usr/bin/env python3
"""Bancada de atuacao em malha aberta do reel (A6): casos M0 a M4.

Comanda TORQUE no eixo do reel e confere o torque aplicado contra o estimador dinamico
`tau_est = I*alpha + b*omega` validado em A4. Sem UAV, sem tether, sem malha fechada.
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
WORLD = ROOT / 'src' / 'pacote_do_drone' / 'worlds' / 'reel_torque_bench.sdf'
PLUGIN_PATH = ROOT / 'build' / 'gz_plugins'
STATE_TOPIC = '/bancada/reel/state'
ACTUATOR_TOPIC = '/cabo/tms/reel_actuator'
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


def centered_derivative(t, values, half=5, min_dt=1e-4):
    out = np.full_like(values, np.nan)
    for i in range(len(values)):
        lo, hi = max(0, i - half), min(len(values) - 1, i + half)
        dt = t[hi] - t[lo]
        if dt > min_dt:
            out[i] = (values[hi] - values[lo]) / dt
    return out


def run_case(case, limits, inertia, damping, out_dir):
    """Cada caso e uma lista de (duracao_s, comando_Nm)."""
    subprocess.run([str(ROOT / 'tools' / 'generate_reel_bench.py'), '--torque', '0',
                    '--actuator', '--tau-max', str(limits['tau_max']),
                    '--omega-max', str(limits['omega_max']),
                    '--ramp-rate', str(limits['ramp_rate'])],
                   check=True, stdout=subprocess.DEVNULL)

    env = {**os.environ,
           'GZ_SIM_SYSTEM_PLUGIN_PATH': f'{PLUGIN_PATH}:{os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")}'}
    sim = subprocess.Popen(['gz', 'sim', '-s', '-r', str(WORLD)],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)

    deadline, ready = time.time() + 25.0, False
    while time.time() < deadline and not ready:
        listing = subprocess.run(['gz', 'topic', '-l'], capture_output=True, text=True)
        ready = STATE_TOPIC in listing.stdout and ACTUATOR_TOPIC in listing.stdout
        if sim.poll() is not None:
            break
        if not ready:
            time.sleep(0.5)

    crashed = sim.poll() is not None
    stop = threading.Event()
    caps = {'state': Capture(STATE_TOPIC, BENCH_IO.StampedVector3dParser, stop),
            'actuator': Capture(ACTUATOR_TOPIC, REC.Vector3dStreamParser, stop)}
    segments = []
    if not crashed:
        for cap in caps.values():
            cap.start()
        time.sleep(0.5)
        for duration, command in case['sequence']:
            publish_command(command)
            start = time.time()
            time.sleep(duration)
            segments.append({'command_nm': command, 'wall_start': start,
                             'wall_end': time.time()})
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

    result = {'case': case['name'], 'description': case['description'],
              'crashed': bool(crashed), 'limits': limits}
    if crashed or not caps['state'].samples:
        result.update({'samples': 0, 'valid': False})
        return result, None

    t = np.array([s[0] for _, s in caps['state'].samples])
    wall = np.array([w for w, _ in caps['state'].samples])
    theta = np.array([s[1] for _, s in caps['state'].samples])
    omega = np.array([s[2] for _, s in caps['state'].samples])
    alpha = centered_derivative(t, omega)
    tau_est = inertia * alpha + damping * omega

    a_wall = np.array([w for w, _ in caps['actuator'].samples])
    a_cmd = np.array([s[0] for _, s in caps['actuator'].samples])
    a_applied = np.array([s[1] for _, s in caps['actuator'].samples])
    a_limit = np.array([s[2] for _, s in caps['actuator'].samples])
    # Alinha o atuador ao relogio simulado pelo wall-clock comum das duas capturas.
    applied_on_state = np.interp(wall, a_wall, a_applied) if a_wall.size else np.full_like(t, np.nan)

    per_segment = []
    for seg in segments:
        # Descarta a rampa: so o ultimo terco de cada patamar entra na estatistica.
        lo = seg['wall_start'] + 0.65 * (seg['wall_end'] - seg['wall_start'])
        mask = (wall >= lo) & (wall <= seg['wall_end']) & np.isfinite(alpha)
        if np.count_nonzero(mask) < 20:
            per_segment.append({**seg, 'samples': int(np.count_nonzero(mask)), 'valid': False})
            continue
        applied = applied_on_state[mask]
        est = tau_est[mask]
        per_segment.append({
            'command_nm': seg['command_nm'],
            'samples': int(np.count_nonzero(mask)),
            'valid': True,
            'tau_applied_mean': float(np.mean(applied)),
            'tau_est_mean': float(np.mean(est)),
            'abs_error_mean': float(np.mean(np.abs(est - applied))),
            'abs_error_max': float(np.max(np.abs(est - applied))),
            'omega_mean': float(np.mean(omega[mask])),
            'omega_max_abs': float(np.max(np.abs(omega[mask]))),
            'alpha_max_abs': float(np.max(np.abs(alpha[mask]))),
        })

    result.update({
        'samples': int(len(t)),
        'valid': True,
        'theta_range_rad': [float(theta.min()), float(theta.max())],
        'omega_abs_max': float(np.max(np.abs(omega))),
        'tau_applied_abs_max': float(np.nanmax(np.abs(applied_on_state))),
        'command_abs_max': float(np.max(np.abs(a_cmd))) if a_cmd.size else None,
        'limit_flag_values': sorted({float(v) for v in a_limit}) if a_limit.size else [],
        'nan_theta': int(np.count_nonzero(~np.isfinite(theta))),
        'nan_omega': int(np.count_nonzero(~np.isfinite(omega))),
        'segments': per_segment,
    })

    slug = case['name'].lower()
    with (out_dir / f'{slug}.csv').open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['t_sim', 'theta', 'omega', 'alpha', 'tau_applied', 'tau_est'])
        writer.writerows(zip(t, theta, omega, alpha, applied_on_state, tau_est))

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(t, applied_on_state, lw=1.0, label='tau aplicado')
    axes[0].plot(t, tau_est, lw=0.7, alpha=0.8, label='tau_est = I*alpha + b*omega')
    axes[0].axhline(limits['tau_max'], ls='--', color='k', lw=0.8, label='+-tau_max')
    axes[0].axhline(-limits['tau_max'], ls='--', color='k', lw=0.8)
    axes[0].set_ylabel('torque [N.m]')
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.3)
    axes[1].plot(t, omega, lw=0.9, color='tab:green')
    axes[1].set_ylabel('omega [rad/s]')
    axes[1].grid(alpha=0.3)
    axes[2].plot(t, theta, lw=0.9, color='tab:purple')
    axes[2].set_ylabel('theta [rad]')
    axes[2].set_xlabel('t_sim [s]')
    axes[2].grid(alpha=0.3)
    fig.suptitle(f"A6 {case['name']} - {case['description']}")
    fig.tight_layout()
    fig.savefig(out_dir / 'plots' / f'{slug}.png', dpi=110)
    plt.close(fig)
    return result, None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', default='results/a6')
    parser.add_argument('--tau-max', type=float, default=0.05)
    parser.add_argument('--omega-max', type=float, default=12.0)
    parser.add_argument('--ramp-rate', type=float, default=0.05)
    args = parser.parse_args()

    inertia, damping = BENCH_IO.reel_constants()
    limits = {'tau_max': args.tau_max, 'omega_max': args.omega_max,
              'ramp_rate': args.ramp_rate}
    out_dir = Path(args.output_dir)
    (out_dir / 'plots').mkdir(parents=True, exist_ok=True)

    tau = args.tau_max
    cases = [
        {'name': 'M0', 'description': 'comando zero',
         'sequence': [(5.0, 0.0)]},
        {'name': 'M1', 'description': 'comando positivo dentro do limite',
         'sequence': [(2.0, 0.0), (6.0, 0.6 * tau)]},
        {'name': 'M2', 'description': 'comando negativo, simetria de sinal',
         'sequence': [(2.0, 0.0), (6.0, -0.6 * tau)]},
        {'name': 'M3', 'description': 'rampa ate o limite e retorno a zero',
         'sequence': [(1.0, 0.0), (6.0, tau), (6.0, 0.0)]},
        {'name': 'M4', 'description': 'comando acima do limite, clamp',
         'sequence': [(2.0, 0.0), (7.0, 3.0 * tau)]},
    ]

    report = {'inertia_axial_kg_m2': inertia, 'damping_nms_rad': damping,
              'actuation': 'torque no eixo do reel (pos-reducao)',
              'command_topic': COMMAND_TOPIC, 'limits': limits, 'cases': []}

    for case in cases:
        result, _ = run_case(case, limits, inertia, damping, out_dir)
        report['cases'].append(result)
        segs = ' | '.join(
            f"cmd={s['command_nm']:+.4f} tau={s.get('tau_applied_mean', float('nan')):+.4f} "
            f"est={s.get('tau_est_mean', float('nan')):+.4f}"
            for s in result.get('segments', []) if s.get('valid'))
        print(f"{result['case']:4s} crash={result['crashed']} "
              f"omega_max={result.get('omega_abs_max')} :: {segs}")

    report['summary'] = {
        'cases_run': len(report['cases']),
        'cases_valid': sum(1 for c in report['cases'] if c.get('valid')),
        'any_crash': any(c['crashed'] for c in report['cases']),
        'any_nan': any(c.get('nan_theta', 0) or c.get('nan_omega', 0) for c in report['cases']),
    }
    (out_dir / 'reel_actuator_bench.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report['summary'], indent=2))


if __name__ == '__main__':
    main()
