#!/usr/bin/env python3
"""Executa a bancada A4 e estima o torque do reel pela dinamica.

Para cada caso de torque: gera o mundo, sobe o `gz sim` headless, captura
`/bancada/reel/state` e o relogio `/stats`, encerra, e compara o torque de referencia
`tau_ref = r x F` com a estimativa dinamica

    tau_est = I*alpha + b*omega

com `I` (inercia axial) e `b` (damping) lidos do proprio modelo de producao e `alpha`
derivado de `omega(t)` sobre tempo **simulado**.
"""
import sys

sys.path = ['/usr/lib/python3/dist-packages'] + [p for p in sys.path if 'local/lib' not in p]

import argparse  # noqa: E402
import bisect  # noqa: E402
import csv  # noqa: E402
import importlib.util  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
import re  # noqa: E402
import signal  # noqa: E402
import subprocess  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402
import xml.etree.ElementTree as ET  # noqa: E402
from pathlib import Path  # noqa: E402

import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT / 'src' / 'pacote_do_drone' / 'worlds' / 'reel_torque_bench.sdf'
PRODUCTION = ROOT / 'src' / 'pacote_do_drone' / 'models' / 'tether_anchor_chain' / 'model.sdf'
PLUGIN_PATH = ROOT / 'build' / 'gz_plugins'
STATE_TOPIC = '/bancada/reel/state'
WRENCH_TOPIC = '/bancada/reel/wrench_torque'


def _load_module(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'tools' / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_recorder():
    """Reaproveita os parsers ja testados de record_tether_timeseries.py."""
    return _load_module('record_tether_timeseries')


REC = _load_recorder()
BENCH_IO = _load_module('reel_bench_io')
StampedVector3dParser = BENCH_IO.StampedVector3dParser
reel_constants = BENCH_IO.reel_constants


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


def centered_derivative(t, values, half=5, min_dt=1e-4):
    out = np.full_like(values, np.nan)
    for i in range(len(values)):
        lo, hi = max(0, i - half), min(len(values) - 1, i + half)
        dt = t[hi] - t[lo]
        if dt > min_dt:
            out[i] = (values[hi] - values[lo]) / dt
    return out


def run_case(torque, duration, settle, inertia, damping, probe_wrench, out_dir, label):
    subprocess.run([str(ROOT / 'tools' / 'generate_reel_bench.py'), '--torque', str(torque)]
                   + (['--probe-transmitted-wrench'] if probe_wrench else []),
                   check=True, stdout=subprocess.DEVNULL)

    env = {**__import__('os').environ,
           'GZ_SIM_SYSTEM_PLUGIN_PATH': f'{PLUGIN_PATH}:'
                                        f'{__import__("os").environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")}'}
    sim = subprocess.Popen(['gz', 'sim', '-s', '-r', str(WORLD)],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           text=True, env=env)

    stop = threading.Event()
    caps = {'state': Capture(STATE_TOPIC, StampedVector3dParser, stop),
            'world': Capture('/stats', REC.WorldStatsStreamParser, stop)}
    if probe_wrench:
        caps['wrench'] = Capture(WRENCH_TOPIC, REC.Vector3dStreamParser, stop)

    deadline = time.time() + 25.0
    ready = False
    while time.time() < deadline and not ready:
        listing = subprocess.run(['gz', 'topic', '-l'], capture_output=True, text=True)
        ready = STATE_TOPIC in listing.stdout
        if sim.poll() is not None:
            break
        if not ready:
            time.sleep(0.5)

    crashed = sim.poll() is not None
    if not crashed:
        for cap in caps.values():
            cap.start()
        time.sleep(duration)
        stop.set()
        for cap in caps.values():
            cap.terminate()
        for cap in caps.values():
            cap.join(timeout=5)

    sim.terminate()
    try:
        sim_out, _ = sim.communicate(timeout=10)
    except subprocess.TimeoutExpired:
        sim.kill()
        sim_out, _ = sim.communicate()

    result = {'label': label, 'tau_ref_nm': torque, 'crashed': bool(crashed),
              'probe_transmitted_wrench': probe_wrench,
              'sim_stderr_tail': '\n'.join(sim_out.splitlines()[-6:]) if sim_out else ''}
    if crashed or not caps['state'].samples:
        result.update({'samples': 0, 'valid': False})
        return result, None

    world_rows = [{'t_wall': t, 'sim_time': v[0], 'real_time': v[1], 'rtf': v[2]}
                  for t, v in caps['world'].samples]

    t = np.array([s[0] for _, s in caps['state'].samples])
    theta = np.array([s[1] for _, s in caps['state'].samples])
    omega = np.array([s[2] for _, s in caps['state'].samples])
    tau_ref = np.array([s[3] for _, s in caps['state'].samples])
    ok = np.isfinite(t) & np.isfinite(theta) & np.isfinite(omega)
    t, theta, omega, tau_ref = t[ok], theta[ok], omega[ok], tau_ref[ok]

    alpha = centered_derivative(t, omega)
    tau_est = inertia * alpha + damping * omega

    # Descarta o transiente inicial de arranque do stream e as bordas da derivada.
    window = np.isfinite(alpha) & (t > t[0] + settle)
    err = tau_est[window] - tau_ref[window]
    denom = abs(torque) if abs(torque) > 1e-12 else None

    if probe_wrench and caps.get('wrench') and caps['wrench'].samples:
        axial = np.array([s[0] for _, s in caps['wrench'].samples])
        flags = np.array([s[2] for _, s in caps['wrench'].samples])
        good = np.isfinite(axial) & (flags >= 0.5)
        if good.any():
            axial_ok = axial[good]
            # O wrench transmitido e a reacao da junta: sinal oposto ao torque aplicado.
            result['wrench_probe'] = {
                'samples': int(good.sum()),
                'available_fraction': float(np.mean(flags >= 0.5)),
                'axial_mean_nm': float(np.mean(axial_ok)),
                'axial_std_nm': float(np.std(axial_ok)),
                'sign_inverted_vs_applied': bool(torque != 0 and np.mean(axial_ok) * torque < 0),
                'abs_error_vs_tau_ref_nm': float(np.mean(np.abs(np.abs(axial_ok) - abs(torque)))),
            }

    rtfs = np.array([r['rtf'] for r in world_rows if math.isfinite(r['rtf'])])
    result.update({
        'samples': int(len(t)),
        'samples_in_window': int(np.count_nonzero(window)),
        'valid': bool(np.count_nonzero(window) > 50),
        'theta_rad': {'min': float(theta.min()), 'max': float(theta.max())},
        'omega_rad_s': {'min': float(omega.min()), 'max': float(omega.max()),
                        'final': float(omega[-1])},
        'alpha_rad_s2': {'max_abs': float(np.nanmax(np.abs(alpha[window]))) if window.any() else None},
        'tau_est_nm': {'mean': float(np.mean(tau_est[window])),
                       'std': float(np.std(tau_est[window]))} if window.any() else None,
        'abs_error_nm': {'mean': float(np.mean(np.abs(err))),
                         'max': float(np.max(np.abs(err)))} if window.any() else None,
        'rel_error_pct': (float(100.0 * np.mean(np.abs(err)) / denom)
                          if denom and window.any() else None),
        'omega_steady_predicted': (torque / damping) if damping else None,
        'rtf_mean': float(np.mean(rtfs)) if rtfs.size else None,
    })
    return result, (t, theta, omega, alpha, tau_ref, tau_est, window)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', default='results/a4')
    parser.add_argument('--duration', type=float, default=6.0)
    parser.add_argument('--settle', type=float, default=0.3)
    parser.add_argument('--repeats', type=int, default=1)
    parser.add_argument('--probe-transmitted-wrench', action='store_true')
    parser.add_argument('--cases', type=float, nargs='+',
                        default=[0.0, 0.02, -0.02, 0.005, 0.05])
    args = parser.parse_args()

    inertia, damping = reel_constants()
    out_dir = Path(args.output_dir)
    plot_dir = out_dir / 'plots'
    plot_dir.mkdir(parents=True, exist_ok=True)

    report = {'inertia_axial_kg_m2': inertia, 'damping_nms_rad': damping,
              'estimator': 'tau_est = I*alpha + b*omega', 'cases': []}

    for torque in args.cases:
        for rep in range(args.repeats):
            label = f'tau={torque:+.4g}' + (f' rep{rep + 1}' if args.repeats > 1 else '')
            result, series = run_case(torque, args.duration, args.settle, inertia, damping,
                                      args.probe_transmitted_wrench, out_dir, label)
            report['cases'].append(result)
            print(f"{label:22s} crash={result['crashed']} samples={result.get('samples')} "
                  f"tau_est={result.get('tau_est_nm', {}) and result['tau_est_nm']['mean']} "
                  f"err_abs={result.get('abs_error_nm', {}) and result['abs_error_nm']['mean']}")

            if series and rep == 0:
                t, theta, omega, alpha, tau_ref, tau_est, window = series
                slug = f"tau_{torque:+.4g}".replace('+', 'p').replace('-', 'm').replace('.', '_')
                with (out_dir / f'{slug}.csv').open('w', newline='') as handle:
                    writer = csv.writer(handle)
                    writer.writerow(['t_sim', 'theta', 'omega', 'alpha', 'tau_ref', 'tau_est'])
                    writer.writerows(zip(t, theta, omega, alpha, tau_ref, tau_est))

                fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
                axes[0].plot(t, omega, lw=0.8)
                if damping:
                    axes[0].axhline(torque / damping, ls='--', color='k', lw=0.8,
                                    label=f'omega_ss = tau/b = {torque / damping:.3g}')
                    axes[0].legend(fontsize=8)
                axes[0].set_ylabel('omega [rad/s]')
                axes[0].grid(alpha=0.3)
                axes[1].plot(t, alpha, lw=0.8, color='tab:orange')
                axes[1].set_ylabel('alpha [rad/s2]')
                axes[1].grid(alpha=0.3)
                axes[2].plot(t, tau_est, lw=0.8, color='tab:green', label='tau_est = I*alpha + b*omega')
                axes[2].plot(t, tau_ref, ls='--', color='k', lw=1.0, label='tau_ref = r x F')
                axes[2].set_ylabel('torque [N.m]')
                axes[2].set_xlabel('t_sim [s]')
                axes[2].grid(alpha=0.3)
                axes[2].legend(fontsize=8)
                fig.suptitle(f'Bancada A4 - {label}')
                fig.tight_layout()
                fig.savefig(plot_dir / f'{slug}.png', dpi=110)
                plt.close(fig)

    ok = [c for c in report['cases'] if c.get('valid')]
    report['summary'] = {
        'cases_run': len(report['cases']),
        'cases_valid': len(ok),
        'any_crash': any(c['crashed'] for c in report['cases']),
        'max_abs_error_nm': max((c['abs_error_nm']['max'] for c in ok if c.get('abs_error_nm')),
                                default=None),
        'max_rel_error_pct': max((c['rel_error_pct'] for c in ok if c.get('rel_error_pct')),
                                 default=None),
    }
    (out_dir / 'reel_torque_bench.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report['summary'], indent=2))


if __name__ == '__main__':
    main()
