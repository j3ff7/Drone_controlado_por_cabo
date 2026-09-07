#!/usr/bin/env python3
"""Grava series temporais dos topicos /cabo/* e do relogio /stats do Gazebo.

Cada mensagem de texto emitida por `gz topic -e` e fechada por uma linha em branco.
Marcamos o instante de parede da chegada e depois convertemos para tempo de
simulacao interpolando o mapeamento wall->sim amostrado em /stats.
"""
import argparse
import bisect
import csv
import json
import math
import re
import signal
import subprocess
import threading
import time
from pathlib import Path


VECTOR_FIELD_RE = re.compile(
    r'\s*([xyz]):\s*('
    r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?'
    r'|[-+]?inf'
    r'|nan'
    r')\s*$',
    re.IGNORECASE,
)

SCALAR_FIELD_RE = re.compile(r'\s*([a-z_]+):\s*(\S+)\s*$')


class Vector3dStreamParser:
    """Fecha uma amostra por delimitador de mensagem, nunca por presenca de x/y."""

    def __init__(self):
        self.current = {}
        self.invalid_messages = 0

    def feed_line(self, line):
        if not line.strip():
            return self._finish()
        match = VECTOR_FIELD_RE.match(line)
        if not match:
            return None
        field = match.group(1).lower()
        finished = None
        if field in self.current:
            finished = self._finish()
        self.current[field] = float(match.group(2))
        return finished

    def flush(self):
        return self._finish()

    def _finish(self):
        if not self.current:
            return None
        current, self.current = self.current, {}
        if 'x' not in current or 'y' not in current:
            self.invalid_messages += 1
            return None
        return (current['x'], current['y'], current.get('z', 0.0))


class WorldStatsStreamParser:
    """Extrai sim_time, real_time e real_time_factor de gz.msgs.WorldStatistics."""

    def __init__(self):
        self.current = {}
        self.block = None
        self.invalid_messages = 0

    def feed_line(self, line):
        stripped = line.strip()
        if not stripped:
            return self._finish()
        if stripped.endswith('{'):
            self.block = stripped[:-1].strip()
            self.current.setdefault(self.block, {})
            return None
        if stripped == '}':
            self.block = None
            return None
        match = SCALAR_FIELD_RE.match(line)
        if not match:
            return None
        key, raw = match.group(1), match.group(2)
        if self.block is not None:
            self.current[self.block][key] = raw
        else:
            self.current[key] = raw
        return None

    def flush(self):
        return self._finish()

    def _finish(self):
        if not self.current:
            return None
        current, self.current = self.current, {}
        self.block = None
        sim = current.get('sim_time')
        # Um bloco `sim_time {` aberto e cortado no fim do stream chega vazio;
        # tratar isso como 0 s inventaria um retrocesso do relogio.
        if not sim or not ('sec' in sim or 'nsec' in sim):
            self.invalid_messages += 1
            return None
        try:
            sim_time = int(sim.get('sec', 0)) + int(sim.get('nsec', 0)) * 1e-9
            real = current.get('real_time', {})
            real_time = int(real.get('sec', 0)) + int(real.get('nsec', 0)) * 1e-9
            rtf = float(current.get('real_time_factor', 'nan'))
        except (TypeError, ValueError):
            self.invalid_messages += 1
            return None
        return (sim_time, real_time, rtf)


def build_sim_clock(stats_rows):
    """Mapeia wall-clock -> sim time por interpolacao linear das amostras de /stats."""
    walls = [row['t_wall'] for row in stats_rows]
    sims = [row['sim_time'] for row in stats_rows]

    def to_sim(t_wall):
        if not walls:
            return None
        index = bisect.bisect_left(walls, t_wall)
        if index <= 0:
            return sims[0] + (t_wall - walls[0])
        if index >= len(walls):
            return sims[-1] + (t_wall - walls[-1])
        w0, w1 = walls[index - 1], walls[index]
        s0, s1 = sims[index - 1], sims[index]
        if w1 <= w0:
            return s0
        return s0 + (s1 - s0) * (t_wall - w0) / (w1 - w0)

    return to_sim


class TopicRecorder(threading.Thread):
    def __init__(self, topic, parser_factory, stop_event):
        super().__init__(daemon=True)
        self.topic = topic
        self.parser = parser_factory()
        self.stop_event = stop_event
        self.samples = []
        self.process = None
        self.error = None

    def run(self):
        try:
            self.process = subprocess.Popen(
                ['gz', 'topic', '-e', '-t', self.topic],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                bufsize=1,
            )
            for line in self.process.stdout:
                sample = self.parser.feed_line(line)
                if sample is not None:
                    self.samples.append((time.time(), sample))
                if self.stop_event.is_set():
                    break
            sample = self.parser.flush()
            if sample is not None:
                self.samples.append((time.time(), sample))
        except Exception as exc:  # pragma: no cover - falha de ambiente
            self.error = repr(exc)

    def terminate(self):
        if self.process and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--prefix', default='tether')
    args = parser.parse_args()

    stop_event = threading.Event()
    recorders = {
        'error': TopicRecorder('/cabo/conexao/error', Vector3dStreamParser, stop_event),
        'force': TopicRecorder('/cabo/conexao/force', Vector3dStreamParser, stop_event),
        'stats': TopicRecorder('/cabo/conexao/stats', Vector3dStreamParser, stop_event),
        'anchor': TopicRecorder('/cabo/anchor/stats', Vector3dStreamParser, stop_event),
        'world': TopicRecorder('/stats', WorldStatsStreamParser, stop_event),
    }
    for recorder in recorders.values():
        recorder.start()

    time.sleep(args.duration)
    stop_event.set()
    for recorder in recorders.values():
        recorder.terminate()
    for recorder in recorders.values():
        recorder.join(timeout=5)

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    world_rows = [
        {'t_wall': t, 'sim_time': value[0], 'real_time': value[1], 'rtf': value[2]}
        for t, value in recorders['world'].samples
    ]
    to_sim = build_sim_clock(world_rows)

    manifest = {'duration_s': args.duration, 'topics': {}}

    world_path = out_dir / f'{args.prefix}_world_stats.csv'
    with world_path.open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=['t_wall', 'sim_time', 'real_time', 'rtf'])
        writer.writeheader()
        writer.writerows(world_rows)
    rtfs = [row['rtf'] for row in world_rows if math.isfinite(row['rtf'])]
    manifest['rtf_samples'] = len(rtfs)
    manifest['rtf_mean'] = (sum(rtfs) / len(rtfs)) if rtfs else None
    manifest['rtf_min'] = min(rtfs) if rtfs else None
    manifest['sim_time_span_s'] = (
        world_rows[-1]['sim_time'] - world_rows[0]['sim_time'] if len(world_rows) > 1 else None
    )

    for key in ('error', 'force', 'stats', 'anchor'):
        recorder = recorders[key]
        path = out_dir / f'{args.prefix}_{key}.csv'
        with path.open('w', newline='') as handle:
            writer = csv.writer(handle)
            writer.writerow(['t_wall', 't_sim', 'x', 'y', 'z'])
            for t_wall, (x, y, z) in recorder.samples:
                writer.writerow([t_wall, to_sim(t_wall), x, y, z])
        manifest['topics'][recorder.topic] = {
            'csv': str(path),
            'samples': len(recorder.samples),
            'invalid_messages': recorder.parser.invalid_messages,
            'subprocess_error': recorder.error,
        }

    manifest_path = out_dir / f'{args.prefix}_record_manifest.json'
    manifest_path.write_text(json.dumps(manifest, indent=2))
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
