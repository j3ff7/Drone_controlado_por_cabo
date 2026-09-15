#!/usr/bin/env python3
"""Copia o mundo `default` do PX4 com outro passo de fisica.

A baseline do tether com colisao exige `max_step_size = 0.001 s`; o mundo do PX4 usa 0.004 s. O
arquivo do PX4 e so lido: a copia vai para --output e o Gazebo deve servi-la ANTES do PX4, que se
liga a um mundo ja rodando (px4-rc.simulator: "gazebo already running world: default").

    ./tools/generate_px4_world_step.py --step 0.001 --output results/tether_angles/manual/world/default.sdf
"""
import argparse
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PX4_DEFAULT_WORLD = ROOT / 'px4' / 'PX4-Autopilot' / 'Tools' / 'simulation' / 'gz' / 'worlds' / 'default.sdf'
OLD_STEP = '<max_step_size>0.004</max_step_size>'
OLD_RATE = '<real_time_update_rate>250</real_time_update_rate>'


def world_with_step(text, step):
    """Troca passo e taxa de atualizacao (1/passo, para RTF = 1) de um mundo default do PX4."""
    if step <= 0.0:
        raise ValueError('step must be positive')
    if text.count(OLD_STEP) != 1 or text.count(OLD_RATE) != 1:
        raise ValueError('mundo default do PX4 inesperado: passo/taxa nao encontrados uma vez')
    text = text.replace(OLD_STEP, f'<max_step_size>{step:g}</max_step_size>')
    return text.replace(OLD_RATE, f'<real_time_update_rate>{round(1.0 / step):d}</real_time_update_rate>')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--step', type=float, default=0.001, help='max_step_size [s]')
    parser.add_argument('--source', default=str(PX4_DEFAULT_WORLD))
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    try:
        out.write_text(world_with_step(Path(args.source).read_text(), args.step))
    except ValueError as exc:
        raise SystemExit(str(exc))
    print(f'world={out} max_step_size={args.step:g} real_time_update_rate={round(1.0 / args.step)}')


if __name__ == '__main__':
    main()
