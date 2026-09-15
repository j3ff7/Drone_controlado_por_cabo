#!/usr/bin/env python3
"""PX4 SITL + X500 original + tether ligado por BallJoint, composto no launch.

Arquitetura validada na `dev` (docs/X500_TETHER_BALL_JOINT_INTEGRATION.md), portada sem tocar no
PX4. O gz-sim 7 nao cria uma junta `ball` entre modelos ja spawnados (o `DetachableJoint` so
cria `fixed` e o `UserCommands` nao tem servico de juntas), entao a conexao e composta ANTES do
spawn, num modelo wrapper gerado aqui e spawnado pelo proprio PX4 (`PX4_GZ_MODEL=<wrapper>`):

    <include merge="true"> model://x500           o X500 do PX4, sem copia; o merge mantem
                                                   base_link, sensores e motores no topo do modelo,
                                                   entao os topicos que o PX4 le nao mudam
    <include> model://tether_cable  (cabo_anexado)        pose relativa ao base_link
    <joint type="ball"> base_link -> cabo_anexado::raiz_cabo

Por que nao editar o `x500/model.sdf` do PX4: o PX4 fica fora do repositorio, uma edicao ali nao
e versionada e se perde (ou conflita) a cada atualizacao do PX4. O wrapper acompanha o `x500`
original automaticamente.

O `GZ_SIM_RESOURCE_PATH` nao e herdado do ambiente: com um overlay de `x500` ja modificado no
caminho, o include resolveria para um X500 que ja tem a junta (junta duplicada). O PX4 acrescenta
os proprios modelos DEPOIS deste caminho (gz_env.sh), entao `model://x500` e o original.
"""
import argparse
import os
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PX4 = ROOT / 'px4' / 'PX4-Autopilot'
PX4_MODELS = PX4 / 'Tools' / 'simulation' / 'gz' / 'models'
TETHER_MODELS = ROOT / 'src' / 'pacote_do_drone' / 'tether_package' / 'models'
DEFAULT_MODEL_DIR = ROOT / 'build' / 'x500_tether_ball' / 'models'

WRAPPER_SDF = """<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{name}">
    <include merge="true">
      <uri>model://x500</uri>
    </include>

    <include>
      <name>{cable_name}</name>
      <uri>{cable_uri}</uri>
      <pose relative_to="{parent_link}">{pose}</pose>
    </include>

    <joint name="{joint_name}" type="ball">
      <parent>{parent_link}</parent>
      <child>{cable_name}::{cable_root}</child>
    </joint>
  </model>
</sdf>
"""

CONFIG = """<?xml version="1.0" ?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>X500 do PX4 (include merge) + tether por BallJoint; gerado por tools/launch_x500_tether_ball.py</description>
</model>
"""


def wrapper_sdf(args):
    return WRAPPER_SDF.format(name=args.name, cable_name=args.cable_name, cable_uri=args.cable_uri,
                             pose=args.pose, joint_name=args.joint_name,
                             parent_link=args.parent_link, cable_root=args.cable_root)


def write_wrapper(model_root, args):
    model_dir = Path(model_root) / args.name
    model_dir.mkdir(parents=True, exist_ok=True)
    (model_dir / 'model.sdf').write_text(wrapper_sdf(args))
    (model_dir / 'model.config').write_text(CONFIG.format(name=args.name))
    return model_dir


def resource_path(model_root):
    return ':'.join([str(Path(model_root).resolve()), str(TETHER_MODELS)])


def validate(model_dir, model_root):
    path = resource_path(model_root) + ':' + str(PX4_MODELS)      # mesma ordem que o PX4 monta
    env = dict(os.environ, GZ_SIM_RESOURCE_PATH=path, SDF_PATH=path)
    out = subprocess.run(['gz', 'sdf', '-k', str(model_dir / 'model.sdf')], env=env,
                         capture_output=True, text=True)
    lines = [l for l in (out.stdout + out.stderr).splitlines() if 'dynamics' not in l and l.strip()]
    return out.returncode == 0 and lines and lines[-1].strip() == 'Valid.', lines


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--name', default='x500_tether_ball', help='modelo wrapper; o PX4 o chama de <name>_0')
    parser.add_argument('--cable-uri', default='model://tether_cable')
    parser.add_argument('--cable-name', default='cabo_anexado')
    parser.add_argument('--cable-root', default='raiz_cabo')
    parser.add_argument('--parent-link', default='base_link')
    parser.add_argument('--joint-name', default='drone_cabo_joint')
    parser.add_argument('--pose', default='0 0 0.2 0 0 0',
                        help='pose do cabo relativa ao --parent-link (o x500 mesclado traz '
                             '<pose>0 0 .24</pose>; relativa ao wrapper a raiz ficaria mais baixa)')
    parser.add_argument('--model-dir', default=str(DEFAULT_MODEL_DIR))
    parser.add_argument('--only-generate', action='store_true', help='gera e valida, sem iniciar o PX4')
    parser.add_argument('--print-env', action='store_true', help='imprime os exports para uso manual')
    parser.add_argument('--headless', action='store_true')
    args = parser.parse_args()

    if not (TETHER_MODELS / 'tether_cable' / 'model.sdf').exists():
        raise SystemExit(f'modelo do cabo ausente em {TETHER_MODELS}/tether_cable')
    model_dir = write_wrapper(args.model_dir, args)
    ok, lines = validate(model_dir, args.model_dir)
    print(f'wrapper: {model_dir}')
    print('gz sdf -k:', lines[-1] if lines else '(sem saida)')
    if not ok:
        print('\n'.join(lines[-5:]), file=sys.stderr)
        raise SystemExit('wrapper invalido; PX4 nao iniciado')
    env = dict(os.environ, GZ_SIM_RESOURCE_PATH=resource_path(args.model_dir), PX4_GZ_MODEL=args.name)
    env.pop('PX4_GZ_MODEL_NAME', None)       # PX4 tem de spawnar o wrapper, nao se ligar a outro modelo
    if args.headless:
        env['HEADLESS'] = '1'
    if args.print_env:
        print(f'export GZ_SIM_RESOURCE_PATH={env["GZ_SIM_RESOURCE_PATH"]}')
        print(f'export PX4_GZ_MODEL={args.name}')
    if args.only_generate:
        return
    os.chdir(PX4)
    os.execvpe('make', ['make', 'px4_sitl', 'gz_x500'], env)


if __name__ == '__main__':
    main()
