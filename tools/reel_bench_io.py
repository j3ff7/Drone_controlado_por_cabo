#!/usr/bin/env python3
"""Leitura de dados da bancada do reel: parser e constantes fisicas.

Fica separado de `run_reel_torque_bench.py` de proposito: aqui so entram stdlib e
xml, para que os testes possam exercitar a logica sem arrastar numpy/matplotlib —
que nesta maquina exigem um shim de sys.path por causa do conflito numpy 1.x/2.x.
"""
import re
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PRODUCTION = ROOT / 'src' / 'pacote_do_drone' / 'models' / 'tether_anchor_chain' / 'model.sdf'

AXIS_INERTIA = {0: 'ixx', 1: 'iyy', 2: 'izz'}


class StampedVector3dParser:
    """Le `gz.msgs.Vector3d` com header carimbado: devolve (t_sim, x, y, z).

    O tempo vem do carimbo da propria mensagem, nao de interpolacao wall->sim, o que
    importa porque a estimativa de torque depende de derivar omega. Campos ausentes
    valem 0 (o protobuf de texto omite zeros); a mensagem so e invalida se ficou
    aberta no fim do stream.
    """

    FIELD = re.compile(r'^\s*([xyz]):\s*(\S+)\s*$')
    STAMP = re.compile(r'^\s*(sec|nsec):\s*(-?\d+)\s*$')

    def __init__(self):
        self.current = {}
        self.saw_content = False
        self.invalid_messages = 0

    def feed_line(self, line):
        if not line.strip():
            return self._finish()
        self.saw_content = True
        stamp = self.STAMP.match(line)
        if stamp:
            self.current[stamp.group(1)] = int(stamp.group(2))
            return None
        field = self.FIELD.match(line)
        if field:
            try:
                self.current[field.group(1)] = float(field.group(2))
            except ValueError:
                self.current[field.group(1)] = float('nan')
        return None

    def flush(self):
        return self._finish(delimited=False)

    def _finish(self, delimited=True):
        if not self.saw_content:
            return None
        current, self.current, self.saw_content = self.current, {}, False
        if not delimited:
            self.invalid_messages += 1
            return None
        t_sim = current.get('sec', 0) + current.get('nsec', 0) * 1e-9
        return (t_sim, current.get('x', 0.0), current.get('y', 0.0), current.get('z', 0.0))


def reel_constants(model_path=PRODUCTION):
    """Inercia em torno do eixo da junta e damping, lidos do modelo de producao."""
    model = ET.parse(model_path).getroot().find('model')
    link = next(l for l in model.findall('link') if l.attrib['name'] == 'reel_link')
    joint = next(j for j in model.findall('joint') if j.attrib['name'] == 'reel_joint')

    axis = [abs(float(v)) for v in joint.findtext('axis/xyz').split()]
    key = AXIS_INERTIA[max(range(3), key=lambda i: axis[i])]
    inertia = float(link.findtext(f'inertial/inertia/{key}'))
    damping = float(joint.findtext('axis/dynamics/damping'))
    return inertia, damping
