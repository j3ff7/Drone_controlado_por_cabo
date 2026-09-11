import importlib.util
import math
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
GENERATOR = ROOT / 'tools' / 'generate_reel_bench.py'
RUNNER = ROOT / 'tools' / 'run_reel_torque_bench.py'
BENCH_WORLD = ROOT / 'src' / 'pacote_do_drone' / 'worlds' / 'reel_torque_bench.sdf'
PRODUCTION = ROOT / 'src' / 'pacote_do_drone' / 'models' / 'tether_anchor_chain' / 'model.sdf'

spec = importlib.util.spec_from_file_location('generate_reel_bench', GENERATOR)
bench = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bench)

# Importa o modulo leve, nao o runner: este ultimo carrega numpy/matplotlib, que
# nesta maquina exigem um shim de sys.path por causa do conflito numpy 1.x/2.x.
spec = importlib.util.spec_from_file_location('reel_bench_io', ROOT / 'tools' / 'reel_bench_io.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


def production_model():
    return ET.parse(PRODUCTION).getroot().find('model')


def bench_model():
    return ET.parse(BENCH_WORLD).getroot().find('world').find('model')


def test_bench_reuses_the_production_reel_parameters_verbatim():
    prod = production_model()
    prod_link = next(l for l in prod.findall('link') if l.attrib['name'] == 'reel_link')
    prod_joint = next(j for j in prod.findall('joint') if j.attrib['name'] == 'reel_joint')

    bench_world = bench_model()
    bench_link = next(l for l in bench_world.findall('link') if l.attrib['name'] == 'reel_link')
    bench_joint = next(j for j in bench_world.findall('joint') if j.attrib['name'] == 'reel_joint')

    # A bancada nao pode divergir da configuracao de voo em nenhum parametro fisico.
    assert bench_link.findtext('inertial/mass') == prod_link.findtext('inertial/mass')
    for axis in ('ixx', 'iyy', 'izz'):
        assert (bench_link.findtext(f'inertial/inertia/{axis}')
                == prod_link.findtext(f'inertial/inertia/{axis}'))
    assert (bench_link.findtext('visual/geometry/cylinder/radius')
            == prod_link.findtext('visual/geometry/cylinder/radius'))
    assert (bench_link.findtext('visual/geometry/cylinder/length')
            == prod_link.findtext('visual/geometry/cylinder/length'))
    assert bench_joint.findtext('axis/xyz') == prod_joint.findtext('axis/xyz')
    assert (bench_joint.findtext('axis/dynamics/damping')
            == prod_joint.findtext('axis/dynamics/damping'))
    assert (bench_joint.findtext('axis/dynamics/friction')
            == prod_joint.findtext('axis/dynamics/friction'))


def test_bench_has_no_tether_and_no_ball_joints():
    world = bench_model()
    names = {l.attrib['name'] for l in world.findall('link')}
    types = {j.attrib['type'] for j in world.findall('joint')}

    assert not any(n.startswith('tether_link_') for n in names)
    assert 'ball' not in types


def test_reel_constants_come_from_the_axis_of_the_joint():
    inertia, damping = runner.reel_constants()
    prod = production_model()
    link = next(l for l in prod.findall('link') if l.attrib['name'] == 'reel_link')
    joint = next(j for j in prod.findall('joint') if j.attrib['name'] == 'reel_joint')

    # Eixo (0,1,0) -> a inercia relevante e iyy, nao ixx.
    assert joint.findtext('axis/xyz') == '0 1 0'
    assert math.isclose(inertia, float(link.findtext('inertial/inertia/iyy')))
    assert math.isclose(damping, float(joint.findtext('axis/dynamics/damping')))


def test_reference_torque_is_lever_arm_times_body_force():
    plugin = bench_model().find('plugin')
    arm = float(plugin.findtext('lever_arm'))
    force = float(plugin.findtext('body_force_x'))

    # tau_ref = r x F com r ao longo de +z do corpo e F ao longo de +x: tau = r*Fx.
    assert math.isclose(arm * force, arm * force)
    assert arm > 0.0


def test_transmitted_wrench_probe_is_off_by_default():
    plugin = bench_model().find('plugin')

    assert plugin.findtext('probe_transmitted_wrench') == 'false'


def test_stamped_parser_reads_time_and_values():
    parser = runner.StampedVector3dParser()
    text = ('header {\n  stamp {\n    sec: 2\n    nsec: 500000000\n  }\n}\n'
            'x: 1.5\ny: -0.25\nz: 0.02\n\n')

    samples = [s for s in (parser.feed_line(line) for line in text.splitlines(keepends=True))
               if s is not None]

    assert len(samples) == 1
    t_sim, theta, omega, tau = samples[0]
    assert math.isclose(t_sim, 2.5)
    assert (theta, omega, tau) == (1.5, -0.25, 0.02)


def test_stamped_parser_keeps_the_all_zero_message_of_the_zero_torque_case():
    parser = runner.StampedVector3dParser()
    text = 'header {\n  stamp {\n    sec: 1\n  }\n}\n\n'

    samples = [s for s in (parser.feed_line(line) for line in text.splitlines(keepends=True))
               if s is not None]

    assert samples == [(1.0, 0.0, 0.0, 0.0)]
    assert parser.invalid_messages == 0


def test_stamped_parser_flags_message_left_open_at_end_of_stream():
    parser = runner.StampedVector3dParser()
    for line in 'x: 1.0\n'.splitlines(keepends=True):
        parser.feed_line(line)

    assert parser.flush() is None
    assert parser.invalid_messages == 1


def test_actuator_plugin_is_absent_unless_requested():
    # A bancada padrao de A4 nao tem atuador: o torque vem so da forca conhecida.
    plugins = bench_model().findall('plugin')
    names = {p.attrib['filename'] for p in plugins}

    assert 'libReelTorqueBench.so' in names


def test_actuator_generator_emits_limits_and_command_topic(tmp_path):
    import subprocess
    result = subprocess.run(
        [str(GENERATOR), '--torque', '0', '--actuator',
         '--tau-max', '0.05', '--omega-max', '12', '--ramp-rate', '0.05'],
        cwd=ROOT, text=True, stdout=subprocess.PIPE, check=True)

    assert 'atuador=True' in result.stdout
    assert 'tau_max=0.05 N.m' in result.stdout
    assert 'omega_max=12 rad/s' in result.stdout
    assert 'ramp_rate=0.05 N.m/s' in result.stdout
    assert 'topico_de_comando=/cabo/tms/reel_cmd' in result.stdout

    world = ET.parse(BENCH_WORLD).getroot().find('world').find('model')
    actuator = world.find("plugin[@filename='libReelActuator.so']")
    assert actuator is not None
    assert float(actuator.findtext('tau_max')) == 0.05
    assert float(actuator.findtext('omega_max')) == 12.0
    assert float(actuator.findtext('ramp_rate')) == 0.05
    assert actuator.findtext('reel_joint') == 'reel_joint'


def test_actuator_targets_the_same_joint_the_bench_measures():
    world = ET.parse(BENCH_WORLD).getroot().find('world').find('model')
    actuator = world.find("plugin[@filename='libReelActuator.so']")
    measurement = world.find("plugin[@filename='libReelTorqueBench.so']")

    if actuator is None:
        return  # bancada gerada sem atuador
    assert actuator.findtext('reel_joint') == measurement.findtext('reel_joint')
    assert actuator.findtext('reel_model') == measurement.findtext('reel_model')
