"""Classificacao de aborto: construcao do modelo x divergencia simulando.

A distincao decide o proximo passo — um aborto de construcao se reproduz em 15 s num
mundo vazio, sem PX4; um dinamico so aparece rodando. Confundir os dois foi o que fez B2
registrar como fronteira de estabilidade um limite que era de construcao.
"""
import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    'classify_aborts', ROOT / 'tools' / 'classify_aborts.py')
classifier = importlib.util.module_from_spec(spec)
spec.loader.exec_module(classifier)

CONSTRUCAO = """
gz sim: ./dart/dynamics/Joint.cpp:537: virtual void
dart::dynamics::Joint::setTransformFromParentBodyNode(const Isometry3d&):
Assertion `math::verifyTransform(_T)' failed.
#11 in gz::sim::v7::systems::PhysicsPrivate::CreatePhysicsEntities
#6  in gz::physics::dartsim::SDFFeatures::ConstructSdfJoint
"""

DINAMICA_BALL = """
gz sim: ./dart/dynamics/BallJoint.cpp:159: virtual void
dart::dynamics::BallJoint::updateRelativeTransform() const:
Assertion `math::verifyTransform(mT)' failed.
#12 in gz::sim::v7::systems::PhysicsPrivate::Step
"""

DINAMICA_REVOLUTE = """
gz sim: ./dart/dynamics/RevoluteJoint.cpp:186: virtual void
dart::dynamics::RevoluteJoint::updateRelativeTransform() const:
Assertion `math::verifyTransform(mT)' failed.
#12 in gz::sim::v7::systems::PhysicsPrivate::Step
"""


def write(tmp_path, text):
    path = tmp_path / 'px4.log'
    path.write_text(text)
    return path


def test_construction_abort_is_named_as_such(tmp_path):
    assert classifier.classify(write(tmp_path, CONSTRUCAO)) == 'Joint.cpp:537 — CONSTRUCAO'


def test_ball_divergence_is_dynamic(tmp_path):
    assert classifier.classify(write(tmp_path, DINAMICA_BALL)) == 'BallJoint.cpp:159 — DINAMICA'


def test_revolute_divergence_is_dynamic(tmp_path):
    assert classifier.classify(write(tmp_path, DINAMICA_REVOLUTE)) == \
        'RevoluteJoint.cpp:186 — DINAMICA'


def test_ansi_escapes_from_the_px4_console_do_not_hide_the_assertion(tmp_path):
    noisy = DINAMICA_BALL.replace('dart/', '\x1b[2Kdart/').replace('#12', 'pxh> \x1b[0m#12')
    assert classifier.classify(write(tmp_path, noisy)) == 'BallJoint.cpp:159 — DINAMICA'


def test_log_without_assertion_is_not_reported_as_an_abort(tmp_path):
    assert classifier.classify(write(tmp_path, 'INFO [px4] Ready for takeoff!')) == \
        'sem assercao do DART'


def test_missing_log_is_reported(tmp_path):
    assert classifier.classify(tmp_path / 'nao_existe.log') == 'sem log'
