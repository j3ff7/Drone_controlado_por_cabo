"""X500 model checks independent of a running PX4 process."""

from pathlib import Path

import numpy as np
import pytest

from mujoco_px4_sitl.config import Config
from mujoco_px4_sitl.rotorconfig import load_rotors
from mujoco_px4_sitl.sim import MujocoPhysics


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def physics() -> MujocoPhysics:
    cfg = Config(model_path=ROOT / "models/x500.xml", imu_model="ideal")
    return MujocoPhysics(cfg, load_rotors(ROOT / "configs/x500_rotors.yaml"))


def test_mass_matches_aggregated_gazebo_x500(physics: MujocoPhysics) -> None:
    assert physics.vehicle.total_mass == pytest.approx(2.0643076923076924, abs=1e-12)


def test_rotor_geometry_and_order_match_x500(physics: MujocoPhysics) -> None:
    expected = np.array([
        [0.174, -0.174, 0.06 - 0.00186913],
        [-0.174, 0.174, 0.06 - 0.00186913],
        [0.174, 0.174, 0.06 - 0.00186913],
        [-0.174, -0.174, 0.06 - 0.00186913],
    ])
    assert physics.vehicle.arm_body == pytest.approx(expected)
    assert physics.vehicle.params.spin == (+1, +1, -1, -1)


def test_rotor_coefficients_match_x500_sdf(physics: MujocoPhysics) -> None:
    assert physics.vehicle.c_t == pytest.approx(np.full(4, 8.54858e-06))
    assert physics.vehicle.km == pytest.approx(np.full(4, 0.016))
    assert physics.vehicle.omega_max == pytest.approx(np.full(4, 1000.0))


def test_equal_hover_commands_balance_weight_without_moment(physics: MujocoPhysics) -> None:
    vehicle = physics.vehicle
    vehicle.update_commands(np.full(4, vehicle.hover_command()), 10.0)
    force, moment = vehicle.wrench_body()
    assert force[2] == pytest.approx(vehicle.weight, rel=1e-9)
    assert moment == pytest.approx(np.zeros(3), abs=1e-12)
