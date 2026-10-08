"""Regression tests for the endpoint frame error found during M1."""

from pathlib import Path

import mujoco
import numpy as np
import pytest

from mujoco_px4_sitl.config import Config
from mujoco_px4_sitl.rotorconfig import load_rotors
from mujoco_px4_sitl.sim import MujocoPhysics


ROOT = Path(__file__).resolve().parents[1]


def test_connect_anchor_is_the_last_link_endpoint() -> None:
    model = mujoco.MjModel.from_xml_path(str(ROOT / "models/x500_tether_n30.xml"))
    equality = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_EQUALITY, "tether_uav_connection"
    )
    endpoint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tether_endpoint")
    # eq_data[0:3] is connect.anchor in body1's local frame.
    assert model.eq_data[equality, :3] == pytest.approx(model.site_pos[endpoint])


def test_endpoint_constraint_stays_closed() -> None:
    cfg = Config(model_path=ROOT / "models/x500_tether_n30.xml", imu_model="ideal")
    physics = MujocoPhysics(cfg, load_rotors(ROOT / "configs/x500_rotors.yaml"))
    errors = []
    for _ in range(50):
        physics.step_frame(np.zeros(4))
        errors.append(physics.state().tether["connection_error_norm"])
    assert max(errors) < 1e-3
    assert physics.state().tether["anchor_drift"] == pytest.approx(0.0)
