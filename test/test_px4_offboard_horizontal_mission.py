import importlib.util
from pathlib import Path
from types import SimpleNamespace

from pymavlink import mavutil


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'tools' / 'px4_offboard_horizontal_mission.py'


spec = importlib.util.spec_from_file_location('px4_offboard_horizontal_mission', SCRIPT)
mission = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mission)


def test_px4_offboard_custom_mode_encoding():
    assert mission.px4_custom_mode(mission.PX4_CUSTOM_MAIN_MODE_OFFBOARD) == 6 << 16


def test_position_setpoint_mask_keeps_position_and_yaw_active():
    mask = mission.TYPE_MASK_POSITION_YAW

    assert mask & mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    assert mask & mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    assert mask & mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    assert mask & mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    assert mask & mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    assert mask & mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    assert mask & mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    assert not (mask & mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE)
    assert not (mask & mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE)
    assert not (mask & mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE)
    assert not (mask & mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE)


def test_norm_helpers():
    assert mission.norm2(3.0, 4.0) == 5.0
    assert mission.norm3(2.0, 3.0, 6.0) == 7.0


def test_sim_time_comes_from_px4_boot_clock():
    instance = mission.OffboardMission.__new__(mission.OffboardMission)
    instance.local_position = SimpleNamespace(time_boot_ms=12345)
    assert instance.sim_time() == 12.345


def test_phase_clock_selects_simulated_time(monkeypatch):
    instance = mission.OffboardMission.__new__(mission.OffboardMission)
    instance.args = SimpleNamespace(phase_clock='sim')
    instance.local_position = SimpleNamespace(time_boot_ms=2500)
    assert instance.phase_time() == 2.5

    instance.args.phase_clock = 'wall'
    monkeypatch.setattr(mission.time, 'monotonic', lambda: 42.0)
    assert instance.phase_time() == 42.0
