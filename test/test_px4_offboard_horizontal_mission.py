import importlib.util
from pathlib import Path

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
