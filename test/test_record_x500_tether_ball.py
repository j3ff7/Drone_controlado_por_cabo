"""Geometria do X500 + tether por BallJoint: composicao de poses e angulos do primeiro segmento."""
import importlib.util
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('record_x500_tether_ball', ROOT / 'tools' / 'record_x500_tether_ball.py')
rec = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rec)

I = rec.IDENTITY


def axis_angle(axis, degrees):
    h = math.radians(degrees) / 2.0
    return (axis[0] * math.sin(h), axis[1] * math.sin(h), axis[2] * math.sin(h), math.cos(h))


def close(a, b, tol=1e-9):
    return all(math.isclose(x, y, abs_tol=tol) for x, y in zip(a, b))


def test_segment_along_drone_forward_is_azimuth_zero_elevation_zero():
    _, az, el = rec.segment_angles(I, I)
    assert math.isclose(az, 0.0, abs_tol=1e-9) and math.isclose(el, 0.0, abs_tol=1e-9)


def test_segment_hanging_down_is_elevation_minus_90():
    t, _, el = rec.segment_angles(I, axis_angle((0, 1, 0), 90.0))    # +x vira -z
    assert close(t, (0.0, 0.0, -1.0))
    assert math.isclose(el, -90.0, abs_tol=1e-6)


def test_angles_are_expressed_in_the_drone_frame():
    # drone girado +90 graus em yaw; segmento ao longo de +x do mundo fica a direita do drone
    t, az, _ = rec.segment_angles(axis_angle((0, 0, 1), 90.0), I)
    assert close(t, (0.0, -1.0, 0.0))
    assert math.isclose(az, -90.0, abs_tol=1e-6)


def test_nested_cable_pose_is_composed_through_both_parents():
    vehicle = ((1.0, 0.0, 0.227), axis_angle((0, 0, 1), 90.0))
    nested = rec.compose(vehicle, ((0.0, 0.0, 0.0), I))
    root = rec.compose(nested, ((0.1, 0.0, 0.2), I))            # 0,1 m a frente do veiculo
    assert close(root[0], (1.0, 0.1, 0.427))


def test_sample_reports_ball_joint_offset_and_angles():
    poses = {
        'x500_tether_ball_0': ((0.0, 0.0, 0.0), I),
        'base_link': ((0.0, 0.0, 0.227), I),
        'cabo_anexado': ((0.0, 0.0, 0.427), I),
        'raiz_cabo': ((0.0, 0.0, 0.0), I),
        'segment_1': ((0.0, 0.0, 0.0), axis_angle((0, 1, 0), 45.0)),
        'segment_2': ((0.035, 0.0, -0.035), I),
        'ponta_cabo': ((3.0, 0.0, -0.42), I),
    }
    row = rec.sample_from_poses(poses, 'x500_tether_ball_0', 'cabo_anexado', 1.0, 0.0)
    assert math.isclose(row['dist_root_base_m'], 0.2, abs_tol=1e-9)
    assert math.isclose(row['elevation_deg'], -45.0, abs_tol=1e-6)
    assert math.isclose(row['azimuth_deg'], 0.0, abs_tol=1e-6)
    assert row['segments'] == 2


def test_parse_poses_with_omitted_zero_fields():
    text = ('header {\n}\npose {\n  name: "raiz_cabo"\n  id: 3\n  position {\n    z: 0.2\n  }\n'
            '  orientation {\n    w: 1\n  }\n}\n')
    assert rec.parse_poses(text)['raiz_cabo'] == ((0.0, 0.0, 0.2), I)
