import numpy as np
import pytest
from w2_sim.swerve_kinematics import (
    WHEEL_NAMES, WHEEL_POS, WHEEL_RADIUS,
    inverse_kinematics, forward_kinematics_ls, wrap_angle)

def test_constants():
    assert WHEEL_NAMES == ["front_left", "front_right", "rear_left", "rear_right"]
    assert WHEEL_POS.shape == (4, 2)
    assert WHEEL_RADIUS == pytest.approx(0.075)
    # front_left: +x +y
    assert WHEEL_POS[0, 0] > 0 and WHEEL_POS[0, 1] > 0

def test_pure_translation():
    ang, spd = inverse_kinematics(1.0, 0.0, 0.0)
    np.testing.assert_allclose(ang, 0.0, atol=1e-9)
    np.testing.assert_allclose(spd, 1.0, atol=1e-9)

def test_strafe():
    ang, spd = inverse_kinematics(0.0, 0.5, 0.0)
    np.testing.assert_allclose(ang, np.pi / 2, atol=1e-9)
    np.testing.assert_allclose(spd, 0.5, atol=1e-9)

def test_pure_rotation_speed():
    wz = 1.0
    ang, spd = inverse_kinematics(0.0, 0.0, wz)
    r = np.linalg.norm(WHEEL_POS, axis=1)
    np.testing.assert_allclose(np.abs(spd), wz * r, atol=1e-9)

def test_roundtrip_random():
    rng = np.random.default_rng(0)
    for _ in range(100):
        vx, vy, wz = rng.uniform(-1.5, 1.5, 3)
        ang, spd = inverse_kinematics(vx, vy, wz)
        sol = forward_kinematics_ls(ang, spd)
        np.testing.assert_allclose(sol, [vx, vy, wz], atol=1e-9)

def test_shortest_path_flip():
    # 上一帧朝前(0°),新指令朝后:应保持角度不动、轮速取负,而不是甩 180°
    ang, spd = inverse_kinematics(-1.0, 0.0, 0.0, prev_angles=np.zeros(4))
    np.testing.assert_allclose(ang, 0.0, atol=1e-9)
    np.testing.assert_allclose(spd, -1.0, atol=1e-9)
    # 翻转后的解 FK 必须还原同一 twist
    sol = forward_kinematics_ls(ang, spd)
    np.testing.assert_allclose(sol, [-1.0, 0.0, 0.0], atol=1e-9)

def test_zero_twist_keeps_prev_angles():
    prev = np.array([0.3, -0.2, 0.1, 0.0])
    ang, spd = inverse_kinematics(0.0, 0.0, 0.0, prev_angles=prev)
    np.testing.assert_allclose(ang, prev)
    np.testing.assert_allclose(spd, 0.0)

def test_wrap():
    assert wrap_angle(np.pi + 0.1) == pytest.approx(-np.pi + 0.1)
    assert wrap_angle(-np.pi - 0.1) == pytest.approx(np.pi - 0.1)
