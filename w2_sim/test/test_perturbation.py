import numpy as np
import pytest
from w2_sim.perturbation import (
    sample_params, corrupt_wheel, GyroCorruptor, theoretical_kappa,
    params_to_dict, vibration_std, DEFAULT_CFG)


def test_sampling_deterministic_and_in_range():
    p1, p2 = sample_params(42, DEFAULT_CFG), sample_params(42, DEFAULT_CFG)
    np.testing.assert_array_equal(p1.steer_bias_rad, p2.steer_bias_rad)
    assert np.all(np.abs(p1.steer_bias_rad) <= np.radians(DEFAULT_CFG["steer_bias_max_deg"]))
    assert p1.speed_scale.shape == (4,)
    p3 = sample_params(43, DEFAULT_CFG)
    assert not np.array_equal(p1.steer_bias_rad, p3.steer_bias_rad)


def test_corrupt_wheel_applies_bias_and_scale():
    rng = np.random.default_rng(0)
    cfg = dict(DEFAULT_CFG, steer_noise_std_deg=0.0, speed_noise_std=0.0)
    p0 = sample_params(1, cfg)
    ang, spd = np.zeros(4), np.ones(4)
    ca, cs = corrupt_wheel(ang, spd, p0, rng)
    np.testing.assert_allclose(ca, p0.steer_bias_rad)
    np.testing.assert_allclose(cs, p0.speed_scale)


def test_theoretical_kappa_zero_for_clean():
    cfg = dict(DEFAULT_CFG, steer_bias_max_deg=0.0, speed_scale_std=0.0)
    p = sample_params(7, cfg)
    assert theoretical_kappa(p) == pytest.approx(0.0, abs=1e-12)


def test_theoretical_kappa_magnitude():
    # 实车量级锚点:δ~0.25°、s~0.4% 时 |κ| 应在 1e-3..1e-1 rad/m 量级(实车 ≈0.036)
    kappas = [abs(theoretical_kappa(sample_params(s, DEFAULT_CFG))) for s in range(50)]
    assert max(kappas) < 0.2
    assert np.median(kappas) > 1e-4


def test_gyro_corruptor_stats():
    p = sample_params(3, DEFAULT_CFG)
    g = GyroCorruptor(p, np.random.default_rng(0))
    n, dt = 20000, 1.0 / 200
    out = np.array([g.step(dt, np.zeros(3)) for _ in range(n)])
    # 零输入下输出均值 ≈ bias(漂移慢),标准差 ≈ 白噪声 σ
    np.testing.assert_allclose(out.mean(axis=0), p.gyro_bias_init_rad, atol=5e-3)
    np.testing.assert_allclose(out.std(axis=0), p.gyro_noise_std_rad, rtol=0.3)


def test_vibration_std_pure_function():
    # speed=0 → 只有电子底
    assert vibration_std(0.05, 0.3, 0.0) == pytest.approx(0.05)
    # speed 大 → 振动主导,符合 sqrt(e² + (c·v)²)
    assert vibration_std(0.05, 0.3, 2.0) == pytest.approx(np.hypot(0.05, 0.6))
    # 单调递增
    s = [vibration_std(0.05, 0.3, v) for v in [0, 0.5, 1.0, 2.0]]
    assert all(b > a for a, b in zip(s, s[1:]))
    # 逐轴 ndarray 也支持
    out = vibration_std(np.array([0.05, 0.07, 0.06]), 0.3, 1.0)
    np.testing.assert_allclose(out, np.hypot([0.05, 0.07, 0.06], 0.3))


def test_gyro_corruptor_vibration_scales_with_speed():
    # speed=0 时 std ≈ 电子底;speed 大时 std 随速度按公式抬升
    p = sample_params(5, DEFAULT_CFG)
    n, dt = 40000, 1.0 / 200
    g0 = GyroCorruptor(p, np.random.default_rng(1))
    out0 = np.array([g0.step(dt, np.zeros(3), 0.0) for _ in range(n)])
    g1 = GyroCorruptor(p, np.random.default_rng(2))
    spd = 1.0
    out1 = np.array([g1.step(dt, np.zeros(3), spd) for _ in range(n)])
    # 去 bias 后逐轴 std
    s0 = out0.std(axis=0)
    s1 = out1.std(axis=0)
    np.testing.assert_allclose(s0, p.gyro_noise_std_rad, rtol=0.05)
    exp1 = np.hypot(p.gyro_noise_std_rad, p.gyro_vib_coeff_rad * spd)
    np.testing.assert_allclose(s1, exp1, rtol=0.05)
    assert np.all(s1 > s0 * 2)   # 行驶噪声显著大于静止


def test_accel_corruptor_vibration_scales_with_speed():
    # 显式给非零 accel 振动系数验证机制(标定后默认 accel 系数为 0,见 perturbation.py 注释)
    cfg = dict(DEFAULT_CFG, accel_vib_coeff_mps2_per_mps=0.5)
    p = sample_params(6, cfg)
    n = 40000
    g = GyroCorruptor(p, np.random.default_rng(3))
    out0 = np.array([g.step_accel(np.zeros(3), 0.0) for _ in range(n)])
    out1 = np.array([g.step_accel(np.zeros(3), 1.0) for _ in range(n)])
    np.testing.assert_allclose(out0.std(axis=0), p.accel_noise_std, rtol=0.05)
    exp1 = np.hypot(p.accel_noise_std, p.accel_vib_coeff * 1.0)
    np.testing.assert_allclose(out1.std(axis=0), exp1, rtol=0.05)
    assert p.accel_vib_coeff == pytest.approx(0.5)


def test_backward_compat_default_speed_zero():
    # 不传 speed → 退化为原电子底行为
    p = sample_params(7, DEFAULT_CFG)
    g = GyroCorruptor(p, np.random.default_rng(4))
    out = np.array([g.step(1.0 / 200, np.zeros(3)) for _ in range(20000)])
    np.testing.assert_allclose(out.std(axis=0), p.gyro_noise_std_rad, rtol=0.1)


def test_params_roundtrip_dict():
    p = sample_params(9, DEFAULT_CFG)
    d = params_to_dict(p)
    assert "kappa_theory" in d and d["seed"] == 9
    assert len(d["steer_bias_deg"]) == 4
