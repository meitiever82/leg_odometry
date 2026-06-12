import numpy as np
import pytest
from w2_sim.perturbation import (
    sample_params, corrupt_wheel, GyroCorruptor, theoretical_kappa,
    params_to_dict, DEFAULT_CFG)


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


def test_params_roundtrip_dict():
    p = sample_params(9, DEFAULT_CFG)
    d = params_to_dict(p)
    assert "kappa_theory" in d and d["seed"] == 9
    assert len(d["steer_bias_deg"]) == 4
