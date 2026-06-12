import numpy as np
from w2_sim.trajectory_generator import generate_profile, LIMITS

def _profile(seed=0, dur=180.0):
    return generate_profile(seed, dur, rate_hz=50.0)

def test_shape_and_time():
    p = _profile()
    assert p.shape[1] == 4            # t, vx, vy, wz
    assert abs(p[-1, 0] - 180.0) < 0.1
    np.testing.assert_allclose(np.diff(p[:, 0]), 0.02, atol=1e-9)

def test_still_bookends():
    p = _profile()
    head = p[p[:, 0] < 5.0]           # 前 5s 必静止
    tail = p[p[:, 0] > p[-1, 0] - 3.0]  # 末 3s 必静止
    assert np.all(np.abs(head[:, 1:]) < 1e-9)
    assert np.all(np.abs(tail[:, 1:]) < 1e-9)

def test_limits():
    p = _profile(seed=3)
    assert np.all(np.hypot(p[:, 1], p[:, 2]) <= LIMITS["v_max"] + 1e-6)
    assert np.all(np.abs(p[:, 3]) <= LIMITS["w_max"] + 1e-6)
    acc = np.abs(np.diff(p[:, 1:3], axis=0)) * 50.0
    assert np.all(acc <= LIMITS["a_max"] + 1e-6)

def test_deterministic():
    np.testing.assert_array_equal(_profile(5), _profile(5))
    assert not np.array_equal(_profile(5), _profile(6))

def test_coverage_180s():
    # 3 分钟内应同时出现:前进、侧移(swerve 特有)、旋转,且有中途静止段
    p = _profile(seed=1)
    mid = p[(p[:, 0] > 10) & (p[:, 0] < p[-1, 0] - 5)]
    assert np.max(np.abs(mid[:, 1])) > 0.5          # 前进
    assert np.max(np.abs(mid[:, 2])) > 0.2          # 侧移
    assert np.max(np.abs(mid[:, 3])) > 0.3          # 旋转
    still = np.all(np.abs(mid[:, 1:]) < 1e-9, axis=1)
    assert still.sum() * 0.02 > 3.0                 # 中途静止累计 >3s
