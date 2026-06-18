# test/test_wheel_noise.py —— 阶段六 Step A:仅轮速通道高斯噪声(IMU 不动)。
import sys, numpy as np
sys.path.insert(0, "learned_wheel_twist")
from lwt.realism import apply_wheel_noise


def test_wheel_noise_perturbs_only_wheel_channels():
    # 14ch 窗:[0:4]steer [4:8]speed [8:14]IMU。噪声只动 0-7,IMU 通道逐元素不变。
    W = np.zeros((30, 14), dtype=np.float32)
    W[:, 8:14] = 1.234   # IMU 通道任意非零值
    W0 = W.copy()
    rng = np.random.default_rng(0)
    out = apply_wheel_noise(W, enable=True, speed_std=0.05, steer_std_deg=1.0, rng=rng)
    # 轮速/转向被扰动
    assert np.any(out[:, 0:4] != W0[:, 0:4])
    assert np.any(out[:, 4:8] != W0[:, 4:8])
    # IMU 通道逐元素严格不变
    np.testing.assert_array_equal(out[:, 8:14], W0[:, 8:14])


def test_wheel_noise_disabled_is_noop():
    W = np.random.randn(20, 14).astype(np.float32)
    W0 = W.copy()
    out = apply_wheel_noise(W, enable=False)
    np.testing.assert_array_equal(out, W0)


def test_wheel_noise_amplitude_matches_std():
    # 大样本下,扰动量级应 ≈ 配置 std(speed 直接 m/s,steer 转 rad)。
    W = np.zeros((20000, 14), dtype=np.float64)
    rng = np.random.default_rng(1)
    out = apply_wheel_noise(W, enable=True, speed_std=0.02, steer_std_deg=0.5, rng=rng)
    assert abs(out[:, 4:8].std() - 0.02) < 0.002
    assert abs(out[:, 0:4].std() - np.radians(0.5)) < 0.002
