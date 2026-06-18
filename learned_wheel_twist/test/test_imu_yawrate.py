import sys, numpy as np
sys.path.insert(0, "learned_wheel_twist")
from lwt.imu_yawrate import (project_yawrate, debias_yawrate, static_window_mask,
                             from_two_vectors, base_frame_imu)


def test_base_zup_equals_gyro_z():
    # sim 情形:重力沿 +Z(加速度计报比力向上),up≈[0,0,1] → yaw_rate≈gyro_z。
    rng = np.random.default_rng(0)
    n = 500
    gyro = rng.normal(0, 0.5, (n, 3))
    accel = np.zeros((n, 3)) + np.array([0, 0, 9.81]) + rng.normal(0, 0.05, (n, 3))
    yr = project_yawrate(accel, gyro, sign=1.0)
    np.testing.assert_allclose(yr, gyro[:, 2], atol=0.05)


def test_sign_flip():
    rng = np.random.default_rng(1)
    n = 300
    gyro = rng.normal(0, 0.5, (n, 3))
    accel = np.zeros((n, 3)) + np.array([0, 0, 9.81])
    yp = project_yawrate(accel, gyro, sign=1.0)
    yn = project_yawrate(accel, gyro, sign=-1.0)
    np.testing.assert_allclose(yp, -yn, atol=1e-9)


def test_tilted_frame_projects_onto_gravity_axis():
    # 倾斜系:重力轴 = up;绕该轴的纯旋转应被完整投影出,垂直分量应被滤掉。
    rng = np.random.default_rng(2)
    up = np.array([0.1, -0.2, 0.97]); up = up / np.linalg.norm(up)
    n = 400
    omega = rng.normal(0, 0.5, n)                 # 绕重力轴的真 yaw-rate
    perp = np.cross(up, [1, 0, 0]); perp /= np.linalg.norm(perp)
    noise = rng.normal(0, 0.3, n)
    gyro = omega[:, None] * up[None, :] + noise[:, None] * perp[None, :]
    accel = np.zeros((n, 3)) + 9.81 * up
    yr = project_yawrate(accel, gyro, sign=1.0)
    np.testing.assert_allclose(yr, omega, atol=1e-6)


def test_gravity_down_needs_negative_sign():
    # 加速度计报「重力向下」(-up):几何 up 指向 -Z,up·gyro = -gyro_z,
    # 需 sign=-1 才还原 +CCW。验证 sign=-1 恢复 gyro_z。
    rng = np.random.default_rng(3)
    n = 300
    gyro = rng.normal(0, 0.5, (n, 3))
    accel = np.zeros((n, 3)) + np.array([0, 0, -9.81])
    yr = project_yawrate(accel, gyro, sign=-1.0)
    np.testing.assert_allclose(yr, gyro[:, 2], atol=1e-9)


# --- 迭代e:静止窗去陀螺零偏 ---

def test_debias_removes_constant_offset():
    # 启动 2s 静止(轮速≈0),其后运动;通道叠了常值零偏 b。去偏须恰好扣掉 b。
    rng = np.random.default_rng(4)
    n = 1000
    t = np.arange(n) * 0.02            # 50Hz → 前 2s = 前 100 帧
    speed = np.zeros((n, 4))
    speed[100:] = 0.5 + rng.normal(0, 0.01, (n - 100, 4))   # 100 帧后开动
    bias = 0.019
    true_yr = np.concatenate([np.zeros(100), rng.normal(0, 0.3, n - 100)])
    yr = true_yr + bias
    out, est_bias, found = debias_yawrate(yr, t, speed, static_sec=2.0, speed_eps=0.05)
    assert found
    np.testing.assert_allclose(est_bias, bias, atol=1e-9)   # 静止窗真值=0 → 均值=bias
    np.testing.assert_allclose(out, true_yr, atol=1e-9)


def test_debias_preserves_shape_2d():
    # (N,1) 输入须返回 (N,1)。
    t = np.arange(200) * 0.02
    speed = np.zeros((200, 4))
    yr = (np.arange(200, dtype=float) * 0.0 + 0.5)[:, None]
    out, b, found = debias_yawrate(yr, t, speed)
    assert out.shape == (200, 1)
    np.testing.assert_allclose(out, 0.0, atol=1e-12)   # 全静止 → 全程均值=0.5 扣掉


def test_static_window_guard_searches_when_start_moving():
    # 起始就在运动,中段才静止:守卫须找到中段静止窗(found=True),而非盲用前 2s。
    n = 1000
    t = np.arange(n) * 0.02
    speed = np.full((n, 4), 0.5)
    speed[300:500] = 0.0          # 6s..10s 静止(远 >2s,留余量给守卫滑窗)
    mask, found = static_window_mask(t, speed, static_sec=2.0, speed_eps=0.05)
    assert found
    idx = np.where(mask)[0]
    # 须落在中段静止段内、远离前 2s(起点应在静止段附近,而非 t0)。
    assert idx[0] >= 250 and idx[-1] < 500
    assert np.abs(speed[mask]).mean() < 0.05


def test_static_window_guard_falls_back_when_never_static():
    # 全程运动:找不到静止窗 → found=False,回退前 2s。
    n = 500
    t = np.arange(n) * 0.02
    speed = np.full((n, 4), 0.5)
    mask, found = static_window_mask(t, speed, static_sec=2.0, speed_eps=0.05)
    assert not found
    assert mask[:100].all() and not mask[100:].any()


# --- phase-3:base-frame 全向量(旋转对齐 + 去偏) ---

def test_from_two_vectors_maps_a_onto_b():
    rng = np.random.default_rng(10)
    for _ in range(20):
        a = rng.normal(0, 1, 3); b = rng.normal(0, 1, 3)
        R = from_two_vectors(a, b)
        np.testing.assert_allclose(R @ (a/np.linalg.norm(a)), b/np.linalg.norm(b), atol=1e-9)
        # 仍是旋转(正交、det=+1)
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-9)
        np.testing.assert_allclose(np.linalg.det(R), 1.0, atol=1e-9)


def test_from_two_vectors_identity_and_flip():
    a = np.array([0, 0, 1.0])
    np.testing.assert_allclose(from_two_vectors(a, a), np.eye(3), atol=1e-12)
    np.testing.assert_allclose(from_two_vectors(a, -a), -np.eye(3), atol=1e-12)


def test_base_frame_rotation_sends_gravity_to_zup():
    # 倾斜安装系:静止窗内加速度计报比力沿 +up(倾斜方向)。base_frame 旋转后
    # a_base 静止窗均值须 ≈ [0,0,+|g|]。
    rng = np.random.default_rng(11)
    up = np.array([-0.85, -0.0, -0.52]); up = up / np.linalg.norm(up)  # 真机量级倾斜
    n = 1000
    t = np.arange(n) * 0.02
    speed = np.zeros((n, 4)); speed[100:] = 0.6  # 前 2s 静止
    g = 9.81
    accel = np.tile(g * up, (n, 1)) + rng.normal(0, 0.02, (n, 3))
    gyro = rng.normal(0, 0.3, (n, 3))
    g_base, a_base, R, b_g, found = base_frame_imu(accel, gyro, t, speed)
    assert found
    static = t < t[0] + 2.0
    np.testing.assert_allclose(a_base[static].mean(0), [0, 0, g], atol=0.05)


def test_base_frame_removes_gyro_bias():
    # 启动静止窗内陀螺有常值零偏 b;去偏后静止窗 g_base 均值 ≈ 0(已扣 b_g,且 R 不放大零均值)。
    rng = np.random.default_rng(12)
    n = 1000
    t = np.arange(n) * 0.02
    speed = np.zeros((n, 4)); speed[100:] = 0.6
    accel = np.tile([0, 0, 9.81], (n, 1))  # base z-up,R≈I
    bias = np.array([0.01, -0.02, 0.019])
    gyro = rng.normal(0, 0.3, (n, 3)) + bias
    g_base, a_base, R, b_g, found = base_frame_imu(accel, gyro, t, speed)
    assert found
    np.testing.assert_allclose(b_g, bias, atol=0.05)
    static = t < t[0] + 2.0
    np.testing.assert_allclose(g_base[static].mean(0), [0, 0, 0], atol=0.05)


def test_base_frame_sim_near_identity():
    # sim 情形:base z-up、无零偏 → R≈I,b_g≈0,g_base≈gyro,a_base≈accel,
    # 且 g_base[:,2] ≈ project_yawrate(同号 +CCW)。
    rng = np.random.default_rng(13)
    n = 800
    t = np.arange(n) * 0.02
    speed = np.zeros((n, 4)); speed[100:] = 0.6
    accel = np.tile([0, 0, 9.81], (n, 1)) + rng.normal(0, 0.01, (n, 3))
    gyro = rng.normal(0, 0.4, (n, 3))
    g_base, a_base, R, b_g, found = base_frame_imu(accel, gyro, t, speed)
    np.testing.assert_allclose(R, np.eye(3), atol=0.01)
    np.testing.assert_allclose(b_g, 0.0, atol=0.05)
    # g_base[:,2] 应与重力投影 yaw-rate 高度一致
    yr = project_yawrate(accel, gyro, sign=1.0)
    assert np.corrcoef(g_base[:, 2], yr)[0, 1] > 0.99
