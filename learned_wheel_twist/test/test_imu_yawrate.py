import sys, numpy as np
sys.path.insert(0, "learned_wheel_twist")
from lwt.imu_yawrate import project_yawrate


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
