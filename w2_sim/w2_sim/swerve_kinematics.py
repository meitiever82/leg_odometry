"""w2 swerve 底盘运动学。坐标系:base_link,x 前 y 左,wz 逆时针为正。
轮序与实车 /robot/wheel_status 一致。只依赖 numpy(Isaac 自带 python 也能 import)。"""
import numpy as np

WHEEL_NAMES = ["front_left", "front_right", "rear_left", "rear_right"]
WHEELBASE = 0.435   # m, 前后轮心距
TRACK = 0.400       # m, 左右轮心距
WHEEL_RADIUS = 0.075  # m
WHEEL_POS = np.array([
    [ WHEELBASE / 2,  TRACK / 2],   # front_left
    [ WHEELBASE / 2, -TRACK / 2],   # front_right
    [-WHEELBASE / 2,  TRACK / 2],   # rear_left
    [-WHEELBASE / 2, -TRACK / 2],   # rear_right
])

def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

def inverse_kinematics(vx, vy, wz, prev_angles=None):
    """底盘 twist -> (转向角[4] rad, 带符号轮速[4] m/s)。
    给 prev_angles 时取 ±180° 中靠近上一帧的解(轮速变号),静止轮保持上一帧角度。"""
    v = np.stack([vx - wz * WHEEL_POS[:, 1], vy + wz * WHEEL_POS[:, 0]], axis=1)
    speeds = np.linalg.norm(v, axis=1)
    moving = speeds > 1e-9
    angles = np.zeros(4) if prev_angles is None else np.array(prev_angles, dtype=float).copy()
    angles[moving] = np.arctan2(v[moving, 1], v[moving, 0])
    signed = speeds.copy()
    if prev_angles is not None:
        for i in range(4):
            if not moving[i]:
                continue
            if abs(wrap_angle(angles[i] - prev_angles[i])) > np.pi / 2:
                angles[i] = wrap_angle(angles[i] + np.pi)
                signed[i] = -speeds[i]
    return angles, signed

def forward_kinematics_ls(angles, speeds):
    """8×3 最小二乘:4×(转向角 rad, 带符号轮速 m/s) -> (vx, vy, wz)。
    与实车 wheel_odometry 同一公式。"""
    A = np.zeros((8, 3))
    b = np.zeros(8)
    for i, (x, y) in enumerate(WHEEL_POS):
        A[2 * i] = [1.0, 0.0, -y]
        b[2 * i] = speeds[i] * np.cos(angles[i])
        A[2 * i + 1] = [0.0, 1.0, x]
        b[2 * i + 1] = speeds[i] * np.sin(angles[i])
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    return sol
