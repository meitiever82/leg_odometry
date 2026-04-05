"""Bloesch EKF 腿式里程计核心实现。

Error-state EKF，状态向量 24 维:
  [0:3]   p       - 机身位置 (世界系)
  [3:6]   v       - 机身速度 (世界系)
  [6:9]   theta   - 姿态误差 (error-state)
  [9:12]  b_a     - 加速度计偏置
  [12:15] b_g     - 陀螺仪偏置
  [15:18] p_fl    - 左脚位置 (世界系)
  [18:21] p_fr    - 右脚位置 (世界系)

名义状态额外维护:
  R       - 旋转矩阵 SO(3), 3x3

参考: Bloesch et al. "State Estimation for Legged Robots", RSS 2013.
"""

import numpy as np
from scipy.spatial.transform import Rotation


def skew(v: np.ndarray) -> np.ndarray:
    """向量的反对称矩阵 (叉积矩阵)。"""
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0],
    ])


def exp_so3(phi: np.ndarray) -> np.ndarray:
    """SO(3) 指数映射: 轴角 -> 旋转矩阵。"""
    if not np.all(np.isfinite(phi)):
        return np.eye(3)
    angle = np.linalg.norm(phi)
    if angle < 1e-10:
        return np.eye(3) + skew(phi)
    axis = phi / angle
    K = skew(axis)
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K


class BloeSchEKF:
    """Bloesch error-state EKF for legged robot odometry."""

    # 状态索引
    P = slice(0, 3)     # position
    V = slice(3, 6)     # velocity
    TH = slice(6, 9)    # theta (rotation error)
    BA = slice(9, 12)   # accel bias
    BG = slice(12, 15)  # gyro bias
    FL = slice(15, 18)  # left foot position
    FR = slice(18, 21)  # right foot position
    DIM = 21            # error-state 维度 (theta 用 3 维)

    def __init__(self, params: dict):
        self.g = np.array(params.get('gravity', [0, 0, -9.81]))

        # IMU 噪声
        self.sigma_a = params.get('accel_noise', 0.1)
        self.sigma_g = params.get('gyro_noise', 0.01)
        self.sigma_ba = params.get('accel_bias_walk', 0.001)
        self.sigma_bg = params.get('gyro_bias_walk', 0.0001)

        # 足端噪声
        self.sigma_contact = params.get('foot_contact_noise', 0.01)
        self.sigma_swing = params.get('foot_swing_noise', 1.0)

        # 观测噪声
        self.sigma_fk = params.get('fk_position_noise', 0.01)
        self.sigma_zupt = params.get('zupt_noise', 0.1)  # m/s, ZUPT 速度约束噪声

        # 平面运动约束
        self.sigma_flat_z = params.get('flat_z_noise', 0.0)  # m, Z 位置约束噪声 (0=关闭)
        self.sigma_flat_vz = params.get('flat_vz_noise', 0.0)  # m/s, Z 速度约束噪声 (0=关闭)

        # 名义状态
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.R = np.eye(3)
        self.b_a = np.zeros(3)
        self.b_g = np.zeros(3)
        self.p_fl = np.zeros(3)
        self.p_fr = np.zeros(3)

        # ZUPT: 缓存上一帧 FK、角速度和实�� dt
        self._prev_fk_left = None
        self._prev_fk_right = None
        self._last_gyro = np.zeros(3)
        self._last_dt = 0.005

        # 协方差
        self.P_cov = np.eye(self.DIM) * 0.01
        self._set_init_covariance(params)

        self.initialized = False

    def _set_init_covariance(self, params):
        entries = [
            (self.P, params.get('init_pos_std', 0.01)),
            (self.V, params.get('init_vel_std', 0.1)),
            (self.TH, params.get('init_rot_std', 0.01)),
            (self.BA, params.get('init_accel_bias_std', 0.1)),
            (self.BG, params.get('init_gyro_bias_std', 0.01)),
            (self.FL, params.get('init_foot_std', 0.1)),
            (self.FR, params.get('init_foot_std', 0.1)),
        ]
        for sl, std in entries:
            self.P_cov[sl, sl] = np.eye(3) * std**2

    def initialize(self, accel: np.ndarray, p_fl_body: np.ndarray,
                   p_fr_body: np.ndarray):
        """用第一帧 IMU 和 FK 初始化。

        用加速度估计初始姿态（对齐重力），假设初始位置为原点。

        Args:
            accel: 加速度计读数 (body 系)
            p_fl_body: 左脚在 body 系中的位置 (FK)
            p_fr_body: 右脚在 body 系中的位置 (FK)
        """
        # 用加速度方向估计初始姿态 (假设静止, accel ≈ -g_body)
        g_body = -accel / np.linalg.norm(accel) * 9.81
        # 找旋转使 R @ g_body = g_world = [0, 0, -9.81]
        g_world = np.array([0, 0, -9.81])
        self.R = self._rotation_from_gravity(g_body, g_world)

        self.p = np.zeros(3)
        self.v = np.zeros(3)

        # 初始偏置: 静止时 R@(accel-b_a)+g=0 => b_a = accel - R^T@(-g)
        self.b_a = accel - self.R.T @ (-self.g)
        self.b_g = np.zeros(3)  # 陀螺偏置初始为零（静止时角速度应为零）

        # 初始化脚位置 (世界系)
        self.p_fl = self.p + self.R @ p_fl_body
        self.p_fr = self.p + self.R @ p_fr_body

        self.P_cov = np.eye(self.DIM) * 0.01
        self._set_init_covariance({})
        self.initialized = True

    @staticmethod
    def _rotation_from_gravity(g_body: np.ndarray, g_world: np.ndarray) -> np.ndarray:
        """计算使 R @ g_body = g_world 的旋转矩阵。"""
        g_b = g_body / np.linalg.norm(g_body)
        g_w = g_world / np.linalg.norm(g_world)

        v = np.cross(g_b, g_w)
        c = np.dot(g_b, g_w)

        if np.linalg.norm(v) < 1e-10:
            return np.eye(3) if c > 0 else -np.eye(3)

        V = skew(v)
        R = np.eye(3) + V + V @ V / (1 + c)
        return R

    def predict(self, accel: np.ndarray, gyro: np.ndarray, dt: float,
                contact_left: bool, contact_right: bool):
        """EKF 预测步骤 (IMU 驱动)。

        Args:
            accel: 加速度计读数 (body 系, m/s^2)
            gyro: 陀螺仪读数 (body 系, rad/s)
            dt: 时间步 (s)
            contact_left: 左脚是否着地
            contact_right: 右脚是否着地
        """
        if not self.initialized:
            return

        # 去偏置
        a = accel - self.b_a
        w = gyro - self.b_g
        self._last_gyro = w.copy()
        self._last_dt = dt

        # 名义状态更新
        a_world = self.R @ a + self.g
        self.p = self.p + self.v * dt + 0.5 * a_world * dt**2
        self.v = self.v + a_world * dt
        self.R = self.R @ exp_so3(w * dt)
        # b_a, b_g 不变
        # p_fl, p_fr 不变

        # 状态转移矩阵 F (error-state)
        F = np.eye(self.DIM)
        F[self.P, self.V] = np.eye(3) * dt
        F[self.P, self.TH] = -0.5 * self.R @ skew(a) * dt**2
        F[self.P, self.BA] = -0.5 * self.R * dt**2
        F[self.V, self.TH] = -self.R @ skew(a) * dt
        F[self.V, self.BA] = -self.R * dt
        F[self.TH, self.TH] = exp_so3(-w * dt)
        F[self.TH, self.BG] = -np.eye(3) * dt

        # 过程噪声 Q
        Q = np.zeros((self.DIM, self.DIM))
        Q[self.P, self.P] = np.eye(3) * (self.sigma_a * dt)**2
        Q[self.V, self.V] = np.eye(3) * (self.sigma_a * dt)**2
        Q[self.TH, self.TH] = np.eye(3) * (self.sigma_g * dt)**2
        Q[self.BA, self.BA] = np.eye(3) * (self.sigma_ba * np.sqrt(dt))**2
        Q[self.BG, self.BG] = np.eye(3) * (self.sigma_bg * np.sqrt(dt))**2

        # 脚位置噪声：取决于接触状态
        sigma_fl = self.sigma_contact if contact_left else self.sigma_swing
        sigma_fr = self.sigma_contact if contact_right else self.sigma_swing
        Q[self.FL, self.FL] = np.eye(3) * (sigma_fl * np.sqrt(dt))**2
        Q[self.FR, self.FR] = np.eye(3) * (sigma_fr * np.sqrt(dt))**2

        # 协方差传播
        self.P_cov = F @ self.P_cov @ F.T + Q
        self.P_cov = 0.5 * (self.P_cov + self.P_cov.T)  # 对称化

        # 锁定偏置协方差
        if self.sigma_ba == 0:
            self.P_cov[self.BA, :] = 0; self.P_cov[:, self.BA] = 0
        if self.sigma_bg == 0:
            self.P_cov[self.BG, :] = 0; self.P_cov[:, self.BG] = 0

    def update(self, p_fl_body: np.ndarray, p_fr_body: np.ndarray,
               contact_left: bool = True, contact_right: bool = True):
        """EKF 更新步骤: 运动学位置观测 + 接触脚 ZUPT 速度约束。

        位置观测: h_pos = p + R·FK(q) - p_foot = 0
        ZUPT:     h_vel = v + [ω]×(R·FK) ≈ 0  (接触脚世界速度为零)

        简化 ZUPT: 忽略关节速度 R·J·q̇ 和角速度 [ω]×(R·FK) 项,
        直接约束 d(p_foot)/dt ≈ 0 ⟹ v ≈ R·J·q̇ - [ω]×(R·FK)
        近似为 v_foot = v + [ω]×(R·FK) ≈ 0

        Args:
            p_fl_body: 左脚在 body 系中的位置 (FK)
            p_fr_body: 右脚在 body 系中的位置 (FK)
            contact_left: 左脚是否着地
            contact_right: 右脚是否着地
        """
        if not self.initialized:
            return

        R_pos = np.eye(3) * self.sigma_fk**2
        R_zupt = np.eye(3) * self.sigma_zupt**2

        # === 左脚 FK 位置观测（始终更新）===
        r_fl_world = self.R @ p_fl_body
        z_fl = self.p + r_fl_world
        res_fl = -(z_fl - self.p_fl)

        H_fl = np.zeros((3, self.DIM))
        H_fl[:, self.P] = np.eye(3)
        H_fl[:, self.TH] = -skew(r_fl_world)
        H_fl[:, self.FL] = -np.eye(3)
        self._kalman_update(res_fl, H_fl, R_pos)

        # 左脚 ZUPT（仅接触时）
        if contact_left and self._prev_fk_left is not None:
            self._zupt_update(p_fl_body, self._prev_fk_left, R_zupt)
        self._prev_fk_left = p_fl_body.copy()

        # === 右脚 FK 位置观测（始终更新）===
        r_fr_world = self.R @ p_fr_body
        z_fr = self.p + r_fr_world
        res_fr = -(z_fr - self.p_fr)

        H_fr = np.zeros((3, self.DIM))
        H_fr[:, self.P] = np.eye(3)
        H_fr[:, self.TH] = -skew(r_fr_world)
        H_fr[:, self.FR] = -np.eye(3)
        self._kalman_update(res_fr, H_fr, R_pos)

        # 右脚 ZUPT（仅接触时）
        if contact_right and self._prev_fk_right is not None:
            self._zupt_update(p_fr_body, self._prev_fk_right, R_zupt)
        self._prev_fk_right = p_fr_body.copy()

        # === 平面运动约束: Z 速度 ≈ 0, Z 位置 ≈ 0 ===
        if self.sigma_flat_vz > 0:
            H_vz = np.zeros((1, self.DIM))
            H_vz[0, self.V.start + 2] = 1.0
            self._kalman_update(
                np.array([-self.v[2]]),
                H_vz,
                np.array([[self.sigma_flat_vz**2]]))

        if self.sigma_flat_z > 0:
            H_pz = np.zeros((1, self.DIM))
            H_pz[0, self.P.start + 2] = 1.0
            self._kalman_update(
                np.array([-self.p[2]]),
                H_pz,
                np.array([[self.sigma_flat_z**2]]))

    def _zupt_update(self, fk_now: np.ndarray, fk_prev: np.ndarray,
                     R_zupt: np.ndarray):
        """零速度更新: 接触脚世界速度为零 → 约束 body 速度。

        接触时足端固定: d/dt(p + R·FK) = 0
        展开: v + [ω_w]×(R·FK) + R·dFK/dt = 0
        因此: v = -[ω_w]×(R·FK) - R·dFK/dt
        """
        dt = self._last_dt
        if dt <= 1e-6 or dt > 0.1:
            return

        # R·dFK/dt (有限差分，用固定 dt 避免浮点精度问题)
        dt_fk = 0.005  # FK 采样间隔
        R_dfk = self.R @ ((fk_now - fk_prev) / dt_fk)

        # [ω_world]×(R·FK)
        r_world = self.R @ fk_now
        omega_world = self.R @ self._last_gyro
        omega_cross = np.cross(omega_world, r_world)

        # 预期 body 速度
        v_expected = -(omega_cross + R_dfk)
        residual = v_expected - self.v

        H = np.zeros((3, self.DIM))
        H[:, self.V] = np.eye(3)

        self._kalman_update(residual, H, R_zupt)

    def _kalman_update(self, residual: np.ndarray, H: np.ndarray,
                       R: np.ndarray):
        """执行一次卡尔曼更新并将 error-state 注入名义状态。"""
        # 跳过包含 NaN 的情况
        if not np.all(np.isfinite(residual)):
            return

        S = H @ self.P_cov @ H.T + R

        try:
            K = np.linalg.solve(S.T, (self.P_cov @ H.T).T).T
        except np.linalg.LinAlgError:
            return

        dx = K @ residual

        if not np.all(np.isfinite(dx)):
            return

        # 限制单次更新幅度，防止发散
        dx_pos = dx[self.P]
        if np.linalg.norm(dx_pos) > 1.0:
            dx *= 1.0 / np.linalg.norm(dx_pos)

        # 锁定偏置：如果偏置游走噪声为 0，不更新偏置
        if self.sigma_ba == 0:
            dx[self.BA] = 0.0
        if self.sigma_bg == 0:
            dx[self.BG] = 0.0

        # 注入 error-state 到名义状态
        self.p += dx[self.P]
        self.v += dx[self.V]
        self.R = self.R @ exp_so3(dx[self.TH])
        self.b_a += dx[self.BA]
        self.b_g += dx[self.BG]
        self.p_fl += dx[self.FL]
        self.p_fr += dx[self.FR]

        # 协方差更新 (Joseph form for numerical stability)
        I_KH = np.eye(self.DIM) - K @ H
        self.P_cov = I_KH @ self.P_cov @ I_KH.T + K @ R @ K.T

        # 对称化
        self.P_cov = 0.5 * (self.P_cov + self.P_cov.T)

    def get_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """返回当前位姿估计。

        Returns:
            (position [3], quaternion [4] as [x, y, z, w])
        """
        rot = Rotation.from_matrix(self.R)
        quat = rot.as_quat()  # [x, y, z, w]
        return self.p.copy(), quat

    def get_velocity(self) -> np.ndarray:
        """返回当前速度估计 (世界系)。"""
        return self.v.copy()
