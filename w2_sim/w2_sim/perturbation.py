"""w2_sim 扰动参数采样/施加。design §6。
每 episode 采样一次、全程固定(模拟"本次开机的标定状态")。
"""
from dataclasses import dataclass
import numpy as np
from w2_sim.swerve_kinematics import inverse_kinematics, forward_kinematics_ls

DEFAULT_CFG = {
    "steer_bias_max_deg": 0.5,     # δ_i ~ U(-max, +max),每轮独立
    "speed_scale_std": 0.005,      # s_i ~ N(1, std),每轮独立
    "steer_noise_std_deg": 0.05,   # 测量白噪声
    "speed_noise_std": 0.01,       # m/s
    "gyro_bias_init_max_dps": 0.5,         # 初始 bias ~ U(-max, +max) °/s
    "gyro_rw_std": 5e-5,                   # bias 随机游走 rad/s/√s(~10°/h 不稳定性量级)
    "gyro_noise_density_dps_sqrthz": 0.01, # 噪声密度 (°/s)/√Hz
    "accel_noise_density_mg_sqrthz": 0.1,  # mg/√Hz
    "imu_rate_hz": 200.0,
}


@dataclass
class PerturbationParams:
    seed: int
    steer_bias_rad: np.ndarray     # (4,)
    speed_scale: np.ndarray        # (4,)
    steer_noise_std_rad: float
    speed_noise_std: float
    gyro_bias_init_rad: np.ndarray  # (3,) rad/s
    gyro_rw_std: float
    gyro_noise_std_rad: float      # 离散 σ = density*√rate
    accel_noise_std: float         # m/s²


def sample_params(seed, cfg=DEFAULT_CFG):
    """采样一组固定扰动参数(per-episode 调用一次)。"""
    rng = np.random.default_rng(seed)
    rate = cfg["imu_rate_hz"]
    return PerturbationParams(
        seed=seed,
        steer_bias_rad=rng.uniform(-1, 1, 4) * np.radians(cfg["steer_bias_max_deg"]),
        speed_scale=rng.normal(1.0, cfg["speed_scale_std"], 4),
        steer_noise_std_rad=np.radians(cfg["steer_noise_std_deg"]),
        speed_noise_std=cfg["speed_noise_std"],
        gyro_bias_init_rad=np.radians(rng.uniform(-1, 1, 3) * cfg["gyro_bias_init_max_dps"]),
        gyro_rw_std=cfg["gyro_rw_std"],
        gyro_noise_std_rad=np.radians(cfg["gyro_noise_density_dps_sqrthz"]) * np.sqrt(rate),
        accel_noise_std=cfg["accel_noise_density_mg_sqrthz"] * 1e-3 * 9.81 * np.sqrt(rate),
    )


def corrupt_wheel(angles, speeds, p, rng):
    """真值 (转向角, 轮速) -> 退化测量值。"""
    ca = angles + p.steer_bias_rad + rng.normal(0, p.steer_noise_std_rad, 4)
    cs = speeds * p.speed_scale + rng.normal(0, p.speed_noise_std, 4)
    return ca, cs


class GyroCorruptor:
    """gyro bias 初值 + 随机游走 + 白噪声;accel 白噪声。逐样本有状态。"""

    def __init__(self, p, rng):
        self.p, self.rng = p, rng
        self.bias = p.gyro_bias_init_rad.copy()

    def step(self, dt, gyro):
        self.bias = self.bias + self.rng.normal(0, self.p.gyro_rw_std * np.sqrt(dt), 3)
        return gyro + self.bias + self.rng.normal(0, self.p.gyro_noise_std_rad, 3)

    def step_accel(self, accel):
        return accel + self.rng.normal(0, self.p.accel_noise_std, 3)


def theoretical_kappa(p):
    """理论等效 κ (rad/m):直行 1 m/s 真值经确定性退化(无白噪声)后 8×3 LS 解出的 ω_z。
    与实车 κ 定义一致(ω_z = κ·v_x)。验收第 3 条用它对照数据回归值。"""
    ang, spd = inverse_kinematics(1.0, 0.0, 0.0)
    ca = ang + p.steer_bias_rad
    cs = spd * p.speed_scale
    vx, vy, wz = forward_kinematics_ls(ca, cs)
    return float(wz / vx)


def params_to_dict(p):
    """序列化为可 JSON/YAML 存储的字典,包含理论 κ。"""
    return {
        "seed": int(p.seed),
        "steer_bias_deg": np.degrees(p.steer_bias_rad).tolist(),
        "speed_scale": p.speed_scale.tolist(),
        "steer_noise_std_deg": float(np.degrees(p.steer_noise_std_rad)),
        "speed_noise_std": float(p.speed_noise_std),
        "gyro_bias_init_dps": np.degrees(p.gyro_bias_init_rad).tolist(),
        "gyro_rw_std": float(p.gyro_rw_std),
        "gyro_noise_std_dps": float(np.degrees(p.gyro_noise_std_rad)),
        "accel_noise_std": float(p.accel_noise_std),
        "kappa_theory": theoretical_kappa(p),
    }
