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
    "gyro_noise_density_dps_sqrthz": 0.01, # 噪声密度 (°/s)/√Hz;静止窗实测仿真噪声底 0.142°/s ≈ 真实 AIRY 0.115°/s(已标定)
    "accel_noise_density_mg_sqrthz": 0.1,  # mg/√Hz;静止窗实测仿真底 0.073 ≈ 真实 AIRY 0.056 m/s²(已标定)
    # 运动相关机械振动项(电机/轮地/底盘),叠加在电子噪声底之上,std = coeff × |车速 v|。
    # 总噪声 std = sqrt(electronic² + (coeff·|v|)²):静止(v=0)退化为电子底,行驶时按斜率抬升。
    #
    # 标定自真实 bag rosbag2_2026_05_22-13_58_40 的 noise-vs-speed 分桶斜率
    # (scripts/analyze_imu_vib.py,相邻差分法):真机 gyro 噪声-速度斜率 ≈0.82°/s per m/s、
    # accel ≈0.67 (m/s²)/(m/s)。但仿真物理(/sim/imu 干净流)本身已随速度产生振动:
    # 实测干净 gyro 斜率 ≈0.58、accel 斜率 ≈1.42 —— accel 已远超真机,陀螺也贡献过半。
    # 所以叠加量取 sqrt(real² − sim_clean²):
    #   gyro_vib_coeff = sqrt(0.82² − 0.58²) ≈ 0.58°/s per m/s(补足真机差额)。
    #   accel_vib_coeff = 0:仿真物理 accel 振动已 >真机,再加只会更高(物理底问题,非本层可降)。
    "gyro_vib_coeff_dps_per_mps": 0.58,    # (°/s) per (m/s) 车速;转成 rad/s 在 sample_params 里
    "accel_vib_coeff_mps2_per_mps": 0.0,   # 仿真物理 accel 已过振动,不再叠加(见上)
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
    gyro_noise_std_rad: float      # 离散 σ = density*√rate(电子底)
    accel_noise_std: float         # m/s²(电子底)
    gyro_vib_coeff_rad: float      # 振动 std 斜率 (rad/s) per (m/s 车速)
    accel_vib_coeff: float         # 振动 std 斜率 (m/s²) per (m/s 车速)


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
        gyro_vib_coeff_rad=np.radians(cfg["gyro_vib_coeff_dps_per_mps"]),
        accel_vib_coeff=cfg["accel_vib_coeff_mps2_per_mps"],
    )


def vibration_std(electronic_std, vib_coeff, speed):
    """运动相关振动叠加后的总噪声 std(纯函数,可测)。
    总 std = sqrt(electronic² + (vib_coeff·|speed|)²)。
    speed=0 → 退化为电子底;speed 增大 → 按 vib_coeff 斜率抬升。
    标量或逐轴 ndarray 均可。"""
    vib = vib_coeff * abs(float(speed))
    return np.sqrt(np.asarray(electronic_std) ** 2 + vib ** 2)


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

    def step(self, dt, gyro, speed=0.0):
        """加 bias + 随机游走 + (电子白噪声 ⊕ 运动振动)。
        speed=车速 |v| m/s,默认 0 → 退化为纯电子底(向后兼容)。"""
        self.bias = self.bias + self.rng.normal(0, self.p.gyro_rw_std * np.sqrt(dt), 3)
        std = vibration_std(self.p.gyro_noise_std_rad, self.p.gyro_vib_coeff_rad, speed)
        return gyro + self.bias + self.rng.normal(0, std, 3)

    def step_accel(self, accel, speed=0.0):
        """加 (电子白噪声 ⊕ 运动振动)。speed 默认 0 → 纯电子底。"""
        std = vibration_std(self.p.accel_noise_std, self.p.accel_vib_coeff, speed)
        return accel + self.rng.normal(0, std, 3)


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
        "gyro_vib_coeff_dps_per_mps": float(np.degrees(p.gyro_vib_coeff_rad)),
        "accel_vib_coeff_mps2_per_mps": float(p.accel_vib_coeff),
        "kappa_theory": theoretical_kappa(p),
    }
