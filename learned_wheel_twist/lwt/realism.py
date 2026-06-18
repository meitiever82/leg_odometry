# lwt/realism.py —— approach-2 step2a:把真机域退化注入 SIM 训练输入(缓存数据增广,无需 Isaac 重仿)。
#
# 动机(假设):SIM 陀螺通道 imu_g_base[:,2] ≈ gt_wz **完美无噪**(实测 std 8e-4),模型学到
# "过度信任完美陀螺";真机去偏陀螺有噪声+小残差偏置 → 模型 wz 修正泛化出常值偏移(per-frame DD
# 真机 wz DC −0.006 rad/s)。修复 = 让 SIM 覆盖真机输入分布:
#   1) κ-bulk:把真机 |κ|≈0.04 放到 effective-κ 分布的 **中位**(不是尾部),两符号都覆盖;
#   2) 真实陀螺:per-window 残差偏置 N(0,σ_b≈0.003) + per-sample 白噪声 N(0,σ_n≈0.005),三轴都加;
#   3) 合成减速带/颠簸:~3% 帧聚成短事件,注入轮速 blip + 垂直加速度尖峰 + pitch/roll 角速率尖峰;
#   4) 加速度白噪声:让 accel 不至于不真实地干净。
# 全部只作用于 **data-driven 14ch 输入**(steer4+speed4+imu_g_base3+imu_a_base3),**标签 gt twist 不变**
#  (真机平面真运动在颠簸中近似平滑)。κ-bulk 旧 augment_kappa 的尾部 κ 由本文件 BULK 版接替。
#
# 通道布局(14ch):
#   [0:4]   steer (rad)
#   [4:8]   speed (m/s)   wheels: FL,FR,RL,RR -> 左轮=[4,6] 右轮=[5,7](WHEEL_POS +y/-y)
#   [8:11]  imu_g_base (rad/s)  axis0=roll-rate, axis1=pitch-rate, axis2=yaw-rate
#   [11:14] imu_a_base (m/s²)   axis2=vertical(+z, ~9.8 gravity)
import numpy as np

G0, A0 = 8, 11            # gyro / accel start cols (14ch data-driven layout)
_LEFT, _RIGHT = [4, 6], [5, 7]

# κ = wz/vx 对 左/右轮速相对差分 s 的解析斜率(直行 v=1,纯左右差速):κ ≈ -KAPPA_SLOPE·s。
# 实测(lwt.kinematics.ls_twist):left*(1+s)/right*(1-s) → κ = -2.2907·s。故 s = -κ/KAPPA_SLOPE。
KAPPA_SLOPE = 2.2907


def kappa_to_speed_scale(kappa):
    """目标 effective-κ → 左/右轮速相对差分 s。左轮 *(1+s)、右轮 *(1-s) 即得该 κ(直行近似)。"""
    return -float(kappa) / KAPPA_SLOPE


def apply_kappa_bulk(window, enable=True, kappa_std=0.04, rng=None):
    """κ-bulk 增广:per-window 直接采样目标 κ ~ N(0, kappa_std),映射为左右轮速差分,使
    real |κ|≈0.04 落在 effective-κ 分布 **中位**(两符号)。全窗同一组 scale(固定标定状态)。
    标签 twist 不变(调用方不动 Y)。enable=False 时原样返回。"""
    if not enable:
        return window
    rng = rng or np.random.default_rng()
    k = rng.normal(0.0, kappa_std)
    s = kappa_to_speed_scale(k)
    window[:, _LEFT] *= (1.0 + s)
    window[:, _RIGHT] *= (1.0 - s)
    return window


def apply_gyro_realism(window, enable=True, sigma_bias=0.003, sigma_noise=0.005, rng=None):
    """真实陀螺:imu_g_base 三轴同时加 (per-window 常值残差偏置 N(0,σ_b)) + (per-sample 白噪声 N(0,σ_n))。
    打破 "SIM 陀螺=完美 gt_wz" 假设,让模型在训练里见到去偏后仍残留的偏置+噪声。
    标签不变。enable=False 时原样返回。"""
    if not enable:
        return window
    rng = rng or np.random.default_rng()
    W = window.shape[0]
    b = rng.normal(0.0, sigma_bias, 3)                 # per-window 常值残差偏置(三轴)
    n = rng.normal(0.0, sigma_noise, (W, 3))           # per-sample 白噪声
    window[:, G0:G0 + 3] += (b[None, :] + n)
    return window


def apply_accel_noise(window, enable=True, sigma=0.05, rng=None):
    """imu_a_base 三轴小白噪声,使 accel 不至于不真实地干净。标签不变。"""
    if not enable:
        return window
    rng = rng or np.random.default_rng()
    window[:, A0:A0 + 3] += rng.normal(0.0, sigma, (window.shape[0], 3))
    return window


def apply_speed_bumps(window, enable=True, event_rate=0.03,
                      speed_blip=0.05, accel_z=2.0, pitchroll_rate=0.05,
                      event_len_min=3, event_len_max=10, rng=None,
                      return_mask=False):
    """合成减速带/颠簸:以 ~event_rate 的帧占比把扰动 **聚成短事件** 注入相关瞬态:
      - 4 轮速暂态 bounce/slip(±speed_blip 量级,带随机半正弦包络);
      - imu_a_base[:,2] 垂直加速度尖峰(±accel_z);
      - imu_g_base[:,0:2] pitch/roll 角速率尖峰(±pitchroll_rate)。
    yaw-rate 通道(imu_g_base[:,2])**不作为**设计注入目标——真机平面真运动在颠簸中近似平滑,
    gt twist 标签保持不变。事件数由 event_rate·W / 期望事件长 决定;每事件长 U[min,max]。
    enable=False 返回原窗(mask 全 0)。return_mask=True 额外返回 (W,) 命中事件帧的布尔 mask。"""
    W = window.shape[0]
    mask = np.zeros(W, dtype=bool)
    if not enable or W == 0:
        return (window, mask) if return_mask else window
    rng = rng or np.random.default_rng()
    mean_len = 0.5 * (event_len_min + event_len_max)
    target_frames = event_rate * W
    n_events = int(rng.poisson(max(target_frames / mean_len, 0.0)))
    for _ in range(n_events):
        L = int(rng.integers(event_len_min, event_len_max + 1))
        if L >= W:
            L = W
        start = int(rng.integers(0, W - L + 1))
        sl = slice(start, start + L)
        mask[sl] = True
        # 半正弦包络(0→1→0),让瞬态平滑起落、聚成事件而非孤立单帧。
        env = np.sin(np.linspace(0, np.pi, L))[:, None]            # (L,1)
        # 轮速 bounce/slip:每轮独立符号的瞬态(打滑/弹跳),量级 speed_blip。
        sgn = rng.choice([-1.0, 1.0], size=4)
        window[sl, 4:8] += (env * (sgn[None, :] * speed_blip) * np.abs(window[sl, 4:8]).mean()).astype(window.dtype) \
            if window[sl, 4:8].any() else (env * sgn[None, :] * speed_blip).astype(window.dtype)
        # 垂直加速度尖峰(单符号,撞击向上为主但允许双向)。
        az_sgn = rng.choice([-1.0, 1.0])
        window[sl, A0 + 2] += (env[:, 0] * az_sgn * accel_z).astype(window.dtype)
        # pitch/roll 角速率尖峰(两轴独立符号)。
        pr_sgn = rng.choice([-1.0, 1.0], size=2)
        window[sl, G0:G0 + 2] += (env * pr_sgn[None, :] * pitchroll_rate).astype(window.dtype)
    return (window, mask) if return_mask else window


class RealismAug:
    """编排器:按 config `realism:` 块对一个 data-driven 14ch 窗依次施加四类增广。
    每类独立 toggle;总开关 enable=False 时整体 no-op。**不接收也不修改标签**(调用方不动 Y)。
    施加顺序:κ-bulk(轮速尺度)→ bumps(瞬态)→ gyro 真实化 → accel 噪声;
    bumps 在 gyro/accel 噪声前注入,以便噪声叠在瞬态之上(更真实)。"""

    def __init__(self, cfg, rng=None):
        self.cfg = cfg or {}
        self.enable = bool(self.cfg.get("enable", False))
        self.rng = rng or np.random.default_rng()

    def __call__(self, window):
        if not self.enable:
            return window
        c = self.cfg
        kb = c.get("kappa_bulk", {})
        window = apply_kappa_bulk(window, enable=kb.get("enable", False),
                                  kappa_std=kb.get("kappa_std", 0.04), rng=self.rng)
        bp = c.get("bumps", {})
        window = apply_speed_bumps(window, enable=bp.get("enable", False),
                                   event_rate=bp.get("event_rate", 0.03),
                                   speed_blip=bp.get("speed_blip", 0.05),
                                   accel_z=bp.get("accel_z", 2.0),
                                   pitchroll_rate=bp.get("pitchroll_rate", 0.05),
                                   event_len_min=bp.get("event_len_min", 3),
                                   event_len_max=bp.get("event_len_max", 10), rng=self.rng)
        gy = c.get("gyro", {})
        window = apply_gyro_realism(window, enable=gy.get("enable", False),
                                    sigma_bias=gy.get("sigma_bias", 0.003),
                                    sigma_noise=gy.get("sigma_noise", 0.005), rng=self.rng)
        an = c.get("accel_noise", {})
        window = apply_accel_noise(window, enable=an.get("enable", False),
                                   sigma=an.get("sigma", 0.05), rng=self.rng)
        return window
