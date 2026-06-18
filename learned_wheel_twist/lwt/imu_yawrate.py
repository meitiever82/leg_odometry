# lwt/imu_yawrate.py —— 重力投影 base yaw-rate 通道(单标量,+CCW world-up,rad/s)。
#
# 设计动机(phase-2):phase-1 纯轮速残差被真机 κ≈-0.044 的航向漂移主导。
# IMU 陀螺仪直接观测 yaw-rate,是原则上的修正。但真机 /rslidar_imu_data 在一个
# 倾斜的传感器系里,我们没有真机 base→imu 外参;sim /sim/imu 已是 base z-up。
# 所以用「重力」这一绝对方向来定义竖直/yaw 轴,这正是 yaw-rate 通道所需的全部信息:
#
#   up = normalize(mean(accel))   # 全程平均下运动加速度≈0,均值≈比力(重力反作用)方向
#   yaw_rate_raw = up · gyro      # 把陀螺投影到竖直轴 → 绕竖直轴的角速度
#
# 符号约定(关键):通道含义必须在 sim 与 real 两域一致地表示 +CCW world yaw-rate。
#   - sim:up≈[0,0,1],up·gyro≈gyro_z,直接 ≈ +CCW,slope≈+1。
#   - real:加速度计可能上报「重力向下(-up)」而非比力向上,或存在手性翻转,
#     使几何 up·gyro 回归出 slope≈-1。此时对 real 通道施加一个固定符号翻转,
#     使 real 也回归到 slope≈+1。最终不变量:两域 yaw_rate 通道均=+CCW world
#     yaw-rate(rad/s),vs 各自真值 slope≈+1。符号规则由 `sign` 参数显式给出。
#
# 实测裁决(2026-06,verify_yawrate.py 区间 Δyaw 回归):
#   - SIM /sim/imu mean accel=[0,0,+9.81],up=[0,0,+1] → up·gyro=gyro_z,slope=+1.000 R²=1.0。
#   - REAL /rslidar_imu_data mean accel=[-8.34,-0.05,-5.13](倾斜且 Z 分量为负,加速度计报
#     「重力向下」),几何 up=[-0.85,-0.0,-0.52]。尽管 up 指向下/后,real 通道对参考 TUM 的
#     Δyaw 回归 slope=+1.00/+1.01/+1.06(3 bag),R²=0.996/0.983/0.767,**符号已为 +**。
#     即:倾斜下/后 up 与陀螺的手性恰好抵消了「重力向下」的翻转,几何投影直接给出 +CCW。
#   - 结论:sim 与 real **均用 sign=+1**,无需翻转。extract.py 默认 --imu-sign 1.0 即满足不变量。
import numpy as np


def project_yawrate(accel, gyro, sign=1.0):
    """重力投影 yaw-rate 纯函数。

    accel (N,3) 线加速度(m/s^2),gyro (N,3) 角速度(rad/s),同一 IMU 系、同一时戳。
    返回 yaw_rate (N,) = sign * (up · gyro),up = normalize(mean(accel over all N))。

    sign:全局符号(+1 或 -1),由各域回归真值解析后传入,使通道=+CCW world yaw-rate。
    """
    accel = np.asarray(accel, float)
    gyro = np.asarray(gyro, float)
    g = accel.mean(axis=0)
    n = np.linalg.norm(g)
    if n < 1e-9:
        # 退化(无重力信号):回退到 gyro_z(sim base z-up 时即正确)。
        up = np.array([0.0, 0.0, 1.0])
    else:
        up = g / n
    return sign * (gyro @ up)


def static_window_mask(t, speed, static_sec=2.0, speed_eps=0.05):
    """选取启动静止窗的布尔掩码(对齐到 t / speed 的同一长度时序)。

    迭代e(2026-06-18)修复:`imu_yawrate` 通道是重力投影后的 yaw-rate,但**未扣除陀螺零偏**;
    真机零偏投影到该轴 ~0.019 rad/s(≈1°/s),积分成跑飞的航向漂移。经典 wheel_odometry
    (calibrate_kappa.py)用静止窗均值 b_g 扣零偏;由 `imu_yawrate = up·gyro` 的线性关系,
    本通道的零偏 = up·b_g = **通道自身在静止窗的均值**,故只需对通道做静止窗去偏,无需原始陀螺。

    t (N,) 时戳(s),speed (N,K) 各轮速(m/s)。返回 (N,) bool。
    规则:优先取「前 static_sec 秒」;若该窗非静止(mean|speed|≥speed_eps),
    则沿时间滑动,找最早一个时长≈static_sec 且 mean|speed|<speed_eps 的窗;
    若全程都找不到,回退到「前 static_sec 秒」(由调用方记录 warning)。
    """
    t = np.asarray(t, float)
    speed = np.asarray(speed, float)
    spd = np.abs(speed)
    spd_mag = spd.max(axis=1) if spd.ndim == 2 else spd
    t0 = t[0]
    first = t < t0 + static_sec
    if first.sum() >= 1 and spd_mag[first].mean() < speed_eps:
        return first, True
    # 滑窗找最早静止段:以每个起点 i 取 [t_i, t_i+static_sec)。
    for i in range(len(t)):
        win = (t >= t[i]) & (t < t[i] + static_sec)
        if win.sum() < 1:
            continue
        # 只接受真正覆盖了 ~static_sec 的窗(末点已到 / 数据用尽时长够)。
        if t[win][-1] - t[win][0] < static_sec * 0.5:
            continue
        if spd_mag[win].mean() < speed_eps:
            return win, True
    return first, False  # 回退:前 static_sec 秒,found=False(调用方告警)


def debias_yawrate(yawrate, t, speed, static_sec=2.0, speed_eps=0.05):
    """对重力投影 yaw-rate 通道做启动静止窗去偏(扣陀螺零偏在该轴的投影)。

    yawrate (N,) 或 (N,1);t (N,) 时戳;speed (N,K) 各轮速。
    返回 (debiased_yawrate(同输入形状), bias(float), found_static(bool))。
    bias = 静止窗内 yawrate 均值;debiased = yawrate - bias。
    """
    yawrate = np.asarray(yawrate, float)
    flat = yawrate.reshape(-1)
    mask, found = static_window_mask(t, speed, static_sec, speed_eps)
    bias = float(flat[mask].mean())
    out = (flat - bias).reshape(yawrate.shape)
    return out, bias, found
