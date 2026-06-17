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
