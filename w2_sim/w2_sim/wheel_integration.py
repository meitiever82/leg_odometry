"""轮速 8×3 LS → SE2 积分轨迹;APE 评估。验证管线共用。"""
import numpy as np
from w2_sim.swerve_kinematics import forward_kinematics_ls


def integrate_wheel(times, angles_t, speeds_t):
    """逐帧 LS + 中点法 SE2 积分。返回 (N,4): [t, x, y, yaw],首帧位于原点。

    零阶保持:第 i 帧的轮速保持到第 i+1 帧;末帧轮速再保持一个中位 dt(否则
    N 个采样只覆盖 N-1 个区间,轨迹会短一个 dt)。这样直行/圆弧的解析终点严格闭合。
    """
    n = len(times)
    traj = np.zeros((n, 4))
    traj[:, 0] = times
    x = y = yaw = 0.0
    for i in range(1, n):
        dt = times[i] - times[i - 1]
        vx, vy, wz = forward_kinematics_ls(angles_t[i - 1], speeds_t[i - 1])
        ym = yaw + 0.5 * wz * dt
        x += (vx * np.cos(ym) - vy * np.sin(ym)) * dt
        y += (vx * np.sin(ym) + vy * np.cos(ym)) * dt
        yaw += wz * dt
        traj[i, 1:] = [x, y, yaw]
    # 末帧再保持一个中位 dt,补足第 N-1 个区间
    if n >= 2:
        dt = float(np.median(np.diff(times)))
        vx, vy, wz = forward_kinematics_ls(angles_t[-1], speeds_t[-1])
        ym = yaw + 0.5 * wz * dt
        x += (vx * np.cos(ym) - vy * np.sin(ym)) * dt
        y += (vx * np.sin(ym) + vy * np.cos(ym)) * dt
        yaw += wz * dt
        traj[-1, 1:] = [x, y, yaw]
    return traj


def ls_twist_series(times, angles_t, speeds_t):
    """逐帧 (vx, vy, wz),κ 回归用。"""
    return np.array([forward_kinematics_ls(a, s)
                     for a, s in zip(angles_t, speeds_t)])


def path_length(xy):
    return float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))


def ape_percent(traj, truth):
    """traj/truth: (N,3+) [t,x,y,...]。truth 按时间插值到 traj,首位姿 SE2 对齐,
    返回 RMSE 占 truth 路径长度的百分比。"""
    tx = np.interp(traj[:, 0], truth[:, 0], truth[:, 1])
    ty = np.interp(traj[:, 0], truth[:, 0], truth[:, 2])
    # 首位姿对齐:平移到共同原点(轨迹积分本就从原点出发,truth 减首点)
    tx, ty = tx - tx[0], ty - ty[0]
    ex, ey = traj[:, 1] - traj[0, 1], traj[:, 2] - traj[0, 2]
    rmse = np.sqrt(np.mean((ex - tx) ** 2 + (ey - ty) ** 2))
    return 100.0 * rmse / max(path_length(np.column_stack([tx, ty])), 1e-9)
