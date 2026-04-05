#!/usr/bin/env python3
"""运动学仿真数据生成器：为 leg odometry EKF 生成多场景评估数据集。

不依赖 Isaac Sim，通过运动学仿真生成逼真的行走数据：
- CPG 步态生成关节轨迹
- 从 body 轨迹反算 IMU（加速度 + 角速度）
- 输出 rosbag2 格式，包含 /joint_states, /imu, /ground_truth/odom

用法:
  python3 generate_sim_data.py [--output-dir OUTPUT_DIR] [--scenario SCENARIO|all]
"""

import argparse
import math
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

# ============================================================
# rosbag2 写入（使用 rosbag2_py）
# ============================================================
try:
    import rclpy.serialization
    from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions
    from rosbag2_py._storage import TopicMetadata
    from sensor_msgs.msg import JointState, Imu
    from nav_msgs.msg import Odometry, Path as NavPath
    from geometry_msgs.msg import PoseStamped
    from builtin_interfaces.msg import Time as TimeMsg
    from std_msgs.msg import Header
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    print("Warning: ROS2 not available, will generate .npz instead of rosbag")


# ============================================================
# 数学工具
# ============================================================

def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def rotation_matrix_z(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rotation_matrix_y(pitch):
    c, s = np.cos(pitch), np.sin(pitch)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rotation_matrix_x(roll):
    c, s = np.cos(roll), np.sin(roll)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def numerical_derivative(values, dt):
    """5阶中心差分求导 + Savitzky-Golay 平滑，减少数值误差。"""
    N = len(values)
    deriv = np.zeros_like(values)

    # 5阶中心差分（内部点）
    if N > 4:
        deriv[2:-2] = (-values[4:] + 8*values[3:-1] - 8*values[1:-3] + values[:-4]) / (12 * dt)

    # 边界用低阶差分
    if N > 1:
        deriv[0] = (values[1] - values[0]) / dt
        deriv[-1] = (values[-1] - values[-2]) / dt
    if N > 2:
        deriv[1] = (values[2] - values[0]) / (2 * dt)
        deriv[-2] = (values[-1] - values[-3]) / (2 * dt)

    return deriv


# ============================================================
# CASBot02 腿部参数（从 URDF 提取）
# ============================================================

@dataclass
class LegParams:
    """CASBot02 腿部运动学参数。"""
    # 髋关节到 base_link 的偏移
    hip_offset_y: float = 0.061       # 左右方向
    hip_offset_z: float = 0.0575      # 上下方向

    # 连杆长度（从 URDF joint origins 推算）
    thigh_length: float = 0.280       # 大腿 (pelvic_roll → knee)
    shank_length: float = 0.280       # 小腿 (knee → ankle)
    foot_height: float = 0.035        # 脚踝到地面

    # 站立时 base_link 到地面高度
    @property
    def standing_height(self):
        return self.thigh_length + self.shank_length + self.foot_height + self.hip_offset_z


LEG = LegParams()


# ============================================================
# CPG 步态生成器
# ============================================================

@dataclass
class GaitParams:
    """步态参数。"""
    step_frequency: float = 1.5       # Hz (步频)
    step_length: float = 0.15         # m (步长 - 单脚)
    step_height: float = 0.03         # m (抬脚高度)
    body_sway: float = 0.015          # m (身体左右摇摆幅度)
    body_bob: float = 0.008           # m (身体上下起伏幅度)
    body_pitch_amp: float = 0.02      # rad (身体前后俯仰)
    body_roll_amp: float = 0.02       # rad (身体左右滚转)
    swing_ratio: float = 0.4          # 摆动相占步态周期比例（正常行走 ~40%）


def generate_foot_trajectories_world(t_array, gt_pos, gt_R, gait: GaitParams,
                                     foot_stand_left, foot_stand_right,
                                     standstill: float = 0.0):
    """在世界系中生成左右足端轨迹，确保支撑期脚固定。

    Args:
        t_array: 时间数组
        gt_pos: (N, 3) body 世界系位置
        gt_R: list of (3,3) body 旋转矩阵
        gait: 步态参数
        foot_stand_left: 左脚在 body 系的站立位置 (FK at zero)
        foot_stand_right: 右脚在 body 系的站立位置
        standstill: 静止期时长(s)，该期间双脚都在地面

    Returns:
        foot_left_body: (N, 3) 左脚在 body 系的位置（相对于 stand 的偏移）
        foot_right_body: (N, 3) 右脚在 body 系的位置（相对于 stand 的偏移）
        contact_left: (N,) bool
        contact_right: (N,) bool
    """
    T = 1.0 / gait.step_frequency
    N = len(t_array)
    swing_ratio = gait.swing_ratio

    # 相位布局: [0, swing_ratio) = 摆动, [swing_ratio, 1) = 支撑
    # 初始相位偏移让 t=0 时两只脚都在支撑相中间
    # 左脚: phase_init = (swing_ratio + 1) / 2 = 在支撑相中间
    # 右脚: phase_init + 0.5
    phase_init_left = (swing_ratio + 1.0) / 2.0
    phase_init_right = phase_init_left + 0.5

    def compute_phase(t, phase_offset):
        t_motion = max(0.0, t - standstill)
        return ((t_motion / T) + phase_offset) % 1.0

    def is_contact(t, phase_offset):
        if t < standstill:
            return True  # 静止期双脚着地
        phase = compute_phase(t, phase_offset)
        return phase >= swing_ratio

    # --- 第一遍：确定每个时间步的接触状态 ---
    contact_left = np.array([is_contact(t, phase_init_left) for t in t_array])
    contact_right = np.array([is_contact(t, phase_init_right) for t in t_array])

    # --- 世界系足端轨迹 ---
    foot_left_world = np.zeros((N, 3))
    foot_right_world = np.zeros((N, 3))

    def generate_foot_world(contact, phase_offset, foot_stand):
        """为一只脚生成世界系轨迹。"""
        foot_world = np.zeros((N, 3))

        # 初始化：第一个支撑位置
        foot_world[0] = gt_pos[0] + gt_R[0] @ foot_stand

        # 跟踪当前支撑位置和摆动起止位置
        stance_pos = foot_world[0].copy()
        swing_start_pos = None
        swing_end_pos = None
        was_contact = is_contact(t_array[0], phase_offset)

        for i in range(N):
            phase = compute_phase(t_array[i], phase_offset)
            in_contact = is_contact(t_array[i], phase_offset)

            if in_contact:
                # 支撑相：脚固定在世界系
                foot_world[i] = stance_pos
            else:
                # 摆动相
                s = phase / swing_ratio  # [0, 1]

                if not was_contact and swing_start_pos is not None:
                    # 继续摆动
                    pass
                else:
                    # 摆动开始，记录起始位置，计算终止位置
                    swing_start_pos = stance_pos.copy()
                    # 预测下一个落脚点：当前 body 位置前方 step_length 处
                    # 找摆动结束时的 body 位置（近似）
                    t_end_swing = t_array[i] + swing_ratio * T
                    # 简单近似：用当前 body 的朝向，往前迈 step_length
                    R_cur = gt_R[i]
                    # 前进方向 = body x 轴在世界系中的投影
                    forward = R_cur @ np.array([1, 0, 0])
                    forward[2] = 0  # 水平面
                    if np.linalg.norm(forward[:2]) > 1e-6:
                        forward = forward / np.linalg.norm(forward) * gait.step_length
                    else:
                        forward = np.array([gait.step_length, 0, 0])

                    # 找落脚时的近似 body 位置
                    idx_end = min(int(i + swing_ratio * T / (t_array[1] - t_array[0])), N - 1)
                    body_at_landing = gt_pos[idx_end]
                    R_at_landing = gt_R[idx_end]
                    # 落脚点 = 落地时 body 位置 + 旋转后的 foot_stand
                    swing_end_pos = body_at_landing + R_at_landing @ foot_stand

                # 插值 (使用平滑插值)
                # 位置: 三次 Hermite
                foot_world[i, :2] = swing_start_pos[:2] + s * (swing_end_pos[:2] - swing_start_pos[:2])
                # Z: 抛物线抬脚
                foot_world[i, 2] = (swing_start_pos[2] + s * (swing_end_pos[2] - swing_start_pos[2])
                                     + gait.step_height * 4 * s * (1 - s))

            # 状态转换检测
            if in_contact and not was_contact:
                # 刚着地，更新支撑位置
                stance_pos = foot_world[i].copy()

            was_contact = in_contact

        return foot_world

    foot_left_world = generate_foot_world(contact_left, phase_init_left, foot_stand_left)
    foot_right_world = generate_foot_world(contact_right, phase_init_right, foot_stand_right)

    # --- 转换到 body 系（偏移量 = body系实际位置 - stand位置）---
    foot_left_body = np.zeros((N, 3))
    foot_right_body = np.zeros((N, 3))
    for i in range(N):
        R_inv = gt_R[i].T
        # body 系中的足端绝对位置
        fl_body_abs = R_inv @ (foot_left_world[i] - gt_pos[i])
        fr_body_abs = R_inv @ (foot_right_world[i] - gt_pos[i])
        # 相对于站立位置的偏移
        foot_left_body[i] = fl_body_abs - foot_stand_left
        foot_right_body[i] = fr_body_abs - foot_stand_right

    return foot_left_body, foot_right_body, contact_left, contact_right


# ============================================================
# 场景定义
# ============================================================

@dataclass
class Scenario:
    """一个仿真场景的完整描述。"""
    name: str
    description: str
    duration: float                           # 秒
    gait: GaitParams = field(default_factory=GaitParams)
    # body 轨迹函数: t -> (x, y, z, roll, pitch, yaw) in world frame
    trajectory_fn: object = None


def _straight_walk(speed=0.5, duration=30.0, freq=1.5, step_len=0.15):
    """直线行走场景工厂。"""
    gait = GaitParams(step_frequency=freq, step_length=step_len)

    def traj(t):
        x = speed * t
        y = 0.0
        z = LEG.standing_height
        return x, y, z, 0.0, 0.0, 0.0

    return gait, traj, duration


def _turn_in_place(yaw_rate=0.3, duration=25.0):
    """原地转弯。"""
    gait = GaitParams(step_frequency=1.2, step_length=0.02, step_height=0.02)

    def traj(t):
        yaw = yaw_rate * t
        return 0.0, 0.0, LEG.standing_height, 0.0, 0.0, yaw

    return gait, traj, duration


def _curve_walk(radius=3.0, speed=0.4, duration=40.0):
    """曲线行走（圆弧）。"""
    omega = speed / radius
    gait = GaitParams(step_frequency=1.5, step_length=0.12)

    def traj(t):
        theta = omega * t
        x = radius * np.sin(theta)
        y = radius * (1 - np.cos(theta))
        yaw = theta
        return x, y, LEG.standing_height, 0.0, 0.0, yaw

    return gait, traj, duration


def _s_curve(speed=0.4, amplitude=2.0, period=20.0, duration=45.0):
    """S 形曲线行走。"""
    gait = GaitParams(step_frequency=1.5, step_length=0.12)

    def traj(t):
        x = speed * t
        y = amplitude * np.sin(2 * np.pi * t / period)
        # yaw 跟随切线方向
        dy_dt = amplitude * 2 * np.pi / period * np.cos(2 * np.pi * t / period)
        dx_dt = speed
        yaw = np.arctan2(dy_dt, dx_dt)
        return x, y, LEG.standing_height, 0.0, 0.0, yaw

    return gait, traj, duration


def _stop_and_go(speed=0.5, walk_time=5.0, stop_time=2.0, cycles=5):
    """走走停停。"""
    duration = cycles * (walk_time + stop_time)
    gait = GaitParams(step_frequency=1.5, step_length=0.15)

    def traj(t):
        cycle_t = t % (walk_time + stop_time)
        # 计算走过的总距离
        full_cycles = int(t / (walk_time + stop_time))
        dist = full_cycles * speed * walk_time
        if cycle_t < walk_time:
            dist += speed * cycle_t
        else:
            dist += speed * walk_time

        x = dist
        return x, 0.0, LEG.standing_height, 0.0, 0.0, 0.0

    return gait, traj, duration


def _slope_walk(speed=0.4, slope_deg=5.0, duration=30.0):
    """上下斜坡（先上后下）。"""
    slope_rad = np.radians(slope_deg)
    half = duration / 2
    gait = GaitParams(step_frequency=1.3, step_length=0.10)

    def traj(t):
        if t < half:
            # 上坡
            x = speed * t * np.cos(slope_rad)
            z = LEG.standing_height + speed * t * np.sin(slope_rad)
            pitch = -slope_rad
        else:
            # 下坡
            dt = t - half
            x_up = speed * half * np.cos(slope_rad)
            z_up = speed * half * np.sin(slope_rad)
            x = x_up + speed * dt * np.cos(slope_rad)
            z = LEG.standing_height + z_up - speed * dt * np.sin(slope_rad)
            pitch = slope_rad

        return x, 0.0, z, 0.0, pitch, 0.0

    return gait, traj, duration


def _long_walk(speed=0.45, duration=300.0):
    """长时间行走（5 分钟），带轻微方向变化。"""
    gait = GaitParams(step_frequency=1.5, step_length=0.13)

    def traj(t):
        # 缓慢的正弦形路径
        x = speed * t
        y = 1.5 * np.sin(2 * np.pi * t / 60.0)
        dy_dt = 1.5 * 2 * np.pi / 60.0 * np.cos(2 * np.pi * t / 60.0)
        yaw = np.arctan2(dy_dt, speed)
        return x, y, LEG.standing_height, 0.0, 0.0, yaw

    return gait, traj, duration


SCENARIOS = {}


def _register(name, desc, factory_result):
    gait, traj_fn, duration = factory_result
    SCENARIOS[name] = Scenario(
        name=name, description=desc, duration=duration,
        gait=gait, trajectory_fn=traj_fn,
    )


_register('straight_slow', '直线行走 0.3 m/s',
          _straight_walk(speed=0.3, duration=30, freq=1.2, step_len=0.12))
_register('straight_medium', '直线行走 0.5 m/s',
          _straight_walk(speed=0.5, duration=30, freq=1.5, step_len=0.15))
_register('straight_fast', '直线行走 0.8 m/s',
          _straight_walk(speed=0.8, duration=25, freq=2.0, step_len=0.20))
_register('turn_in_place', '原地转弯 (0.3 rad/s)',
          _turn_in_place(yaw_rate=0.3, duration=25))
_register('curve_walk', '曲线行走 (R=3m)',
          _curve_walk(radius=3.0, speed=0.4, duration=40))
_register('s_curve', 'S 形曲线行走',
          _s_curve(speed=0.4, amplitude=2.0, period=20.0, duration=45))
_register('stop_and_go', '走走停停 (5s走/2s停 ×5)',
          _stop_and_go(speed=0.5, walk_time=5.0, stop_time=2.0, cycles=5))
_register('slope_up_down', '上下斜坡 (5°)',
          _slope_walk(speed=0.4, slope_deg=5.0, duration=30))
_register('long_walk', '长时间行走 (5分钟)',
          _long_walk(speed=0.45, duration=300))


# ============================================================
# 仿真核心
# ============================================================

def simulate_scenario(scenario: Scenario, dt: float = 0.005):
    """运行一个场景的运动学仿真。

    Args:
        scenario: 场景描述
        dt: 仿真步长 (s), 200Hz

    Returns:
        dict with keys:
            t:              (N,) 时间
            gt_position:    (N, 3) ground truth 位置
            gt_orientation: (N, 4) ground truth 四元数 [x,y,z,w]
            gt_velocity:    (N, 3) ground truth 速度 (world)
            accel:          (N, 3) 加速度计读数 (body)
            gyro:           (N, 3) 陀螺仪读数 (body)
            joint_positions:(N, 12) 12 个腿关节角度
            joint_names:    list of 12 joint names
            contact_left:   (N,) bool
            contact_right:  (N,) bool
            effort_left:    (N,) 左踝力矩
            effort_right:   (N,) 右踝力矩
    """
    # 在运动前加入静止期，让 EKF 正确初始化
    STANDSTILL = 2.0   # 静止时间 (s)
    RAMP = 1.0         # 步态渐入时间 (s)
    total_duration = STANDSTILL + scenario.duration
    N = int(total_duration / dt) + 1
    t_array = np.linspace(0, total_duration, N)

    # ---- 1. body 轨迹 ----
    gt_pos = np.zeros((N, 3))
    gt_rpy = np.zeros((N, 3))

    def _warp_time(t_motion):
        """时间扭曲：平滑从 0 加速到全速，避免速度阶跃。

        在 RAMP 时间内，速度从 0 平滑增加到 1x，
        返回 "等效时间"（如果一直以全速运动的话对应的时刻）。
        """
        if RAMP <= 0 or t_motion >= RAMP:
            # ramp 结束后正常：t_eff = t_motion - RAMP/2
            return t_motion - RAMP / 2.0 if RAMP > 0 else t_motion
        s = t_motion / RAMP  # [0, 1]
        # smoothstep(s) 作为速度缩放
        v_scale = s * s * (3 - 2 * s)
        # 积分得到等效时间: ∫₀^(s·RAMP) smoothstep(u/RAMP) du
        # = RAMP · (s³ - s⁴/2)
        return RAMP * (s**3 - 0.5 * s**4)

    for i, t in enumerate(t_array):
        t_motion = max(0.0, t - STANDSTILL)  # 运动时间（减去静止期）
        t_eff = _warp_time(t_motion)           # 平滑等效时间
        t_eff = max(0.0, t_eff)
        x, y, z, roll, pitch, yaw = scenario.trajectory_fn(t_eff)
        gt_pos[i] = [x, y, z]
        gt_rpy[i] = [roll, pitch, yaw]

    # 添加步态引起的 body 晃动（带 ramp 渐入）
    gait = scenario.gait
    T_step = 1.0 / gait.step_frequency
    for i, t in enumerate(t_array):
        t_motion = max(0.0, t - STANDSTILL)
        # ramp: 0→1 在 RAMP 秒内平滑过渡
        ramp = min(1.0, t_motion / RAMP) if RAMP > 0 else (1.0 if t_motion > 0 else 0.0)
        ramp = ramp * ramp * (3 - 2 * ramp)  # smoothstep

        t_eff = max(0.0, _warp_time(t_motion))
        phase = (t_eff / T_step) % 1.0
        # 左右摇摆 (与步频同频)
        gt_pos[i, 1] += ramp * gait.body_sway * np.sin(2 * np.pi * phase)
        # 上下起伏 (步频两倍频)
        gt_pos[i, 2] += ramp * gait.body_bob * np.sin(4 * np.pi * phase)
        # 俯仰
        gt_rpy[i, 1] += ramp * gait.body_pitch_amp * np.sin(4 * np.pi * phase)
        # 滚转
        gt_rpy[i, 0] += ramp * gait.body_roll_amp * np.sin(2 * np.pi * phase)

    # 四元数
    gt_quat = np.zeros((N, 4))
    gt_R = []
    for i in range(N):
        R = rotation_matrix_z(gt_rpy[i, 2]) @ \
            rotation_matrix_y(gt_rpy[i, 1]) @ \
            rotation_matrix_x(gt_rpy[i, 0])
        gt_R.append(R)
        r = Rotation.from_matrix(R)
        gt_quat[i] = r.as_quat()  # [x,y,z,w]

    # ---- 2. 速度、加速度 ----
    gt_vel = numerical_derivative(gt_pos, dt)
    gt_acc_world = numerical_derivative(gt_vel, dt)
    # body 角速度 (数值差分 RPY)
    rpy_dot = numerical_derivative(gt_rpy, dt)

    # ---- 3. IMU 读数 ----
    gravity = np.array([0, 0, -9.81])
    accel_body = np.zeros((N, 3))
    gyro_body = np.zeros((N, 3))

    for i in range(N):
        R = gt_R[i]
        # 加速度计 = R^T @ (acc_world - gravity)
        accel_body[i] = R.T @ (gt_acc_world[i] - gravity)

        # 陀螺仪: 从 RPY 导数转换到 body 角速度
        # 简化: 小角度时 omega_body ≈ R^T @ [rpy_dot mapped to world]
        # 更准确的公式:
        roll, pitch, yaw = gt_rpy[i]
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        # RPY 导数到 body 角速度的变换矩阵
        E = np.array([
            [1, 0, -sp],
            [0, cr, sr * cp],
            [0, -sr, cr * cp],
        ])
        gyro_body[i] = E @ rpy_dot[i]

    # 添加 IMU 噪声
    accel_noise_std = 0.05   # m/s^2
    gyro_noise_std = 0.003   # rad/s
    accel_bias = np.array([0.02, -0.01, 0.03])   # 固定小 bias
    gyro_bias = np.array([0.001, -0.0005, 0.0008])

    accel_body += accel_bias + np.random.randn(N, 3) * accel_noise_std
    gyro_body += gyro_bias + np.random.randn(N, 3) * gyro_noise_std

    # ---- 4. 足端轨迹 & 关节角 (使用真实 URDF IK, 世界系一致) ----
    urdf_path = str(Path(__file__).resolve().parent.parent.parent /
                     'finder_lidar_mapping' / 'glim_ros2' / 'urdf' /
                     'casbot02_7dof_shell.urdf')
    ik_solver = URDFBasedIK(urdf_path)

    # 在世界系中生成足端轨迹（支撑期脚固定）
    foot_left, foot_right, contact_left, contact_right = \
        generate_foot_trajectories_world(
            t_array, gt_pos, gt_R, gait,
            ik_solver.left_foot_stand, ik_solver.right_foot_stand,
            standstill=STANDSTILL)

    joint_names = ik_solver.left_names + ik_solver.right_names
    n_left = len(ik_solver.left_names)
    n_right = len(ik_solver.right_names)
    joint_positions = np.zeros((N, n_left + n_right))

    for i in range(N):
        joint_positions[i, :n_left] = ik_solver.solve(foot_left[i], side='left')
        joint_positions[i, n_left:] = ik_solver.solve(foot_right[i], side='right')

    # ---- 5. 踝关节力矩（模拟，用于接触检测） ----
    effort_left = np.where(contact_left, 15.0 + np.random.randn(N) * 2.0, 1.0 + np.random.randn(N) * 0.5)
    effort_right = np.where(contact_right, 15.0 + np.random.randn(N) * 2.0, 1.0 + np.random.randn(N) * 0.5)

    return {
        't': t_array,
        'dt': dt,
        'gt_position': gt_pos,
        'gt_orientation': gt_quat,
        'gt_velocity': gt_vel,
        'gt_rpy': gt_rpy,
        'accel': accel_body,
        'gyro': gyro_body,
        'joint_positions': joint_positions,
        'joint_names': joint_names,
        'contact_left': contact_left,
        'contact_right': contact_right,
        'effort_left': effort_left,
        'effort_right': effort_right,
    }


class URDFBasedIK:
    """使用真实 URDF FK 做数值迭代 IK。

    确保生成的关节角通过 FK 能准确还原目标足端位置。
    """

    def __init__(self, urdf_path: str):
        from leg_odometry.kinematics import LegKinematics
        with open(urdf_path) as f:
            urdf_str = f.read()
        self.kin = LegKinematics(
            urdf_str,
            base_link='base_link',
            left_foot_link='left_leg_ankle_roll_link',
            right_foot_link='right_leg_ankle_roll_link',
        )
        self.left_names = self.kin.left_joint_names
        self.right_names = self.kin.right_joint_names
        # 缓存上一帧解，用作初始值
        self._last_q_left = np.zeros(len(self.left_names))
        self._last_q_right = np.zeros(len(self.right_names))

        # 找到站立时的足端位置（所有关节为零）
        zero_joints = {n: 0.0 for n in self.left_names + self.right_names}
        self.left_foot_stand = self.kin.fk_left(zero_joints)
        self.right_foot_stand = self.kin.fk_right(zero_joints)

    def solve(self, target_pos_body: np.ndarray, side: str = 'left',
              max_iter: int = 50, tol: float = 1e-5) -> np.ndarray:
        """数值 IK：求解关节角使 FK(q) = target。

        target_pos_body: 足端目标位置（body 系），是相对于站立位置的偏移。
        """
        if side == 'left':
            names = self.left_names
            fk_fn = self.kin.fk_left
            stand = self.left_foot_stand
            q = self._last_q_left.copy()
        else:
            names = self.right_names
            fk_fn = self.kin.fk_right
            stand = self.right_foot_stand
            q = self._last_q_right.copy()

        # 目标：绝对位置 = 站立位置 + 偏移
        target = stand + target_pos_body
        n_joints = len(names)

        for iteration in range(max_iter):
            joints_dict = {names[j]: q[j] for j in range(n_joints)}
            current = fk_fn(joints_dict)
            err = target - current

            if np.linalg.norm(err) < tol:
                break

            # 数值雅可比
            J = np.zeros((3, n_joints))
            eps = 1e-6
            for j in range(n_joints):
                q_plus = q.copy()
                q_plus[j] += eps
                joints_plus = {names[k]: q_plus[k] for k in range(n_joints)}
                fk_plus = fk_fn(joints_plus)
                J[:, j] = (fk_plus - current) / eps

            # 阻尼最小二乘
            lam = 0.01
            dq = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(3), err)
            # 限制步长
            dq = np.clip(dq, -0.1, 0.1)
            q += dq

        # 缓存结果
        if side == 'left':
            self._last_q_left = q.copy()
        else:
            self._last_q_right = q.copy()

        return q


# ============================================================
# rosbag2 输出
# ============================================================

def stamp_from_float(t, t0_sec=1700000000):
    """将浮点秒转为 ROS Time。"""
    total = t0_sec + t
    sec = int(total)
    nanosec = int((total - sec) * 1e9)
    return sec, nanosec


def write_rosbag(data: dict, scenario: Scenario, output_dir: str):
    """将仿真数据写成 rosbag2。"""
    if not HAS_ROS:
        # fallback: 保存 npz
        npz_path = os.path.join(output_dir, f'{scenario.name}.npz')
        np.savez_compressed(npz_path, **data, scenario_name=scenario.name,
                            scenario_desc=scenario.description)
        print(f"  Saved {npz_path}")
        return

    import shutil
    bag_dir = os.path.join(output_dir, scenario.name)
    # rosbag2 不允许覆盖，先删除；且不能预创建目录
    if os.path.exists(bag_dir):
        shutil.rmtree(bag_dir)

    storage_options = StorageOptions(uri=bag_dir, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')

    writer = SequentialWriter()
    writer.open(storage_options, converter_options)

    # 创建话题
    writer.create_topic(TopicMetadata(
        name='/joint_states', type='sensor_msgs/msg/JointState',
        serialization_format='cdr'))
    writer.create_topic(TopicMetadata(
        name='/imu', type='sensor_msgs/msg/Imu',
        serialization_format='cdr'))
    writer.create_topic(TopicMetadata(
        name='/ground_truth/odom', type='nav_msgs/msg/Odometry',
        serialization_format='cdr'))

    t_array = data['t']
    N = len(t_array)

    # URDF 名 → bag 名 反向映射
    URDF_TO_BAG = {
        'left_leg_pelvic_pitch_joint': 'LJ0',
        'left_leg_pelvic_roll_joint': 'LJ1',
        'left_leg_pelvic_yaw_joint': 'LJ2',
        'left_leg_knee_pitch_joint': 'LJ3',
        'left_leg_ankle_pitch_joint': 'LJPITCH',
        'left_leg_ankle_roll_joint': 'LJROLL',
        'right_leg_pelvic_pitch_joint': 'RJ6',
        'right_leg_pelvic_roll_joint': 'RJ7',
        'right_leg_pelvic_yaw_joint': 'RJ8',
        'right_leg_knee_pitch_joint': 'RJ9',
        'right_leg_ankle_pitch_joint': 'RJPITCH',
        'right_leg_ankle_roll_joint': 'RJROLL',
    }
    urdf_joint_names = data['joint_names']
    bag_joint_names = [URDF_TO_BAG.get(n, n) for n in urdf_joint_names]
    n_joints = len(bag_joint_names)

    # 找到 LJPITCH 和 RJPITCH 的索引，用于 effort
    ljpitch_idx = bag_joint_names.index('LJPITCH') if 'LJPITCH' in bag_joint_names else -1
    rjpitch_idx = bag_joint_names.index('RJPITCH') if 'RJPITCH' in bag_joint_names else -1

    for i in range(N):
        t = t_array[i]
        sec, nanosec = stamp_from_float(t)
        stamp = TimeMsg(sec=sec, nanosec=nanosec)
        ts_ns = sec * 10**9 + nanosec

        header = Header(stamp=stamp, frame_id='')

        # --- JointState ---
        js = JointState()
        js.header = Header(stamp=stamp, frame_id='')
        js.name = list(bag_joint_names)
        js.position = data['joint_positions'][i].tolist()
        js.velocity = [0.0] * n_joints
        # effort: 只有踝关节有力矩
        efforts = [0.0] * n_joints
        if ljpitch_idx >= 0:
            efforts[ljpitch_idx] = data['effort_left'][i]
        if rjpitch_idx >= 0:
            efforts[rjpitch_idx] = data['effort_right'][i]
        js.effort = efforts

        writer.write('/joint_states',
                     rclpy.serialization.serialize_message(js), ts_ns)

        # --- IMU ---
        imu = Imu()
        imu.header = Header(stamp=stamp, frame_id='imu_link')
        imu.linear_acceleration.x = float(data['accel'][i, 0])
        imu.linear_acceleration.y = float(data['accel'][i, 1])
        imu.linear_acceleration.z = float(data['accel'][i, 2])
        imu.angular_velocity.x = float(data['gyro'][i, 0])
        imu.angular_velocity.y = float(data['gyro'][i, 1])
        imu.angular_velocity.z = float(data['gyro'][i, 2])
        # orientation from ground truth (有些 IMU 提供)
        imu.orientation.x = float(data['gt_orientation'][i, 0])
        imu.orientation.y = float(data['gt_orientation'][i, 1])
        imu.orientation.z = float(data['gt_orientation'][i, 2])
        imu.orientation.w = float(data['gt_orientation'][i, 3])

        writer.write('/imu',
                     rclpy.serialization.serialize_message(imu), ts_ns)

        # --- Ground Truth Odometry ---
        odom = Odometry()
        odom.header = Header(stamp=stamp, frame_id='world')
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(data['gt_position'][i, 0])
        odom.pose.pose.position.y = float(data['gt_position'][i, 1])
        odom.pose.pose.position.z = float(data['gt_position'][i, 2])
        odom.pose.pose.orientation.x = float(data['gt_orientation'][i, 0])
        odom.pose.pose.orientation.y = float(data['gt_orientation'][i, 1])
        odom.pose.pose.orientation.z = float(data['gt_orientation'][i, 2])
        odom.pose.pose.orientation.w = float(data['gt_orientation'][i, 3])
        odom.twist.twist.linear.x = float(data['gt_velocity'][i, 0])
        odom.twist.twist.linear.y = float(data['gt_velocity'][i, 1])
        odom.twist.twist.linear.z = float(data['gt_velocity'][i, 2])

        writer.write('/ground_truth/odom',
                     rclpy.serialization.serialize_message(odom), ts_ns)

    del writer
    print(f"  Saved rosbag: {bag_dir}")


# ============================================================
# 主程序
# ============================================================

def main():
    parser = argparse.ArgumentParser(description='生成 leg odometry 评估数据集')
    parser.add_argument('--output-dir', '-o', default='data/sim',
                        help='输出目录 (default: data/sim)')
    parser.add_argument('--scenario', '-s', default='all',
                        help=f'场景名 (可选: {", ".join(SCENARIOS.keys())}, all)')
    parser.add_argument('--dt', type=float, default=0.005,
                        help='仿真步长 (default: 0.005 = 200Hz)')
    parser.add_argument('--seed', type=int, default=42,
                        help='随机种子 (default: 42)')
    args = parser.parse_args()

    np.random.seed(args.seed)

    output_dir = os.path.join(os.getcwd(), args.output_dir)
    os.makedirs(output_dir, exist_ok=True)

    if args.scenario == 'all':
        scenarios = list(SCENARIOS.values())
    elif args.scenario in SCENARIOS:
        scenarios = [SCENARIOS[args.scenario]]
    else:
        print(f"Unknown scenario: {args.scenario}")
        print(f"Available: {', '.join(SCENARIOS.keys())}, all")
        sys.exit(1)

    print(f"Output: {output_dir}")
    print(f"Scenarios: {len(scenarios)}")
    print(f"Simulation dt: {args.dt}s ({1/args.dt:.0f} Hz)")
    print()

    for sc in scenarios:
        print(f"[{sc.name}] {sc.description} ({sc.duration}s)")
        data = simulate_scenario(sc, dt=args.dt)

        # 打印统计
        dist = np.linalg.norm(data['gt_position'][-1, :2] - data['gt_position'][0, :2])
        yaw_total = abs(data['gt_rpy'][-1, 2] - data['gt_rpy'][0, 2])
        print(f"  Total distance: {dist:.2f}m, yaw change: {np.degrees(yaw_total):.1f}°")
        print(f"  Samples: {len(data['t'])}, contacts L/R: "
              f"{data['contact_left'].sum()}/{data['contact_right'].sum()}")

        write_rosbag(data, sc, output_dir)
        print()

    print("Done!")


if __name__ == '__main__':
    main()
