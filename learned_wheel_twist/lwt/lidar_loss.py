# lwt/lidar_loss.py —— 阶段六 Step B:真机激光监督可微轨迹损失。
#
# 动机(见 test_report 阶段五/五-b):仿真里 gt_wz≈完美、轮速尺度≈1,真机的系统误差
# (wheel-scale 6-9%、gyro 残差偏置 ~0.003-0.006 rad/s)在 sim 里**不可见**(无错可改)。
# 唯一能看见真偏置的老师 = **真机激光轨迹**。本模块把激光 TUM 参考插值到段窗末时戳、转成
# 局部 (x,y,yaw) 目标,复用 frame-invariant 的 `trajectory_rpe_loss`(无全局对齐作弊)做监督。
#
# 设计要点:
#  - frame-invariant:RPE 对全局刚体变换不变(损失只比"旋入起点航向系的相对位移 + Δyaw"),
#    所以无需把 pred 与 lidar 做全局 Umeyama 对齐——杜绝"对齐后看着准"的作弊。
#  - 稀疏鲁棒:激光参考 dt 可能 0.1-0.3s(比轮速 50Hz 稀),线性插值到窗末时戳即可;
#    yaw 由四元数(平面运动主用 z 分量)解算后插值(unwrap 防 ±π 跳变)。
import numpy as np
import torch
from lwt.losses import trajectory_rpe_loss


def quat_to_yaw(quat):
    """四元数 (...,4)=[qx,qy,qz,qw] → 绕 z 的 yaw(rad)。平面运动主用 z/w,但用完整公式更稳。
    yaw = atan2(2(qw qz + qx qy), 1 - 2(qy² + qz²))。返回 numpy (...,)。"""
    q = np.asarray(quat, float)
    qx, qy, qz, qw = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    return np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def lidar_poses_at_times(tum, query_t):
    """激光 TUM 参考插值到 query 时戳,返回局部系 (M,3)=[x,y,yaw]。

    tum: (N,8)=[t tx ty tz qx qy qz qw](已按 t 升序;否则内部排序)。
    query_t: (M,) 目标时戳(段窗末,绝对 epoch)。
    返回 numpy (M,3),**首点平移到原点**(局部系,与 integrate_twist 起点 (0,0,0) 同口径);
    yaw 也以首点为基准减去 yaw[0](局部航向),frame-invariant RPE 下其实不依赖该平移/旋转,
    但保持与 pred 同口径便于调试/可视化。time-clamp:query 落在参考时间范围外则取端点(np.interp 行为)。
    """
    T = np.asarray(tum, float)
    order = np.argsort(T[:, 0])
    T = T[order]
    t = T[:, 0]; x = T[:, 1]; y = T[:, 2]
    yaw = np.unwrap(quat_to_yaw(T[:, 4:8]))             # unwrap 防 ±π 跳变后再插值
    q = np.asarray(query_t, float)
    xi = np.interp(q, t, x)
    yi = np.interp(q, t, y)
    yi_yaw = np.interp(q, t, yaw)
    out = np.stack([xi - xi[0], yi - yi[0], yi_yaw - yi_yaw[0]], axis=1)
    return out


def _disp_scale_anchor(pred_traj, lidar_traj, strides, min_disp=0.10):
    """frame-invariant **绝对** 相对位移幅值误差(scale 锚,不归一化、不饱和)。

    归一化 RPE 位置项在 pred 平移塌缩(vx→0)时会饱和到 ~1.0,诱发退化解(实测 fold 全塌缩)。
    本项直接比 |pred 子段位移| vs |lidar 子段位移| 的绝对差(米),按 lidar 位移幅值平方归一到
    无量纲但**不封顶**:塌缩越多惩罚越大且无上界 → 强约束 pred 保持正确平移尺度。
    仅在 lidar 位移 >= min_disp 的运动子段累加(静止段位移≈0,屏蔽)。"""
    total = 0.0; n = 0
    for st in strides:
        N = pred_traj.shape[-2]; M = N - st
        if M <= 0:
            continue
        px, py = pred_traj[..., 0], pred_traj[..., 1]
        lx, ly = lidar_traj[..., 0], lidar_traj[..., 1]
        pd = torch.sqrt((px[..., st:] - px[..., :M]) ** 2 + (py[..., st:] - py[..., :M]) ** 2 + 1e-12)
        ld = torch.sqrt((lx[..., st:] - lx[..., :M]) ** 2 + (ly[..., st:] - ly[..., :M]) ** 2 + 1e-12)
        mask = (ld >= min_disp).float()
        denom = mask.sum().clamp(min=1.0)
        # (|pred_disp| - |lidar_disp|)² / |lidar_disp|² —— 相对幅值误差平方,不封顶。
        frac = ((pd - ld) ** 2) / (ld ** 2 + min_disp ** 2)
        total = total + (frac * mask).sum() / denom
        n += 1
    if n == 0:
        return pred_traj.sum() * 0.0
    return total / n


def lidar_traj_loss(pred_twists, window_t, tum, strides=(50, 150, 350),
                    yaw_weight=3.0, min_disp=0.10, disp_weight=2.0):
    """端到端真机激光监督损失。

    pred_twists: (S,3) 或 (B,S,3) 可微 base-frame twist(模型 roll 输出,float)。
    window_t:    (S,)  对应窗末时戳(float64,绝对 epoch),与 pred_twists 第 -2 维对齐。
    tum:         激光 TUM (N,8)。
    disp_weight: scale 锚权重(见 _disp_scale_anchor)。设 0 退化为纯归一化 RPE(会诱发塌缩,不推荐)。
    流程:pred_twists --可微 SE2 积分--> pred 轨迹 (S,3);激光插值到 window_t --> lidar 目标 (S,3);
         loss = frame-invariant RPE(trajectory_rpe_loss)+ disp_weight·绝对位移尺度锚。
         返回标量 loss(可微回流到 pred_twists)。仅支持单段(B 维可选)。
    """
    if pred_twists.dim() == 2:
        pred_twists = pred_twists[None]              # (1,S,3)
        wt = window_t[None]
        squeeze = True
    else:
        wt = window_t
        squeeze = False
    from lwt.kinematics_torch import integrate_twist_torch
    # 用 float64 积分(长程累积数值稳定);integrate_twist_torch 内部会把 times 对齐到 twists dtype。
    pred_traj = integrate_twist_torch(wt.double(), pred_twists.double())
    # 激光目标(numpy → tensor),逐 batch 段相同 tum 但不同 window_t。
    qt = wt.detach().cpu().numpy()
    tgt = np.stack([lidar_poses_at_times(tum, qt[b]) for b in range(qt.shape[0])], axis=0)
    lidar_traj = torch.as_tensor(tgt, dtype=pred_traj.dtype, device=pred_traj.device)
    loss = trajectory_rpe_loss(pred_traj, lidar_traj, list(strides), yaw_weight=yaw_weight, min_disp=min_disp)
    if disp_weight > 0:
        loss = loss + disp_weight * _disp_scale_anchor(pred_traj, lidar_traj, list(strides), min_disp=min_disp)
    return loss
