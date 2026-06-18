# lwt/losses.py —— 逐轴加权 MSE(预热)+ 异方差高斯 NLL。w=逐轴权重(1/训练std)。
import torch


def weighted_mse(pred, target, w):
    return (((pred - target) ** 2) * w[None, :]).mean()


def gaussian_nll(pred, target, logvar, w):
    """0.5·mean( w·((e²/σ²) + logσ²) ),e=pred-target,σ²=exp(logvar)。"""
    inv = torch.exp(-logvar)
    return 0.5 * (w[None, :] * (((pred - target) ** 2) * inv + logvar)).mean()


def batch_bias_penalty(pred, target, weight, w):
    """惩罚整个 batch 的逐轴系统性偏差(均值误差),与 weighted_mse/gaussian_nll 同的逐轴权重 w。
    weight·Σ_axis( w_axis · (mean_batch(pred-target))² )。零偏→0,偏差平方→二次增长。"""
    mean_err = (pred - target).mean(dim=0)          # (3,) per-axis bias
    return weight * ((mean_err ** 2) * w).sum()


def trajectory_rpe_loss(pred_traj, gt_traj, strides, yaw_weight=10.0, min_disp=0.10):
    """长程轨迹相对位姿损失(approach-2 step1 核心)。

    pred_traj/gt_traj: (B, S, 3)=[x,y,yaw]。对每个子段跨度 stride(帧)计算相对位姿误差
    (旋入起点航向系的 Δxy 与包裹后的 Δyaw)。
    - 航向项(主):Δyaw 弧度均方,yaw_weight 加权,**所有**子段都计。Δyaw 对常值 wz
      偏置高度敏感(stride 越大越放大)→ 使积分航向漂移被损失直接看见。这是核心机制。
    - 位置项(辅):相对位移误差按 GT 位移幅值归一化(无量纲),但只在 GT 位移 >= min_disp
      的"运动中"子段计(慢速/静止段位移≈0,归一化会放大噪声 → 屏蔽,类 KITTI 只评有效段)。
    多 stride 平均 → 同时约束短程(局部一致)与长程(累积漂移)。
    """
    from lwt.kinematics_torch import relative_pose_error
    total = 0.0
    n = 0
    for st in strides:
        rel_pos, rel_yaw = relative_pose_error(pred_traj, gt_traj, st)  # (B,M,2),(B,M)
        if rel_pos.shape[-2] == 0:
            continue
        x, y = gt_traj[..., 0], gt_traj[..., 1]
        M = rel_pos.shape[-2]
        gdx = x[..., st:] - x[..., :M]
        gdy = y[..., st:] - y[..., :M]
        gmag = torch.sqrt(gdx ** 2 + gdy ** 2 + 1e-9)                 # (B,M) GT 位移幅值
        mask = (gmag >= min_disp).float()
        denom = mask.sum().clamp(min=1.0)
        # 归一化位置误差分数,仅在运动段累加。
        pos_frac = (rel_pos ** 2).sum(dim=-1) / (gmag ** 2 + min_disp ** 2)
        pos_term = (pos_frac * mask).sum() / denom
        yaw_term = (rel_yaw ** 2).mean()
        total = total + pos_term + yaw_weight * yaw_term
        n += 1
    if n == 0:
        return pred_traj.sum() * 0.0
    return total / n
