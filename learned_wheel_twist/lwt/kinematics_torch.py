# lwt/kinematics_torch.py —— 可微 SE2 twist 积分(torch),与 numpy integrate_twist 同语义。
# 用于 approach-2 step1:长程轨迹损失要求积分器端到端可微,梯度回流到逐帧 twist。
import torch


def integrate_twist_torch(times, twists):
    """中点法 SE2 积分(可微),与 numpy `kinematics.integrate_twist` 同语义。

    参数:
      times:  (..., N)        时间戳(单调递增,首帧为段起点)。
      twists: (..., N, 3)     base 系 twist (vx, vy, wz)。
    返回:
      traj:   (..., N, 3) = [x, y, yaw],首帧为原点 (0,0,0)。

    与 numpy 版逐位匹配的离散方式:第 i 步(i>=1)用 twists[i-1] 与 dt=t[i]-t[i-1],
    中点航向 ym = yaw + 0.5*wz*dt 旋转 base 位移后累加(前向欧拉 + 航向中点)。
    完全 torch 算子构造 → autograd 可对 twists(及 times)求导。
    """
    if not torch.is_tensor(twists):
        twists = torch.as_tensor(twists, dtype=torch.float64)
    if not torch.is_tensor(times):
        times = torch.as_tensor(times, dtype=twists.dtype, device=twists.device)
    times = times.to(twists.dtype)

    N = twists.shape[-2]
    # dt[k] = times[k+1]-times[k],对应用 twists[k] 推进到第 k+1 帧。 (..., N-1)
    dt = times[..., 1:] - times[..., :-1]
    vx = twists[..., :-1, 0]
    vy = twists[..., :-1, 1]
    wz = twists[..., :-1, 2]

    dyaw = wz * dt                                  # (..., N-1) 每步航向增量
    # yaw_before[k] = sum_{j<k} dyaw[j];中点航向 = yaw_before + 0.5*dyaw。
    yaw_before = torch.cumsum(dyaw, dim=-1) - dyaw  # 排他前缀和(首步=0)
    ym = yaw_before + 0.5 * dyaw

    cy = torch.cos(ym)
    sy = torch.sin(ym)
    dx = (vx * cy - vy * sy) * dt                   # (..., N-1)
    dy = (vx * sy + vy * cy) * dt

    x = torch.cumsum(dx, dim=-1)                    # (..., N-1) 第 1..N-1 帧位置
    y = torch.cumsum(dy, dim=-1)
    yaw = torch.cumsum(dyaw, dim=-1)

    # 前置首帧原点 (0,0,0)。
    zeros = torch.zeros(twists.shape[:-2] + (1,), dtype=twists.dtype, device=twists.device)
    x = torch.cat([zeros, x], dim=-1)
    y = torch.cat([zeros, y], dim=-1)
    yaw = torch.cat([zeros, yaw], dim=-1)
    return torch.stack([x, y, yaw], dim=-1)         # (..., N, 3)


def relative_pose_error(pred_traj, gt_traj, stride):
    """KITTI/RPE 风格相对位姿误差:对每个起点 i,取子段 i→i+stride 的相对位移
    (旋入 i 点本轨迹航向系) 与 gt 同口径相对位移比差。返回逐元素 (Δx,Δy,Δyaw) 误差。

    pred_traj/gt_traj: (..., N, 3)=[x,y,yaw]。stride: int 子段跨度(帧)。
    返回 (rel_pos_err (..., M, 2), rel_yaw_err (..., M))。M=N-stride。
    全 torch,可微。
    """
    N = pred_traj.shape[-2]
    M = N - stride
    if M <= 0:
        z = pred_traj[..., :0, :2]
        return z, z[..., 0]

    def rel(traj):
        x, y, yaw = traj[..., 0], traj[..., 1], traj[..., 2]
        xi, yi, yawi = x[..., :M], y[..., :M], yaw[..., :M]
        xj, yj, yawj = x[..., stride:], y[..., stride:], yaw[..., stride:]
        dx = xj - xi
        dy = yj - yi
        c = torch.cos(yawi)
        s = torch.sin(yawi)
        # 旋入 i 点航向系:R(-yawi) @ (dx,dy)
        lx = c * dx + s * dy
        ly = -s * dx + c * dy
        dyaw = yawj - yawi
        return lx, ly, dyaw

    plx, ply, pdyaw = rel(pred_traj)
    glx, gly, gdyaw = rel(gt_traj)
    rel_pos = torch.stack([plx - glx, ply - gly], dim=-1)        # (..., M, 2)
    # 航向误差包裹到 (-pi, pi]
    dyaw = pdyaw - gdyaw
    dyaw = torch.atan2(torch.sin(dyaw), torch.cos(dyaw))
    return rel_pos, dyaw
