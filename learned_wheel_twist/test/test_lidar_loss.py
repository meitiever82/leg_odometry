# test/test_lidar_loss.py —— 阶段六 Step B:真机激光监督轨迹损失 TDD。
# 验证:(a) pred==lidar 时损失=0;(b) 常值 wz 偏置被检测到且随段长增长;(c) 梯度回流到模型参数。
import sys, numpy as np, torch
sys.path.insert(0, "learned_wheel_twist")
from lwt.lidar_loss import quat_to_yaw, lidar_poses_at_times, lidar_traj_loss
from lwt.kinematics_torch import integrate_twist_torch
from lwt.losses import trajectory_rpe_loss


def test_quat_to_yaw_known_angles():
    # 绕 z 轴 90°:qz=sin(45°),qw=cos(45°) → yaw=90°。
    for deg in [-179.0, -90.0, 0.0, 45.0, 90.0, 170.0]:
        r = np.radians(deg)
        q = np.array([0.0, 0.0, np.sin(r / 2), np.cos(r / 2)])  # x,y,z,w
        y = quat_to_yaw(q[None])[0]
        assert abs(np.degrees(y) - deg) < 1e-6, (deg, np.degrees(y))


def _make_tum(times, xs, ys, yaws):
    """构造 TUM 数组 (N,8)=[t tx ty tz qx qy qz qw],平面运动(只用 z 四元数)。"""
    N = len(times)
    T = np.zeros((N, 8))
    T[:, 0] = times; T[:, 1] = xs; T[:, 2] = ys
    T[:, 6] = np.sin(np.asarray(yaws) / 2)   # qz
    T[:, 7] = np.cos(np.asarray(yaws) / 2)   # qw
    return T


def test_lidar_poses_interp_local_frame():
    # 直线 +x 运动,航向恒 0。query 时戳取中间 → 插值应落在直线上,首点平移到原点。
    times = np.linspace(0, 10, 11)
    T = _make_tum(times, xs=times * 0.5, ys=np.zeros(11), yaws=np.zeros(11))
    q = np.array([1.0, 3.0, 5.0, 7.0])
    poses = lidar_poses_at_times(T, q)            # (4,3) [x,y,yaw], 首点=原点
    assert poses.shape == (4, 3)
    np.testing.assert_allclose(poses[0], [0, 0, 0], atol=1e-9)
    # x 应 = 0.5*(t - t0)
    np.testing.assert_allclose(poses[:, 0], 0.5 * (q - q[0]), atol=1e-6)
    np.testing.assert_allclose(poses[:, 1], 0.0, atol=1e-9)


def test_loss_zero_when_pred_equals_lidar():
    # pred 轨迹 == lidar 目标 → frame-invariant RPE 损失 = 0。
    t = torch.arange(0, 10, 0.02, dtype=torch.float64)[None]   # (1,N)
    tw = torch.zeros((1, t.shape[1], 3), dtype=torch.float64); tw[..., 0] = 0.4; tw[..., 2] = 0.08
    pred = integrate_twist_torch(t, tw)            # (1,N,3)
    lidar = pred.clone().detach()
    loss = trajectory_rpe_loss(pred, lidar, strides=[50, 250], yaw_weight=3.0)
    assert loss.item() < 1e-12


def test_loss_detects_wz_bias_growing_with_segment_length():
    # pred 比 lidar 多一个常值 wz 偏置 → 损失 > 0,且长 stride 惩罚 > 短 stride。
    t = torch.arange(0, 12, 0.02, dtype=torch.float64)[None]
    g = torch.zeros((1, t.shape[1], 3), dtype=torch.float64); g[..., 0] = 0.4
    lidar = integrate_twist_torch(t, g)
    biased = g.clone(); biased[..., 2] += 0.01     # 常值 wz 偏置(模拟真机未修正偏置)
    pred = integrate_twist_torch(t, biased)
    short = trajectory_rpe_loss(pred, lidar, strides=[25], yaw_weight=3.0).item()
    long = trajectory_rpe_loss(pred, lidar, strides=[500], yaw_weight=3.0).item()
    assert long > short > 0


def test_lidar_traj_loss_end_to_end_zero_and_grad():
    # 端到端 lidar_traj_loss:窗末时戳 → 插值 lidar 目标 → 与 pred 轨迹比 RPE。
    # pred 由可微 twist 积分得到;loss 对 twist 可微,grad 回流且 wz 分量非零。
    times = np.linspace(100.0, 130.0, 31)          # 30s 直线+转弯参考(绝对 epoch 量级)
    yaws = np.linspace(0, 0.5, 31)
    xs = np.cumsum(np.cos(yaws)) * 0.3; ys = np.cumsum(np.sin(yaws)) * 0.3
    T = _make_tum(times, xs, ys, yaws)
    # 窗末时戳:在参考时间范围内均匀取 200 个点。
    wt = torch.tensor(np.linspace(101.0, 129.0, 200), dtype=torch.float64)
    # 一个带偏置的可微 twist(模拟模型输出)。
    pred_tw = torch.zeros((200, 3), dtype=torch.float64)
    pred_tw[:, 0] = 0.3; pred_tw[:, 2] = 0.02
    pred_tw.requires_grad_(True)
    loss = lidar_traj_loss(pred_tw, wt, T, strides=[50, 150], yaw_weight=3.0)
    assert loss.item() > 0
    loss.backward()
    assert pred_tw.grad is not None and torch.isfinite(pred_tw.grad).all()
    assert pred_tw.grad[:, 2].abs().sum() > 0       # 梯度回流到 wz


def test_lidar_traj_loss_disp_anchor_penalizes_scale_collapse():
    """scale 锚:压缩 pred 平移(vx 塌缩)必须被**单调强惩罚**(不饱和)。
    没有 disp_weight 时归一化 RPE 位置项会在 scale→0 处饱和(~1.0),诱发退化解。
    加上 disp_weight 后,loss 应随 scale 远离 1.0 单调增大且不封顶。"""
    times = np.linspace(0, 12, 13)
    yaws = np.linspace(0, 0.6, 13)
    xs = np.cumsum(np.cos(yaws)) * 0.5; ys = np.cumsum(np.sin(yaws)) * 0.5
    T = _make_tum(times, xs, ys, yaws)
    wt = torch.tensor(np.linspace(1.0, 11.0, 300), dtype=torch.float64)
    lidar = torch.as_tensor(lidar_poses_at_times(T, wt.numpy())[None], dtype=torch.float64)
    # 反推一组能复现 lidar 的 twist(用差分近似),再按 scale 缩放平移分量。
    from lwt.kinematics_torch import integrate_twist_torch
    # 直接构造缩放 twist:vx 恒定、wz 恒定逼近参考(够测试单调性)。
    losses = []
    for sc in [1.0, 0.5, 0.2, 0.05]:
        tw = torch.zeros((300, 3), dtype=torch.float64)
        tw[:, 0] = 0.45 * sc; tw[:, 2] = 0.05
        L = lidar_traj_loss(tw, wt, T, strides=[50, 150], yaw_weight=3.0, disp_weight=2.0).item()
        losses.append(L)
    # 单调:scale 越小 loss 越大,且 scale=0.05 处远超归一化饱和值(~1)。
    assert losses[0] < losses[1] < losses[2] < losses[3]
    assert losses[3] > 1.5    # 不饱和:塌缩被强惩罚(归一化项单独会封顶 ~1)


def test_lidar_traj_loss_disp_anchor_zero_on_match():
    # disp_weight 项在 pred==lidar 时仍为 0(不破坏完美匹配的零损失)。
    t = torch.arange(0, 10, 0.02, dtype=torch.float64)
    tw = torch.zeros((t.shape[0], 3), dtype=torch.float64); tw[:, 0] = 0.4; tw[:, 2] = 0.08
    pred = integrate_twist_torch(t[None], tw[None].double())
    times = np.linspace(float(t[0]), float(t[-1]), 60)
    pr = pred[0].detach().numpy()
    # 用 pred 自身做 lidar 目标(同轨迹)→ 任何权重下 loss=0。
    yaw = pr[:, 2]
    Ti = np.interp(times, t.numpy(), np.arange(len(t)))
    xs = np.interp(times, t.numpy(), pr[:, 0]); ys = np.interp(times, t.numpy(), pr[:, 1])
    yy = np.interp(times, t.numpy(), yaw)
    T = _make_tum(times, xs, ys, yy)
    loss = lidar_traj_loss(tw, t, T, strides=[50, 250], yaw_weight=3.0, disp_weight=2.0)
    assert loss.item() < 1e-6


def test_lidar_traj_loss_grad_flows_to_model_params():
    # grad 须能穿过 lidar_traj_loss 回到 nn 参数。
    import torch.nn as nn
    times = np.linspace(50.0, 70.0, 21)
    T = _make_tum(times, xs=np.linspace(0, 6, 21), ys=np.zeros(21), yaws=np.zeros(21))
    wt = torch.tensor(np.linspace(51.0, 69.0, 100), dtype=torch.float64)
    net = nn.Linear(4, 3).double()
    feats = torch.randn(100, 4, dtype=torch.float64)
    pred_tw = net(feats)                            # (100,3) 可微 twist
    loss = lidar_traj_loss(pred_tw, wt, T, strides=[40], yaw_weight=3.0)
    loss.backward()
    g = net.weight.grad
    assert g is not None and torch.isfinite(g).all() and g.abs().sum() > 0
