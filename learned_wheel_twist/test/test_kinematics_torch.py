import sys, numpy as np, torch
sys.path.insert(0, "learned_wheel_twist")
from lwt.kinematics import integrate_twist
from lwt.kinematics_torch import integrate_twist_torch, relative_pose_error


def test_matches_numpy_constant_twist():
    """常值 twist:torch 积分逐位匹配 numpy integrate_twist (<1e-5)。"""
    t = np.arange(0, 10, 0.02)
    tw = np.zeros((len(t), 3)); tw[:, 0] = 1.0; tw[:, 2] = 0.2
    np_traj = integrate_twist(t, tw)[:, 1:]                       # (N,3)[x,y,yaw]
    tt = integrate_twist_torch(torch.as_tensor(t), torch.as_tensor(tw))
    np.testing.assert_allclose(tt.numpy(), np_traj, atol=1e-5)


def test_matches_numpy_varying_twist():
    """时变 twist(随机但平滑):torch == numpy。"""
    rng = np.random.default_rng(0)
    t = np.cumsum(np.full(300, 0.02)) - 0.02
    tw = np.zeros((300, 3))
    tw[:, 0] = 0.8 + 0.3 * np.sin(np.linspace(0, 6, 300))
    tw[:, 1] = 0.05 * np.cos(np.linspace(0, 4, 300))
    tw[:, 2] = 0.4 * np.sin(np.linspace(0, 8, 300)) + 0.01
    np_traj = integrate_twist(t, tw)[:, 1:]
    tt = integrate_twist_torch(torch.as_tensor(t), torch.as_tensor(tw))
    np.testing.assert_allclose(tt.numpy(), np_traj, atol=1e-5)


def test_batched():
    """批量 (B,N,3) 与逐个调用一致。"""
    rng = np.random.default_rng(1)
    t = np.cumsum(np.full(100, 0.02)) - 0.02
    tw = rng.normal(0, 0.3, (4, 100, 3))
    tb = torch.as_tensor(tw)
    out = integrate_twist_torch(torch.as_tensor(t), tb)
    assert out.shape == (4, 100, 3)
    for b in range(4):
        single = integrate_twist_torch(torch.as_tensor(t), torch.as_tensor(tw[b]))
        np.testing.assert_allclose(out[b].numpy(), single.numpy(), atol=1e-9)


def test_gradient_flows_to_twists():
    """对积分末端的 loss 能产生对输入 twists 的有限非零梯度(非退化工作点)。"""
    t = torch.arange(0, 5, 0.02, dtype=torch.float64)
    tw0 = torch.zeros((len(t), 3), dtype=torch.float64); tw0[:, 0] = 0.6; tw0[:, 2] = 0.15
    tw = tw0.clone().requires_grad_(True)
    traj = integrate_twist_torch(t, tw)
    target = torch.tensor([3.0, 1.0, 0.5], dtype=torch.float64)
    loss = ((traj[-1] - target) ** 2).sum()
    loss.backward()
    assert tw.grad is not None
    assert torch.isfinite(tw.grad).all()
    assert tw.grad.abs().sum() > 0  # 非零梯度


def test_gradient_wz_bias_affects_yaw():
    """核心机理:常值 wz 偏置会被末端 yaw loss 看到 → 对 wz 通道的梯度非零且主导。"""
    t = torch.arange(0, 10, 0.02, dtype=torch.float64)   # 长程 10s
    tw = torch.zeros((len(t), 3), dtype=torch.float64, requires_grad=True)
    traj = integrate_twist_torch(t, tw)
    yaw_end = traj[-1, 2]
    yaw_end.backward()
    g = tw.grad
    # d(yaw_end)/d(wz[k]) = dt 对每步,vx/vy 通道对 yaw 无直接贡献(在 0 twist 处)。
    assert g[:, 2].abs().sum() > 0
    np.testing.assert_allclose(g[:-1, 2].numpy(), 0.02, atol=1e-6)


def test_rpe_zero_on_identical():
    t = torch.arange(0, 10, 0.02, dtype=torch.float64)
    tw = torch.zeros((len(t), 3), dtype=torch.float64); tw[:, 0] = 0.5; tw[:, 2] = 0.1
    traj = integrate_twist_torch(t, tw)
    rp, ry = relative_pose_error(traj, traj, stride=50)
    assert rp.abs().max() < 1e-9
    assert ry.abs().max() < 1e-9


def test_rpe_detects_yaw_bias():
    """两条轨迹仅 wz 差一个常偏 → RPE 航向误差非零。"""
    t = torch.arange(0, 10, 0.02, dtype=torch.float64)
    g = torch.zeros((len(t), 3), dtype=torch.float64); g[:, 0] = 0.5; g[:, 2] = 0.1
    p = g.clone(); p[:, 2] += 0.01    # +0.01 rad/s wz 偏置
    tg = integrate_twist_torch(t, g); tp = integrate_twist_torch(t, p)
    rp, ry = relative_pose_error(tp, tg, stride=50)
    assert ry.abs().mean() > 1e-3
