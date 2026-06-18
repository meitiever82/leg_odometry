import sys, torch
sys.path.insert(0, "learned_wheel_twist")
from lwt.losses import weighted_mse, gaussian_nll, batch_bias_penalty, trajectory_rpe_loss
from lwt.kinematics_torch import integrate_twist_torch

def test_weighted_mse_zero():
    p = torch.zeros(8, 3); t = torch.zeros(8, 3)
    assert weighted_mse(p, t, torch.ones(3)).item() == 0.0

def test_nll_lower_when_confident_correct():
    t = torch.zeros(4, 3); p = torch.zeros(4, 3)
    nll_small = gaussian_nll(p, t, torch.full((4, 3), -4.0), torch.ones(3))
    nll_big = gaussian_nll(p, t, torch.full((4, 3), 2.0), torch.ones(3))
    assert nll_small < nll_big

def test_nll_penalizes_overconfident_wrong():
    t = torch.zeros(4, 3); p = torch.ones(4, 3)
    nll_conf = gaussian_nll(p, t, torch.full((4, 3), -4.0), torch.ones(3))
    nll_humble = gaussian_nll(p, t, torch.full((4, 3), 2.0), torch.ones(3))
    assert nll_conf > nll_humble


def test_bias_penalty_zero_when_no_offset():
    # zero-mean residual (errors cancel across batch) -> ~0 penalty.
    t = torch.zeros(6, 3)
    p = torch.tensor([[1., 1., 1.], [-1., -1., -1.]] * 3)  # mean residual = 0 per axis
    assert batch_bias_penalty(p, t, 1.0, torch.ones(3)).item() < 1e-12


def test_bias_penalty_positive_and_scales_with_offset_squared():
    g = torch.randn(32, 3)
    p1 = g + 0.1   # constant per-axis offset 0.1
    p2 = g + 0.2   # double the offset
    w = torch.ones(3)
    b1 = batch_bias_penalty(p1, g, 1.0, w)
    b2 = batch_bias_penalty(p2, g, 1.0, w)
    assert b1.item() > 0
    # penalty ~ offset^2 -> doubling offset quadruples penalty
    assert abs(b2.item() / b1.item() - 4.0) < 1e-3


def test_bias_penalty_scales_with_weight():
    g = torch.zeros(8, 3)
    p = g + 0.3
    w = torch.ones(3)
    b1 = batch_bias_penalty(p, g, 1.0, w)
    b2 = batch_bias_penalty(p, g, 2.5, w)
    assert abs(b2.item() / b1.item() - 2.5) < 1e-5


def test_traj_loss_zero_on_identical():
    t = torch.arange(0, 10, 0.02, dtype=torch.float64)[None]   # (1,N)
    tw = torch.zeros((1, len(t[0]), 3), dtype=torch.float64); tw[..., 0] = 0.5; tw[..., 2] = 0.1
    tr = integrate_twist_torch(t, tw)
    assert trajectory_rpe_loss(tr, tr, strides=[50, 250]).item() < 1e-12


def test_traj_loss_penalizes_wz_bias_more_at_long_stride():
    """长程子段对 wz 直流偏置的惩罚应大于短程 → 这是 approach-2 的核心机制。"""
    t = torch.arange(0, 10, 0.02, dtype=torch.float64)[None]
    g = torch.zeros((1, len(t[0]), 3), dtype=torch.float64); g[..., 0] = 0.5
    p = g.clone(); p[..., 2] += 0.01     # 常值 wz 偏置
    tg = integrate_twist_torch(t, g); tp = integrate_twist_torch(t, p)
    short = trajectory_rpe_loss(tp, tg, strides=[25]).item()
    long = trajectory_rpe_loss(tp, tg, strides=[450]).item()
    assert long > short > 0


def test_traj_loss_grad_flows_to_twist():
    t = torch.arange(0, 8, 0.02, dtype=torch.float64)[None]
    g = torch.zeros((1, len(t[0]), 3), dtype=torch.float64); g[..., 0] = 0.5; g[..., 2] = 0.05
    pred_tw = g.clone() + 0.02
    pred_tw.requires_grad_(True)
    tp = integrate_twist_torch(t, pred_tw); tg = integrate_twist_torch(t, g.detach())
    loss = trajectory_rpe_loss(tp, tg, strides=[50, 200])
    loss.backward()
    assert pred_tw.grad is not None and torch.isfinite(pred_tw.grad).all()
    assert pred_tw.grad[..., 2].abs().sum() > 0   # 梯度回流到 wz
