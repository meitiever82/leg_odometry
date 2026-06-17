import sys, torch
sys.path.insert(0, "learned_wheel_twist")
from lwt.losses import weighted_mse, gaussian_nll, batch_bias_penalty

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
