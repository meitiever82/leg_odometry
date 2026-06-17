import sys, torch
sys.path.insert(0, "learned_wheel_twist")
from lwt.losses import weighted_mse, gaussian_nll

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
