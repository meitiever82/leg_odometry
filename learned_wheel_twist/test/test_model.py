import sys, torch
sys.path.insert(0, "learned_wheel_twist")
from lwt.model import TwistTCN

def test_forward_shapes():
    m = TwistTCN(in_ch=8, channels=48, layers=4, kernel=3)
    x = torch.randn(16, 8, 25); ls = torch.randn(16, 3)
    twist, logvar = m(x, ls)
    assert twist.shape == (16, 3) and logvar.shape == (16, 3)

def test_residual_on_ls():
    m = TwistTCN(in_ch=8, channels=48, layers=4, kernel=3, zero_init_residual=True)
    x = torch.randn(4, 8, 25); ls = torch.randn(4, 3)
    twist, _ = m(x, ls)
    assert torch.allclose(twist, ls, atol=1e-5)

def test_imu_variant_in_ch():
    # phase-2: 9-ch = 8 wheel + 1 imu_yawrate
    m = TwistTCN(in_ch=9, channels=48, layers=4, kernel=3)
    x = torch.randn(2, 9, 25); ls = torch.randn(2, 3)
    twist, logvar = m(x, ls)
    assert twist.shape == (2, 3)
