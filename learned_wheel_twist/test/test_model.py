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


def test_data_driven_ignores_prior():
    # phase-3: prior='none' → 直接回归,ls_prior 不影响输出(twist 与 ls 无关)。
    m = TwistTCN(in_ch=14, channels=48, layers=4, kernel=3, prior="none"); m.eval()
    x = torch.randn(4, 14, 25)
    t0, _ = m(x, torch.zeros(4, 3))
    t1, _ = m(x, torch.randn(4, 3) * 100)   # 任意先验
    assert torch.allclose(t0, t1, atol=1e-6)   # prior 被忽略
    assert t0.shape == (4, 3)


def test_data_driven_zero_init_disabled():
    # prior='none' 下 zero_init_residual 不应把 head 清零(否则输出恒 0,学不动)。
    m = TwistTCN(in_ch=14, channels=48, layers=4, kernel=3, prior="none", zero_init_residual=True)
    assert not torch.allclose(m.head_delta.weight, torch.zeros_like(m.head_delta.weight))


def test_wz_anchor_zero_residual_equals_prior():
    # phase-4: prior='wz_anchor' + zero_init → 初始残差 0,twist 须恒等于先验 [0,0,gyro]。
    m = TwistTCN(in_ch=14, channels=48, layers=4, kernel=3, prior="wz_anchor",
                 zero_init_residual=True); m.eval()
    x = torch.randn(4, 14, 25)
    prior = torch.zeros(4, 3); prior[:, 2] = torch.tensor([0.1, -0.2, 0.05, 0.0])  # [0,0,gyro]
    twist, _ = m(x, prior)
    assert torch.allclose(twist, prior, atol=1e-5)   # 残差 0 → twist == 先验


def test_wz_anchor_zero_init_applied():
    # wz_anchor 是 non-"none" prior,zero_init_residual 须把 head_delta 清零。
    m = TwistTCN(in_ch=14, channels=48, layers=4, kernel=3, prior="wz_anchor",
                 zero_init_residual=True)
    assert torch.allclose(m.head_delta.weight, torch.zeros_like(m.head_delta.weight))
    assert torch.allclose(m.head_delta.bias, torch.zeros_like(m.head_delta.bias))
