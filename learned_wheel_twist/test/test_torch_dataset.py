import sys, numpy as np, tempfile, os, torch
sys.path.insert(0, "learned_wheel_twist")
from lwt.dataset import TwistDataset

def _mk(td, name, n=80, kappa=0.005):
    rng = np.random.default_rng(abs(hash(name)) % 2**31)
    np.savez(os.path.join(td, name), t_wheel=np.arange(n)*0.02,
             steer=rng.normal(0, 0.1, (n, 4)), speed=rng.uniform(0.2, 1.0, (n, 4)),
             gt_vx=rng.normal(0.5, 0.1, n), gt_vy=np.zeros(n), gt_wz=rng.normal(0, 0.1, n),
             imu_yawrate=rng.normal(0, 0.1, (n, 1)), kappa_theory=kappa)

def test_dataset_item_shapes():
    with tempfile.TemporaryDirectory() as td:
        for i in range(3): _mk(td, f"e{i}.npz")
        ds = TwistDataset([os.path.join(td, f"e{i}.npz") for i in range(3)],
                          window=25, augment=True, with_imu=False)
        x, y, ls = ds[0]
        assert x.shape == (8, 25)
        assert y.shape == (3,) and ls.shape == (3,)
        assert isinstance(x, torch.Tensor)

def test_dataset_with_imu_9ch():
    # phase-2: with_imu → 9 通道(8 wheel + 1 imu_yawrate),增广只动前 8 维。
    with tempfile.TemporaryDirectory() as td:
        _mk(td, "e.npz")
        ds = TwistDataset([os.path.join(td, "e.npz")], window=25, augment=True, with_imu=True)
        x, y, ls = ds[0]
        assert x.shape == (9, 25)
        assert y.shape == (3,) and ls.shape == (3,)


def test_dataset_imu_wz_prior():
    # phase-2b: imu_wz_prior=True + with_imu → ls_prior = [ls_vx, ls_vy, imu_yawrate(窗末)].
    # 先验 wz 用窗末重力投影 yaw-rate(未标准化),vx/vy 仍用 LS。
    from lwt.kinematics import ls_twist
    with tempfile.TemporaryDirectory() as td:
        _mk(td, "e.npz")
        # 不增广 + 不标准化:可精确对照窗末原始值。
        ds = TwistDataset([os.path.join(td, "e.npz")], window=25, augment=False,
                          with_imu=True, imu_wz_prior=True)
        d = np.load(os.path.join(td, "e.npz"))
        i = 7
        x, y, ls = ds[i]
        w_end = i + 25 - 1   # 窗末原始帧索引
        ls_ref = ls_twist(d["steer"][w_end], d["speed"][w_end])
        # vx/vy 来自 LS
        np.testing.assert_allclose(ls[0].item(), ls_ref[0], rtol=1e-5)
        np.testing.assert_allclose(ls[1].item(), ls_ref[1], rtol=1e-5)
        # wz 来自窗末 imu_yawrate(而非 LS wz)
        np.testing.assert_allclose(ls[2].item(), d["imu_yawrate"][w_end, 0], rtol=1e-5)
        assert abs(ls[2].item() - ls_ref[2]) > 1e-9  # 与 LS wz 确实不同


def test_dataset_imu_wz_prior_requires_imu():
    # imu_wz_prior 仅在 with_imu 时生效;wheel-only(in_ch=8)路径不受影响。
    from lwt.kinematics import ls_twist
    with tempfile.TemporaryDirectory() as td:
        _mk(td, "e.npz")
        ds = TwistDataset([os.path.join(td, "e.npz")], window=25, augment=False,
                          with_imu=False, imu_wz_prior=True)
        d = np.load(os.path.join(td, "e.npz"))
        i = 3
        _, _, ls = ds[i]
        ls_ref = ls_twist(d["steer"][i+24], d["speed"][i+24])
        np.testing.assert_allclose(ls[2].item(), ls_ref[2], rtol=1e-5)  # 仍是 LS wz


def test_dataset_no_aug_deterministic():
    with tempfile.TemporaryDirectory() as td:
        _mk(td, "e.npz")
        ds = TwistDataset([os.path.join(td, "e.npz")], window=25, augment=False, with_imu=False)
        x1, _, _ = ds[5]; x2, _, _ = ds[5]
        assert torch.allclose(x1, x2)
