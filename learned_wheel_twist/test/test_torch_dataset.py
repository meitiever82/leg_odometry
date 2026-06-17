import sys, numpy as np, tempfile, os, torch
sys.path.insert(0, "learned_wheel_twist")
from lwt.dataset import TwistDataset

def _mk(td, name, n=80, kappa=0.005):
    rng = np.random.default_rng(abs(hash(name)) % 2**31)
    np.savez(os.path.join(td, name), t_wheel=np.arange(n)*0.02,
             steer=rng.normal(0, 0.1, (n, 4)), speed=rng.uniform(0.2, 1.0, (n, 4)),
             gt_vx=rng.normal(0.5, 0.1, n), gt_vy=np.zeros(n), gt_wz=rng.normal(0, 0.1, n), kappa_theory=kappa)

def test_dataset_item_shapes():
    with tempfile.TemporaryDirectory() as td:
        for i in range(3): _mk(td, f"e{i}.npz")
        ds = TwistDataset([os.path.join(td, f"e{i}.npz") for i in range(3)],
                          window=25, augment=True, with_imu=False)
        x, y, ls = ds[0]
        assert x.shape == (8, 25)
        assert y.shape == (3,) and ls.shape == (3,)
        assert isinstance(x, torch.Tensor)

def test_dataset_no_aug_deterministic():
    with tempfile.TemporaryDirectory() as td:
        _mk(td, "e.npz")
        ds = TwistDataset([os.path.join(td, "e.npz")], window=25, augment=False, with_imu=False)
        x1, _, _ = ds[5]; x2, _, _ = ds[5]
        assert torch.allclose(x1, x2)
