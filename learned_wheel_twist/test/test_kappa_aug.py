import sys, numpy as np
sys.path.insert(0, "learned_wheel_twist")
from lwt.dataset import augment_kappa
from lwt.kinematics import ls_twist

def test_aug_preserves_label_and_shifts_kappa():
    rng = np.random.default_rng(0)
    win = np.zeros((25, 8)); win[:, 4:] = 1.0
    aug, = [augment_kappa(win.copy(), steer_bias_max_deg=1.0, speed_scale_std=0.015, rng=rng)]
    assert not np.allclose(aug[:, :4], 0.0) or not np.allclose(aug[:, 4:], 1.0)
    expected = np.tile(aug[0, :4] - win[0, :4], (len(aug), 1))
    np.testing.assert_allclose(aug[:, :4] - win[:, :4], expected, atol=1e-9)

def test_aug_kappa_covers_real_range():
    rng = np.random.default_rng(1); ks = []
    for _ in range(2000):
        win = np.zeros((1, 8)); win[:, 4:] = 1.0
        aug = augment_kappa(win.copy(), 1.0, 0.015, rng)
        vx, vy, wz = ls_twist(aug[0, :4], aug[0, 4:])
        ks.append(wz/vx)
    ks = np.array(ks)
    assert ks.max() > 0.04 and ks.min() < -0.04, f"κ 覆盖不足: [{ks.min():.3f},{ks.max():.3f}]"
