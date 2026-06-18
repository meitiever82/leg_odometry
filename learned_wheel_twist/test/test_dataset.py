import sys, numpy as np, tempfile, os
sys.path.insert(0, "learned_wheel_twist")
from lwt.dataset import make_windows, stratified_split

def _fake_npz(path, n=200, kappa=0.005):
    rng = np.random.default_rng(0)
    np.savez(path, t_wheel=np.arange(n)*0.02,
             steer=rng.normal(0, 0.1, (n, 4)), speed=rng.uniform(0.2, 1.0, (n, 4)),
             gt_vx=rng.normal(0.5, 0.1, n), gt_vy=np.zeros(n), gt_wz=rng.normal(0, 0.1, n),
             kappa_theory=kappa)

def test_make_windows_shapes():
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, "e.npz"); _fake_npz(p, n=100)
        X, Y, YR = make_windows(p, window=25)
        assert X.shape[0] == 100 - 25 + 1 and X.shape[1:] == (25, 8)
        assert Y.shape == (X.shape[0], 3)
        assert YR.shape == (X.shape[0],)

def test_make_windows_label_is_window_end():
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, "e.npz"); _fake_npz(p, n=50)
        d = np.load(p); X, Y, YR = make_windows(p, window=25)
        np.testing.assert_allclose(Y[0], [d["gt_vx"][24], d["gt_vy"][24], d["gt_wz"][24]])

def test_stratified_split_by_episode():
    eps = [(f"e{i}.npz", -0.01 + 0.0008*i) for i in range(40)]
    tr, va, te = stratified_split(eps, ratios=(0.8, 0.1, 0.1), seed=0)
    names = lambda L: {n for n, _ in L}
    assert len(tr)+len(va)+len(te) == 40
    assert not (names(tr) & names(va)) and not (names(tr) & names(te)) and not (names(va) & names(te))
    allk = [k for _, k in eps]; tek = [k for _, k in te]
    assert min(tek) < np.percentile(allk, 40) and max(tek) > np.percentile(allk, 60)
