import sys, glob, numpy as np, torch
from pathlib import Path
sys.path.insert(0, "learned_wheel_twist")
from lwt.dataset import make_segments, SegmentDataset
from lwt.kinematics import integrate_twist
from lwt.kinematics_torch import integrate_twist_torch

CACHE = Path("~/Documents/Datasets/w2_sim_cache").expanduser()
EPS = sorted(glob.glob(str(CACHE / "*" / "*.npz")))


def test_make_segments_shapes():
    if not EPS:
        import pytest; pytest.skip("no sim cache")
    segs = make_segments(EPS[0], window=25, segment_sec=10.0, wheel_hz=50)
    assert len(segs) > 0
    S = int(round(10.0 * 50))
    s = segs[0]
    assert s["X"].shape == (S, 25, 14)   # data_driven 14ch
    assert s["t"].shape == (S,)
    assert s["G"].shape == (S, 3)
    # 时间戳单调递增
    assert np.all(np.diff(s["t"]) > 0)


def test_segment_dataset_item():
    if not EPS:
        import pytest; pytest.skip("no sim cache")
    ds = SegmentDataset([EPS[0]], window=25, segment_sec=10.0)
    assert len(ds) > 0
    x, t, g, yr = ds[0]
    S = int(round(10.0 * 50))
    assert x.shape == (S, 14, 25)        # (S, C, window)
    assert t.shape == (S,) and g.shape == (S, 3)


def test_segment_gt_integration_matches_numpy():
    """段内 GT twist 经 torch 积分 == numpy 积分(确保段口径一致)。"""
    if not EPS:
        import pytest; pytest.skip("no sim cache")
    segs = make_segments(EPS[0], window=25, segment_sec=10.0)
    s = segs[0]
    np_traj = integrate_twist(s["t"], s["G"])[:, 1:]
    tt = integrate_twist_torch(torch.as_tensor(s["t"]), torch.as_tensor(s["G"], dtype=torch.float64))
    np.testing.assert_allclose(tt.numpy(), np_traj, atol=1e-5)
