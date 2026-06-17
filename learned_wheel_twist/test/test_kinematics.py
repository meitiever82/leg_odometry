import sys, numpy as np
sys.path.insert(0, "learned_wheel_twist")
from lwt.kinematics import ls_twist, integrate_twist, ape_percent, rpe_segment

def test_ls_twist_pure_translation():
    ang = np.zeros(4); spd = np.ones(4)
    vx, vy, wz = ls_twist(ang, spd)
    np.testing.assert_allclose([vx, vy, wz], [1, 0, 0], atol=1e-9)

def test_ls_twist_batch():
    ang = np.zeros((5, 4)); spd = np.ones((5, 4))
    tw = ls_twist(ang, spd)
    assert tw.shape == (5, 3)
    np.testing.assert_allclose(tw[:, 0], 1.0, atol=1e-9)

def test_integrate_twist_straight():
    # np.arange(0, 5, 0.02) ends at 4.98 (249 intervals × 0.02 s × 1 m/s = 4.98 m).
    # atol=0.02 accepts ±1 dt of discretisation; y must be exactly 0.
    t = np.arange(0, 5, 0.02)
    tw = np.zeros((len(t), 3)); tw[:, 0] = 1.0
    traj = integrate_twist(t, tw)
    np.testing.assert_allclose(traj[-1, 1], t[-1], atol=1e-9)   # x == last timestamp
    np.testing.assert_allclose(traj[-1, 2], 0.0,   atol=1e-9)   # y == 0

def test_integrate_twist_arc():
    w = np.pi/10; t = np.arange(0, 5, 0.02)
    tw = np.zeros((len(t), 3)); tw[:, 0] = 1.0; tw[:, 2] = w
    traj = integrate_twist(t, tw)
    R = 1.0/w
    np.testing.assert_allclose(traj[-1, 1:3], [R, R], atol=0.02)

def test_ape_zero_identical():
    t = np.arange(0, 5, 0.02); tw = np.zeros((len(t), 3)); tw[:, 0] = 1.0
    traj = integrate_twist(t, tw)
    assert ape_percent(traj, traj[:, :3]) < 1e-6

def test_rpe_zero_identical():
    t = np.arange(0, 10, 0.02); tw = np.zeros((len(t), 3)); tw[:, 0] = 0.5
    traj = integrate_twist(t, tw)
    assert rpe_segment(traj, traj[:, :3], seg_len=2.0) < 1e-6

def _gentle_arc_truth(n=120):
    """Build a (n,3) truth [t,x,y] as a gentle curving path (heading varies)."""
    t = np.linspace(0, 60, n)
    ds = (t[1] - t[0]) * 0.5            # ~0.25 m per step
    heading = 0.3 * np.sin(np.linspace(0, 2*np.pi, n))  # gently curving heading
    dx = ds * np.cos(heading); dy = ds * np.sin(heading)
    x = np.concatenate([[0.0], np.cumsum(dx[:-1])])
    y = np.concatenate([[0.0], np.cumsum(dy[:-1])])
    return np.column_stack([t, x, y])

def _rot(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])

def test_rpe_invariant_to_global_rotation():
    truth = _gentle_arc_truth()
    # rigid transform: rotate (x,y) by 1.0 rad about the first point + translate
    p0 = truth[0, 1:3]
    R = _rot(1.0)
    xy = (truth[:, 1:3] - p0) @ R.T + p0 + np.array([10.0, -5.0])
    yaw = np.zeros((len(truth), 1))
    traj = np.column_stack([truth[:, 0], xy, yaw])   # (N,4)
    assert rpe_segment(traj, truth, 10.0) < 1.0

def test_rpe_identity_zero():
    truth = _gentle_arc_truth()
    yaw = np.zeros((len(truth), 1))
    traj = np.column_stack([truth[:, 0], truth[:, 1:3], yaw])  # (N,4)
    assert rpe_segment(traj, truth, 10.0) < 1e-6

def test_rpe_detects_scale_drift():
    truth = _gentle_arc_truth()
    p0 = truth[0, 1:3]
    xy = (truth[:, 1:3] - p0) * 1.1 + p0   # genuine shape drift (not rigid)
    yaw = np.zeros((len(truth), 1))
    traj = np.column_stack([truth[:, 0], xy, yaw])
    assert rpe_segment(traj, truth, 10.0) > 5.0
