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
