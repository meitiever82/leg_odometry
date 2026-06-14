import numpy as np
from w2_sim.swerve_kinematics import inverse_kinematics
from w2_sim.wheel_integration import integrate_wheel, ape_percent


def _make_series(twists, dt=0.02):
    """twists: [(vx,vy,wz), ...] 每条持续 1s → (times, angles_t, speeds_t) 序列"""
    times, angs, spds = [], [], []
    t, prev = 0.0, np.zeros(4)
    for vx, vy, wz in twists:
        for _ in range(int(1.0 / dt)):
            a, s = inverse_kinematics(vx, vy, wz, prev_angles=prev)
            prev = a
            times.append(t); angs.append(a); spds.append(s)
            t += dt
    return np.array(times), np.array(angs), np.array(spds)


def test_straight_line():
    times, angs, spds = _make_series([(1.0, 0.0, 0.0)] * 5)
    traj = integrate_wheel(times, angs, spds)
    np.testing.assert_allclose(traj[-1, 1:3], [5.0, 0.0], atol=1e-6)


def test_quarter_turn_arc():
    # v=1, w=π/10,5s 转 90°:终点 (R, R), R = v/w
    w = np.pi / 10
    times, angs, spds = _make_series([(1.0, 0.0, w)] * 5)
    traj = integrate_wheel(times, angs, spds)
    R = 1.0 / w
    np.testing.assert_allclose(traj[-1, 1:3], [R, R], atol=0.01)
    np.testing.assert_allclose(traj[-1, 3], np.pi / 2, atol=1e-3)


def test_ape_zero_for_identical():
    times, angs, spds = _make_series([(1.0, 0.0, 0.2)] * 5)
    traj = integrate_wheel(times, angs, spds)
    truth = traj[:, :3]   # t, x, y
    assert ape_percent(traj, truth) < 1e-6
