import numpy as np
from w2_sim.degradation_node import joint_state_to_modules
from w2_sim.swerve_kinematics import WHEEL_NAMES, WHEEL_RADIUS


def test_joint_state_to_modules():
    # /sim/joint_states: 8 关节乱序也要正确映射;轮速 rad/s → m/s
    names = ["rear_left_wheel_joint", "front_left_steer_joint",
             "front_left_wheel_joint", "rear_left_steer_joint",
             "front_right_steer_joint", "front_right_wheel_joint",
             "rear_right_wheel_joint", "rear_right_steer_joint"]
    pos = dict(zip(names, [0.0, 0.1, 0.0, 0.3, 0.2, 0.0, 0.0, 0.4]))
    vel = dict(zip(names, [40.0, 0.0, 10.0, 0.0, 0.0, 20.0, 30.0, 0.0]))
    angles, speeds = joint_state_to_modules(
        names, [pos[n] for n in names], [vel[n] for n in names])
    np.testing.assert_allclose(angles, [0.1, 0.2, 0.3, 0.4])
    np.testing.assert_allclose(speeds, np.array([10, 20, 40, 30]) * WHEEL_RADIUS)
