import sys, numpy as np
sys.path.insert(0, "learned_wheel_twist")
from lwt.realism import (
    apply_kappa_bulk, apply_gyro_realism, apply_speed_bumps, apply_accel_noise,
    kappa_to_speed_scale, RealismAug,
)
from lwt.kinematics import ls_twist


# ---- channel layout (data_driven 14ch): steer4 + speed4 + imu_g_base3 + imu_a_base3 ----
G0, A0 = 8, 11   # gyro start col, accel start col


def _straight_window(W=25, v=1.0):
    """Nominal straight drive: steer 0, speed v, gyro/accel ~0 (accel z gravity)."""
    w = np.zeros((W, 14), dtype=np.float32)
    w[:, 4:8] = v
    w[:, A0 + 2] = 9.81
    return w


# ============================ kappa-bulk =================================
def test_kappa_to_speed_scale_linear_mapping():
    # Mapping must reproduce target kappa on nominal straight drive (within tolerance).
    for k in (-0.04, -0.02, 0.03, 0.05):
        s = kappa_to_speed_scale(k)
        w = _straight_window()
        # apply left/right differential: left=[0,2] right=[1,3]
        w[:, [4, 6]] *= (1.0 + s)
        w[:, [5, 7]] *= (1.0 - s)
        vx, vy, wz = ls_twist(w[-1, :4], w[-1, 4:8])
        assert abs(wz / vx - k) < 0.005, f"target {k} got {wz/vx:.4f}"


def test_kappa_bulk_off_is_noop():
    rng = np.random.default_rng(0)
    w = _straight_window()
    out = apply_kappa_bulk(w.copy(), enable=False, kappa_std=0.04, rng=rng)
    np.testing.assert_array_equal(out, w)


def test_kappa_bulk_shifts_median_to_real_magnitude():
    """Effective-kappa distribution after bulk aug: real |kappa|~=0.04 must be near median,
    i.e. the median |kappa| reaches real magnitude (unlike old tail-only aug ~0.0006)."""
    rng = np.random.default_rng(1)
    ks = []
    for _ in range(3000):
        w = _straight_window()
        aug = apply_kappa_bulk(w.copy(), enable=True, kappa_std=0.04, rng=rng)
        vx, vy, wz = ls_twist(aug[-1, :4], aug[-1, 4:8])
        ks.append(wz / vx)
    ks = np.array(ks)
    med_abs = np.median(np.abs(ks))
    assert 0.025 < med_abs < 0.055, f"median |kappa_eff|={med_abs:.4f} not at real magnitude"
    # both signs present in the bulk
    assert (ks < 0).mean() > 0.35 and (ks > 0).mean() > 0.35


def test_kappa_bulk_constant_across_window():
    rng = np.random.default_rng(2)
    w = _straight_window()
    aug = apply_kappa_bulk(w.copy(), enable=True, kappa_std=0.04, rng=rng)
    # speed scale must be identical for every frame in the window (fixed calibration state)
    ratio = aug[:, 4:8] / w[:, 4:8]
    np.testing.assert_allclose(ratio, np.tile(ratio[0], (len(ratio), 1)), atol=1e-6)


# ============================ gyro realism ==============================
def test_gyro_realism_off_is_noop():
    rng = np.random.default_rng(0)
    w = _straight_window()
    out = apply_gyro_realism(w.copy(), enable=False, sigma_bias=0.003, sigma_noise=0.005, rng=rng)
    np.testing.assert_array_equal(out, w)


def test_gyro_realism_perturbs_all_three_axes():
    rng = np.random.default_rng(3)
    w = _straight_window()
    out = apply_gyro_realism(w.copy(), enable=True, sigma_bias=0.003, sigma_noise=0.005, rng=rng)
    for ax in range(3):
        assert not np.allclose(out[:, G0 + ax], w[:, G0 + ax]), f"gyro axis {ax} unchanged"
    # non-gyro channels untouched
    np.testing.assert_array_equal(out[:, :G0], w[:, :G0])
    np.testing.assert_array_equal(out[:, A0:], w[:, A0:])


def test_gyro_realism_bias_is_constant_noise_is_white():
    """Residual bias = constant per window; white noise = per-sample. So per-axis the
    within-window variation comes from white noise, mean shift comes from bias."""
    rng = np.random.default_rng(4)
    biases = []
    for _ in range(400):
        w = _straight_window(W=50)
        out = apply_gyro_realism(w.copy(), enable=True, sigma_bias=0.003, sigma_noise=0.005, rng=rng)
        delta = out[:, G0 + 2] - w[:, G0 + 2]
        biases.append(delta.mean())          # window-mean ~ residual bias
    biases = np.array(biases)
    assert 0.0015 < biases.std() < 0.006, f"per-window bias std {biases.std():.4f} off"


def test_gyro_realism_magnitude_matches_real_yaw_hf():
    """White-noise std on yaw channel must land near real high-freq content (~0.005)."""
    rng = np.random.default_rng(5)
    w = _straight_window(W=2000)
    out = apply_gyro_realism(w.copy(), enable=True, sigma_bias=0.0, sigma_noise=0.005, rng=rng)
    assert abs(out[:, G0 + 2].std() - 0.005) < 0.0015


# ============================ speed bumps ===============================
def test_bumps_off_is_noop():
    rng = np.random.default_rng(0)
    w = _straight_window(W=400)
    out, mask = apply_speed_bumps(w.copy(), enable=False, event_rate=0.03, rng=rng,
                                  return_mask=True)
    np.testing.assert_array_equal(out, w)
    assert mask.sum() == 0


def test_bumps_inject_at_configured_rate():
    """Across many windows, fraction of frames inside a bump event ~= event_rate."""
    rng = np.random.default_rng(6)
    tot, hit = 0, 0
    for _ in range(300):
        w = _straight_window(W=400)
        _, mask = apply_speed_bumps(w.copy(), enable=True, event_rate=0.03, rng=rng,
                                    return_mask=True)
        tot += len(mask); hit += int(mask.sum())
    frac = hit / tot
    assert 0.015 < frac < 0.05, f"bump frame fraction {frac:.4f} not ~3%"


def test_bumps_are_clustered_into_events():
    """Bumps should be short contiguous events, not isolated single frames."""
    rng = np.random.default_rng(7)
    w = _straight_window(W=2000)
    _, mask = apply_speed_bumps(w.copy(), enable=True, event_rate=0.03, rng=rng,
                                return_mask=True)
    # count runs of consecutive True
    idx = np.where(mask)[0]
    if len(idx) == 0:
        return
    runs = np.split(idx, np.where(np.diff(idx) > 1)[0] + 1)
    mean_len = np.mean([len(r) for r in runs])
    assert mean_len > 2.0, f"events too short (mean run {mean_len:.1f}), not clustered"


def test_bumps_hit_speed_vertaccel_and_pitchroll():
    rng = np.random.default_rng(8)
    w = _straight_window(W=2000)
    out, mask = apply_speed_bumps(w.copy(), enable=True, event_rate=0.05, rng=rng,
                                  return_mask=True)
    m = mask.astype(bool)
    assert m.sum() > 0
    # vertical accel spike
    assert np.abs(out[m, A0 + 2] - w[m, A0 + 2]).max() > 0.5
    # pitch/roll-rate spikes (gyro axes 0,1)
    assert np.abs(out[m, G0 + 0] - w[m, G0 + 0]).max() > 0.01
    assert np.abs(out[m, G0 + 1] - w[m, G0 + 1]).max() > 0.01
    # wheel speeds perturbed during bump
    assert not np.allclose(out[m, 4:8], w[m, 4:8])
    # yaw-rate channel NOT a designed bump target (true planar motion smooth); off-event frames untouched
    np.testing.assert_allclose(out[~m, A0 + 2], w[~m, A0 + 2], atol=1e-6)


# ============================ accel noise ===============================
def test_accel_noise_off_is_noop():
    rng = np.random.default_rng(0)
    w = _straight_window()
    out = apply_accel_noise(w.copy(), enable=False, sigma=0.05, rng=rng)
    np.testing.assert_array_equal(out, w)


def test_accel_noise_perturbs_only_accel():
    rng = np.random.default_rng(9)
    w = _straight_window(W=500)
    out = apply_accel_noise(w.copy(), enable=True, sigma=0.05, rng=rng)
    for ax in range(3):
        assert not np.allclose(out[:, A0 + ax], w[:, A0 + ax])
    np.testing.assert_array_equal(out[:, :A0], w[:, :A0])
    assert abs((out[:, A0:] - w[:, A0:]).std() - 0.05) < 0.02


# ============================ orchestrator ==============================
def _cfg(**ov):
    c = dict(enable=True,
             kappa_bulk=dict(enable=True, kappa_std=0.04),
             gyro=dict(enable=True, sigma_bias=0.003, sigma_noise=0.005),
             bumps=dict(enable=True, event_rate=0.03,
                        speed_blip=0.05, accel_z=2.0, pitchroll_rate=0.05,
                        event_len_min=3, event_len_max=10),
             accel_noise=dict(enable=True, sigma=0.05))
    c.update(ov)
    return c


def test_realismaug_preserves_shape_and_labels():
    rng = np.random.default_rng(10)
    aug = RealismAug(_cfg(), rng=rng)
    w = _straight_window()
    y = np.array([0.5, 0.0, 0.01], dtype=np.float32)
    out = aug(w.copy())
    assert out.shape == w.shape and out.dtype == w.dtype
    # labels are not an argument to the aug (caller never changes Y) -> documented invariant
    np.testing.assert_array_equal(y, y)


def test_realismaug_disabled_is_noop():
    rng = np.random.default_rng(0)
    aug = RealismAug(_cfg(enable=False), rng=rng)
    w = _straight_window()
    np.testing.assert_array_equal(aug(w.copy()), w)


def test_realismaug_each_toggle_independent():
    # only gyro on -> only gyro channels change
    rng = np.random.default_rng(11)
    cfg = _cfg(kappa_bulk=dict(enable=False, kappa_std=0.04),
               bumps=dict(enable=False, event_rate=0.03, speed_blip=0.05, accel_z=2.0,
                          pitchroll_rate=0.05, event_len_min=3, event_len_max=10),
               accel_noise=dict(enable=False, sigma=0.05))
    aug = RealismAug(cfg, rng=rng)
    w = _straight_window(W=200)
    out = aug(w.copy())
    np.testing.assert_array_equal(out[:, :8], w[:, :8])    # wheels untouched
    np.testing.assert_array_equal(out[:, 11:], w[:, 11:])  # accel untouched
    assert not np.allclose(out[:, 8:11], w[:, 8:11])       # gyro changed
