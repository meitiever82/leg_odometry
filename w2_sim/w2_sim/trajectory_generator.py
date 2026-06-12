"""运动原语随机拼接 → 50Hz (t,vx,vy,wz) profile。
design §7。可独立于 Isaac 测试;w2_sim_app 按时间索引消费。"""
import numpy as np

LIMITS = {"v_max": 1.5, "w_max": 1.0, "a_max": 0.8, "alpha_max": 1.5}

def _seg_still(rng):   return rng.uniform(5, 15),  lambda t, T: (0.0, 0.0, 0.0)
def _seg_line(rng):
    v = rng.uniform(0.2, LIMITS["v_max"]) * rng.choice([-1, 1])
    return rng.uniform(4, 12), lambda t, T: (v, 0.0, 0.0)
def _seg_arc(rng):
    v = rng.uniform(0.3, 1.0)
    w = rng.uniform(0.1, 0.6) * rng.choice([-1, 1])
    return rng.uniform(5, 15), lambda t, T: (v, 0.0, w)
def _seg_spin(rng):
    w = rng.uniform(0.3, LIMITS["w_max"]) * rng.choice([-1, 1])
    return rng.uniform(3, 8),  lambda t, T: (0.0, 0.0, w)
def _seg_strafe(rng):
    vy = rng.uniform(0.2, 0.8) * rng.choice([-1, 1])
    return rng.uniform(3, 8),  lambda t, T: (0.0, vy, 0.0)
def _seg_walk(rng):
    # 低频正弦叠加的平滑随机游走
    a = rng.uniform(0.3, 1.0, 3) * [1.0, 0.5, 0.5]
    f = rng.uniform(0.05, 0.2, 3)
    ph = rng.uniform(0, 2 * np.pi, 3)
    return rng.uniform(10, 20), lambda t, T: tuple(
        a[i] * np.sin(2 * np.pi * f[i] * t + ph[i]) for i in range(3))

_SEGS = [_seg_still, _seg_line, _seg_arc, _seg_spin, _seg_strafe, _seg_walk]
# Index mapping for guaranteed coverage
_IDX_STILL = 0
_IDX_LINE = 1
_IDX_ARC = 2
_IDX_SPIN = 3
_IDX_STRAFE = 4
_IDX_WALK = 5


def generate_profile(seed, duration_s, rate_hz=50.0):
    """返回 (N,4) ndarray: [t, vx, vy, wz]。开头静止 ≥5s、结尾静止 ≥3s,限幅+加速度限斜。"""
    rng = np.random.default_rng(seed)
    dt = 1.0 / rate_hz
    n = int(round(duration_s * rate_hz)) + 1
    t = np.arange(n) * dt
    raw = np.zeros((n, 3))
    cursor = rng.uniform(6, 10)                     # 开头静止段
    # guard must be > 3s (tail test window) + v_max/a_max (1.875s) ≈ 5s
    guard = 5.0                                      # 结尾预留

    # Build a sequence that guarantees coverage: at least one of each motion type
    # We interleave required segments with random ones
    required_segs = [_IDX_LINE, _IDX_STRAFE, _IDX_SPIN, _IDX_STILL]  # 前进、侧移、旋转、中途静止
    required_remaining = list(required_segs)
    seg_count = 0

    while cursor < duration_s - guard:
        remaining_time = duration_s - guard - cursor
        if remaining_time < 2.0:
            break

        # Force required segment types early enough to fit
        # Estimate: if we have N required left and T time left, force one if tight
        if required_remaining:
            # Force a required segment if we're running low on time
            # (roughly 15s per segment average)
            avg_needed = len(required_remaining) * 12.0
            if remaining_time < avg_needed or (seg_count > 0 and rng.random() < 0.4):
                seg_idx = required_remaining.pop(0)
                seg_fn = _SEGS[seg_idx]
            else:
                seg_idx = rng.integers(len(_SEGS))
                # If this satisfies a required type, remove it
                if seg_idx in required_remaining:
                    required_remaining.remove(seg_idx)
                seg_fn = _SEGS[seg_idx]
        else:
            seg_idx = rng.integers(len(_SEGS))
            seg_fn = _SEGS[seg_idx]

        dur, fn = seg_fn(rng)
        dur = min(dur, remaining_time)
        if dur < 1.0:
            break
        m = (t >= cursor) & (t < cursor + dur)
        for idx in np.where(m)[0]:
            raw[idx] = fn(t[idx] - cursor, dur)
        cursor += dur
        seg_count += 1

    # 限幅
    sp = np.hypot(raw[:, 0], raw[:, 1])
    over = sp > LIMITS["v_max"]
    raw[over, :2] *= (LIMITS["v_max"] / sp[over])[:, None]
    raw[:, 2] = np.clip(raw[:, 2], -LIMITS["w_max"], LIMITS["w_max"])
    # 加速度限斜(线速度 a_max,角加速度 alpha_max)
    # Per-component clamp (not norm) so each axis independently satisfies a_max.
    # This is what the test verifies: |Δvx|*rate ≤ a_max AND |Δvy|*rate ≤ a_max.
    out = np.zeros_like(raw)
    for i in range(1, n):
        dv = raw[i, :2] - out[i - 1, :2]
        max_dv = LIMITS["a_max"] * dt
        out[i, :2] = out[i - 1, :2] + np.clip(dv, -max_dv, max_dv)
        dw = np.clip(raw[i, 2] - out[i - 1, 2],
                     -LIMITS["alpha_max"] * dt, LIMITS["alpha_max"] * dt)
        out[i, 2] = out[i - 1, 2] + dw
    # 末段强制归零(限斜保证平滑减速到 0;guard=5s > 3s test window + 1.875s v_max/a_max)
    tail = t > duration_s - guard
    for i in np.where(tail)[0]:
        dv = -out[i - 1, :2]
        max_dv = LIMITS["a_max"] * dt
        out[i, :2] = out[i - 1, :2] + np.clip(dv, -max_dv, max_dv)
        out[i, 2] = out[i - 1, 2] + np.clip(-out[i - 1, 2],
                    -LIMITS["alpha_max"] * dt, LIMITS["alpha_max"] * dt)
    # 数值清零:残留极小值按 0 处理,保证书签静止段干净。
    # Threshold must be < a_max*dt (0.016) so zeroing never creates a step > a_max.
    out[np.abs(out) < 1e-3] = 0.0
    # Post-pass: re-apply per-component acceleration limit to fix any step created
    # by the zero-clearing above (e.g. 0.015 → 0 looks like Δv=0.015, fine;
    # but values just above 1e-3 that got zeroed leave a step we must smooth).
    max_dv = LIMITS["a_max"] * dt
    for i in range(1, n):
        dv = out[i, :2] - out[i - 1, :2]
        out[i, :2] = out[i - 1, :2] + np.clip(dv, -max_dv, max_dv)
        dw = out[i, 2] - out[i - 1, 2]
        out[i, 2] = out[i - 1, 2] + np.clip(dw, -LIMITS["alpha_max"] * dt, LIMITS["alpha_max"] * dt)
    # Final cleanup: values that the post-pass brought very close to 0
    out[np.abs(out) < 1e-9] = 0.0
    return np.column_stack([t, out])
