#!/usr/bin/env python3
"""Compare an estimated TUM trajectory against a reference (GLIM traj_imu.txt):
time-associate, Umeyama-align (no scale), report APE, and save an XY overlay +
error-over-time PNG.

Standalone (numpy + matplotlib only) — avoids evo's broken mpl_toolkits.mplot3d
import on this machine.

Usage:
  eval_glim.py REF.tum EST.tum [--t_max_diff 0.05] [--mode se3|se2] [--out PNG]
"""
from __future__ import annotations

import argparse
import math
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402
import numpy as np  # noqa: E402


def load_tum(path: str) -> np.ndarray:
    """Return Nx8 array: t, x,y,z, qx,qy,qz,qw."""
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            p = line.split()
            if len(p) >= 8:
                rows.append([float(v) for v in p[:8]])
    if not rows:
        raise ValueError(f"no poses in {path}")
    return np.asarray(rows)


def associate(ref: np.ndarray, est: np.ndarray, t_max_diff: float):
    """For each ref stamp, find nearest est stamp within t_max_diff.
    Returns (ref_xyz, est_xyz) Nx3 matched."""
    et = est[:, 0]
    order = np.argsort(et)
    et_sorted = et[order]
    ref_idx, est_idx = [], []
    for i, t in enumerate(ref[:, 0]):
        j = np.searchsorted(et_sorted, t)
        cand = []
        if j < len(et_sorted):
            cand.append(j)
        if j > 0:
            cand.append(j - 1)
        best = min(cand, key=lambda k: abs(et_sorted[k] - t), default=None)
        if best is not None and abs(et_sorted[best] - t) <= t_max_diff:
            ref_idx.append(i)
            est_idx.append(order[best])
    if len(ref_idx) < 3:
        raise ValueError(f"only {len(ref_idx)} associations within {t_max_diff}s")
    return ref[ref_idx, 1:4], est[est_idx, 1:4], np.array(ref_idx), np.array(est_idx)


def umeyama(src: np.ndarray, dst: np.ndarray):
    """Rigid (no scale) align src->dst. Returns R, t. (src,dst: Nx3)"""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    s, d = src - mu_s, dst - mu_d
    H = s.T @ d / len(src)
    U, _, Vt = np.linalg.svd(H)
    D = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        D[2, 2] = -1
    R = Vt.T @ D @ U.T
    t = mu_d - R @ mu_s
    return R, t


def yaw_from_quat(arr: np.ndarray) -> np.ndarray:
    """Unwrapped yaw (rad) from TUM rows (cols 4..7 = qx,qy,qz,qw)."""
    qx, qy, qz, qw = arr[:, 4], arr[:, 5], arr[:, 6], arr[:, 7]
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return np.unwrap(np.arctan2(siny, cosy))


def yaw_analysis(ref: np.ndarray, est: np.ndarray, hz: float = 8.0):
    """Compare yaw over the time overlap on a uniform grid (avoids the
    total-variation / aliasing traps of sum|dyaw| and nearest-neighbour
    matching). Returns a dict of metrics + the grid arrays for plotting."""
    rt, et = ref[:, 0], est[:, 0]
    ry, ey = yaw_from_quat(ref), yaw_from_quat(est)
    lo, hi = max(rt[0], et[0]), min(rt[-1], et[-1])
    if hi - lo < 1.0:
        return None
    grid = np.arange(lo, hi, 1.0 / hz)
    rg = np.interp(grid, rt, ry)
    eg = np.interp(grid, et, ey)
    rg0, eg0 = rg - rg[0], eg - eg[0]          # drift-align at overlap start
    yaw_err = np.degrees(eg0 - rg0)
    r_rate = np.gradient(rg, grid)
    e_rate = np.gradient(eg, grid)
    corr = float(np.corrcoef(r_rate, e_rate)[0, 1])
    return dict(
        grid=grid - grid[0], ref=np.degrees(rg0), est=np.degrees(eg0),
        err=yaw_err, r_rate=r_rate, e_rate=e_rate, corr=corr,
        ref_net=float(np.degrees(rg0[-1])), est_net=float(np.degrees(eg0[-1])),
        err_std=float(yaw_err.std()), err_final=float(yaw_err[-1]),
        overlap=float(hi - lo),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ref")
    ap.add_argument("est")
    ap.add_argument("--t_max_diff", type=float, default=0.05)
    ap.add_argument("--mode", choices=["se3", "se2"], default="se3",
                    help="se2 zeroes z before aligning (planar wheel odom vs 3D ref)")
    ap.add_argument("--out", default="/tmp/wheel_eval/eval_glim.png")
    args = ap.parse_args()

    ref = load_tum(args.ref)
    est = load_tum(args.est)
    ref_xyz, est_xyz, ref_i, est_i = associate(ref, est, args.t_max_diff)

    src, dst = est_xyz.copy(), ref_xyz.copy()
    if args.mode == "se2":
        src[:, 2] = 0.0
        dst[:, 2] = 0.0

    R, t = umeyama(src, dst)
    est_aligned = (R @ est_xyz.T).T + t

    err = np.linalg.norm(est_aligned - ref_xyz, axis=1)
    rmse = float(np.sqrt((err ** 2).mean()))

    def plen(xyz):
        return float(np.sum(np.linalg.norm(np.diff(xyz, axis=0), axis=1)))

    ref_len = plen(ref[:, 1:4])
    est_len = plen(est[:, 1:4])
    final_err = float(err[-1])
    t_rel = ref[ref_i, 0] - ref[ref_i, 0][0]

    print(f"associations : {len(err)}  (t_max_diff={args.t_max_diff}s, mode={args.mode})")
    print(f"APE rmse     : {rmse:.3f} m")
    print(f"APE mean/med : {err.mean():.3f} / {np.median(err):.3f} m")
    print(f"APE max/min  : {err.max():.3f} / {err.min():.3f} m")
    print(f"final drift  : {final_err:.3f} m")
    print(f"path length  : ref={ref_len:.2f} m   est={est_len:.2f} m   "
          f"ratio={est_len / ref_len if ref_len else float('nan'):.4f}")
    print(f"APE / ref_len: {100 * rmse / ref_len if ref_len else float('nan'):.2f} %")

    ya = yaw_analysis(ref, est)
    if ya:
        print(f"--- yaw (overlap {ya['overlap']:.1f}s, uniform grid) ---")
        print(f"net rotation : ref={ya['ref_net']:+.1f}  est={ya['est_net']:+.1f} deg  "
              f"(ratio {ya['est_net'] / ya['ref_net'] if ya['ref_net'] else float('nan'):.3f})")
        print(f"yaw rate corr: {ya['corr']:.3f}")
        print(f"yaw err      : std={ya['err_std']:.1f} deg  final={ya['err_final']:+.1f} deg")

    # ---- plot ----
    fig, axes = plt.subplots(2, 2, figsize=(16, 13))
    ax = axes[0, 0]
    ax.plot(ref[:, 1], ref[:, 2], "-", color="C0", lw=1.6, label="GLIM ref")
    pts = est_aligned[:, :2].reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc = LineCollection(segs, cmap="viridis", linewidth=1.4)
    lc.set_array(t_rel)
    ax.add_collection(lc)
    ax.plot(est_aligned[0, 0], est_aligned[0, 1], "o", color="lime",
            ms=11, mec="k", label="est start", zorder=5)
    ax.plot(est_aligned[-1, 0], est_aligned[-1, 1], "*", color="red",
            ms=16, mec="k", label="est end", zorder=5)
    ax.set_aspect("equal"); ax.grid(True, ls=":", alpha=0.6)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title(f"XY overlay (est aligned, {args.mode})   "
                 f"APE rmse={rmse:.2f} m  final={final_err:.2f} m")
    ax.legend(loc="best")
    fig.colorbar(lc, ax=ax, shrink=0.8, pad=0.02).set_label("time [s]")

    ax = axes[0, 1]
    ax.plot(t_rel, err, "-", color="C3", lw=1.0)
    ax.axhline(rmse, color="k", ls="--", lw=1.0, alpha=0.7, label=f"rmse={rmse:.2f} m")
    ax.grid(True, ls=":", alpha=0.6)
    ax.set_xlabel("t [s]"); ax.set_ylabel("APE [m]")
    ax.set_title(f"APE over time   path: ref={ref_len:.1f} m est={est_len:.1f} m "
                 f"(ratio {est_len / ref_len:.3f})")
    ax.legend(loc="best")

    # ---- (1,0) yaw over time (drift-aligned at overlap start) ----
    ax = axes[1, 0]
    if ya:
        ax.plot(ya["grid"], ya["ref"], "-", color="C0", lw=1.6, label="GLIM ref")
        ax.plot(ya["grid"], ya["est"], "-", color="C1", lw=1.2, label="wheel est")
        ax.grid(True, ls=":", alpha=0.6)
        ax.set_xlabel("t [s]"); ax.set_ylabel("yaw [deg]")
        ax.set_title(f"yaw (net: ref={ya['ref_net']:+.0f}  est={ya['est_net']:+.0f} deg, "
                     f"ratio {ya['est_net'] / ya['ref_net'] if ya['ref_net'] else float('nan'):.2f})")
        ax.legend(loc="best")
    else:
        ax.text(0.5, 0.5, "no time overlap for yaw", ha="center", va="center")
        ax.set_axis_off()

    # ---- (1,1) yaw error + rate correlation ----
    ax = axes[1, 1]
    if ya:
        ax.plot(ya["grid"], ya["err"], "-", color="C3", lw=1.0)
        ax.axhline(0, color="k", lw=0.5, alpha=0.5)
        ax.grid(True, ls=":", alpha=0.6)
        ax.set_xlabel("t [s]"); ax.set_ylabel("yaw err (est-ref) [deg]")
        ax.set_title(f"yaw error (drift-removed)   std={ya['err_std']:.1f}  "
                     f"final={ya['err_final']:+.1f} deg   rate corr={ya['corr']:.3f}")
    else:
        ax.set_axis_off()

    fig.suptitle(f"wheel_odom vs GLIM   {args.est.split('/')[-1]}", y=0.99)
    plt.tight_layout(rect=(0, 0, 1, 0.97))
    from pathlib import Path
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(args.out, dpi=120)
    print(f"saved {args.out}")


if __name__ == "__main__":
    sys.exit(main())
