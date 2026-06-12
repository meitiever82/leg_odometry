#pragma once
/**
 * @file swerve_kinematics.h
 * @brief 4-wheel swerve inverse kinematics (wheels → body twist).
 *
 * Each wheel i at body-frame position r_i = (x_i, y_i) measures a contact-point
 * velocity u_i = v_i · [cos θ_i, sin θ_i]. Rigid-body kinematics give
 *   u_i = [vx - ω·y_i, vy + ω·x_i]
 * Stacking 4 wheels gives an 8x3 linear system in z = (vx, vy, ω); least-squares
 * solve via column-pivoted Householder QR. The residual norm doubles as a slip
 * indicator (high residual ⇒ at least one wheel's reported (θ, v) inconsistent
 * with rigid-body motion ⇒ likely slipping or sensor glitch).
 *
 * Sign convention:
 *   - body x forward, y left, z up
 *   - steering angle θ measured from +x_body, positive CCW (right-hand around z)
 *   - speed signed scalar; +v means wheel rolls along its current heading
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <Eigen/Dense>

namespace wheel_odom {

struct WheelGeometry {
  // Wheel contact points in body frame, order: FL, FR, RL, RR.
  std::array<Eigen::Vector2d, 4> r;

  // Build from wheelbase L (front-rear) and track W (left-right).
  static WheelGeometry from_LW(double L, double W) {
    WheelGeometry g;
    g.r[0] = Eigen::Vector2d( L * 0.5,  W * 0.5);  // FL
    g.r[1] = Eigen::Vector2d( L * 0.5, -W * 0.5);  // FR
    g.r[2] = Eigen::Vector2d(-L * 0.5,  W * 0.5);  // RL
    g.r[3] = Eigen::Vector2d(-L * 0.5, -W * 0.5);  // RR
    return g;
  }
};

struct SwerveSolution {
  double vx;            // body-frame linear velocity x (m/s)
  double vy;            // body-frame linear velocity y (m/s)
  double omega_z;       // yaw rate (rad/s)
  double residual;      // ||A z - b||₂ over 8 equations (m/s)
  // Analytic (weighted) LS covariance of z=(vx,vy,ωz): Σ = σ²·(AᵀWA)⁻¹, with the
  // unbiased variance σ² = (rᵀWr)/(n−p), n=8, p=3. Full 3×3 incl. vx/vy/ωz cross
  // terms; the downstream node maps it into the published twist covariance.
  // With per-wheel weights wᵢ ∝ |vᵢ| (see solve_body_twist), a near-stationary
  // wheel — whose steering angle barely constrains the solve (vᵢ·cosθ→0) — is
  // down-weighted, so (AᵀWA)⁻¹ inflates the under-constrained direction: the
  // low-speed angle singularity is now reflected in the cov. With weighting off
  // (W=I) this reduces to plain OLS, where that singularity is NOT reflected.
  Eigen::Matrix3d cov;
};

// weight_by_speed=true applies per-wheel weights wᵢ = max(|vᵢ|, weight_floor)
// (the same weight on the wheel's two rows), down-weighting near-stationary
// wheels whose steering angle no longer constrains the solve. weight_floor is a
// small m/s value so a fully stopped wheel keeps a tiny (not zero) vote and the
// system stays well-posed. weight_by_speed=false → W=I → plain OLS.
inline SwerveSolution solve_body_twist(
    const std::array<double, 4>& angles,
    const std::array<double, 4>& speeds,
    const WheelGeometry& geom,
    bool weight_by_speed = false,
    double weight_floor = 0.05)
{
  Eigen::Matrix<double, 8, 3> A;
  Eigen::Matrix<double, 8, 1> b;
  Eigen::Matrix<double, 8, 1> w;   // per-row weight (diag of W)
  for (int i = 0; i < 4; ++i) {
    const double xi = geom.r[i].x();
    const double yi = geom.r[i].y();
    A.row(2 * i)     << 1.0, 0.0, -yi;
    A.row(2 * i + 1) << 0.0, 1.0,  xi;
    b(2 * i)     = speeds[i] * std::cos(angles[i]);
    b(2 * i + 1) = speeds[i] * std::sin(angles[i]);
    const double wi =
        weight_by_speed ? std::max(std::abs(speeds[i]), weight_floor) : 1.0;
    w(2 * i) = wi;
    w(2 * i + 1) = wi;
  }

  // Weighted solve via QR on W^{1/2}·{A,b}; residual r kept UNWEIGHTED so it
  // stays a direct slip indicator (and matches the slip_threshold semantics).
  const Eigen::Matrix<double, 8, 1> sw = w.cwiseSqrt();
  const Eigen::Vector3d z =
      (sw.asDiagonal() * A).colPivHouseholderQr().solve(sw.asDiagonal() * b);
  const Eigen::Matrix<double, 8, 1> r = A * z - b;
  const double residual = r.norm();

  // Weighted covariance: σ² = (rᵀWr)/(n−p), Σ = σ²·(AᵀWA)⁻¹.
  constexpr int n = 8, p = 3;
  const double wsse = (w.array() * r.array().square()).sum();
  const double sigma2 = wsse / static_cast<double>(n - p);
  const Eigen::Matrix3d cov =
      sigma2 * (A.transpose() * w.asDiagonal() * A).inverse();

  return SwerveSolution{ z(0), z(1), z(2), residual, cov };
}

}  // namespace wheel_odom
