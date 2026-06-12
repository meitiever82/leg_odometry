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

#include <array>
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
  // Analytic OLS covariance of z=(vx,vy,ωz): Σ = σ²·(AᵀA)⁻¹, with the unbiased
  // variance σ² = ||r||²/(n−p), n=8, p=3. Full 3×3 incl. vx/vy/ωz cross terms;
  // the downstream node maps it into the published twist covariance. NOTE: this
  // is the UNWEIGHTED OLS cov (matches the unweighted solve). The |v_i|-weighted
  // form (plan §1.7) — which makes the cov inflate for a near-stationary single
  // wheel — is a future refinement; with W=I the low-speed angle-singularity is
  // not reflected here.
  Eigen::Matrix3d cov;
};

inline SwerveSolution solve_body_twist(
    const std::array<double, 4>& angles,
    const std::array<double, 4>& speeds,
    const WheelGeometry& geom)
{
  Eigen::Matrix<double, 8, 3> A;
  Eigen::Matrix<double, 8, 1> b;
  for (int i = 0; i < 4; ++i) {
    const double xi = geom.r[i].x();
    const double yi = geom.r[i].y();
    A.row(2 * i)     << 1.0, 0.0, -yi;
    A.row(2 * i + 1) << 0.0, 1.0,  xi;
    b(2 * i)     = speeds[i] * std::cos(angles[i]);
    b(2 * i + 1) = speeds[i] * std::sin(angles[i]);
  }
  const Eigen::Vector3d z = A.colPivHouseholderQr().solve(b);
  const double residual = (A * z - b).norm();

  // OLS covariance: σ² = SSE/(n−p), Σ = σ²·(AᵀA)⁻¹.  (AᵀA is constant geometry;
  // recomputed here for locality — it is a cheap 3×3.)
  constexpr int n = 8, p = 3;
  const double sigma2 = (residual * residual) / static_cast<double>(n - p);
  const Eigen::Matrix3d cov = sigma2 * (A.transpose() * A).inverse();

  return SwerveSolution{ z(0), z(1), z(2), residual, cov };
}

}  // namespace wheel_odom
