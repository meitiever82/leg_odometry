#pragma once
/**
 * @file UpdaterZUPT.h
 * @brief ZUPT velocity constraint + step-level velocity + stillness.
 *
 * Contact foot world velocity ≈ 0:
 *   v + ω_w × (R·FK) + R·dFK/dt ≈ 0
 *   → v ≈ -(ω_w × (R·FK) + R·dFK/dt)
 */

#include "../state/State.h"
#include "../state/StateHelper.h"

namespace leg_odom {

class UpdaterZUPT {
 public:
  /**
   * @brief Frame-level ZUPT + step-level velocity + stillness + double-support.
   */
  static void update(State& s, const NoiseParams& noise,
                     const Eigen::Vector3d& fk_left_body,
                     const Eigen::Vector3d& fk_right_body,
                     bool contact_left, bool contact_right) {
    // 分轴 ZUPT 噪声: XY 更紧, Z 由 FlatZ 管
    Eigen::Matrix3d R_zupt = Eigen::Matrix3d::Identity() * noise.sigma_zupt * noise.sigma_zupt;
    R_zupt(2, 2) *= 4.0;  // Z 方向放松
    Eigen::MatrixXd H_v = Eigen::MatrixXd::Zero(3, DIM);
    H_v.block<3,3>(0, V) = Eigen::Matrix3d::Identity();

    // === Frame-level ZUPT ===
    if (contact_left && s.has_prev_fk) {
      Eigen::Vector3d v_exp = compute_v_expected(s, fk_left_body, s.prev_fk_left);
      StateHelper::ekf_update(s, noise, v_exp - s.v, H_v, R_zupt);
    }
    if (contact_right && s.has_prev_fk) {
      Eigen::Vector3d v_exp = compute_v_expected(s, fk_right_body, s.prev_fk_right);
      StateHelper::ekf_update(s, noise, v_exp - s.v, H_v, R_zupt);
    }

    // === Step-level velocity (stance phase average) ===
    step_velocity_left(s, noise, fk_left_body, contact_left);
    step_velocity_right(s, noise, fk_right_body, contact_right);

    // === Double-support vz constraint ===
    if (contact_left && contact_right) {
      double sigma_ds = noise.sigma_zupt * 0.5;
      Eigen::MatrixXd H_vz = Eigen::MatrixXd::Zero(1, DIM);
      H_vz(0, V + 2) = 1.0;
      Eigen::VectorXd res(1);
      res << -s.v(2);
      Eigen::MatrixXd R_ds(1, 1);
      R_ds << sigma_ds * sigma_ds;
      StateHelper::ekf_update(s, noise, res, H_vz, R_ds);
    }

    // === Stillness ===
    if (s.is_still) {
      double sigma_still = 0.005;
      Eigen::Matrix3d R_still = Eigen::Matrix3d::Identity() * sigma_still * sigma_still;
      StateHelper::ekf_update(s, noise, -s.v, H_v, R_still);
    }

    // Update previous FK
    s.prev_fk_left = fk_left_body;
    s.prev_fk_right = fk_right_body;
    s.has_prev_fk = true;
  }

 private:
  static Eigen::Vector3d compute_v_expected(const State& s,
                                             const Eigen::Vector3d& fk_now,
                                             const Eigen::Vector3d& fk_prev) {
    double dt_fk = 0.005;  // FK sample interval
    Eigen::Vector3d R_dfk = s.R * ((fk_now - fk_prev) / dt_fk);
    Eigen::Vector3d r_world = s.R * fk_now;
    Eigen::Vector3d omega_world = s.R * s.last_gyro_corrected;
    Eigen::Vector3d omega_cross = omega_world.cross(r_world);
    return -(omega_cross + R_dfk);
  }

  static void step_velocity_left(State& s, const NoiseParams& noise,
                                  const Eigen::Vector3d& fk, bool contact) {
    if (contact && !s.prev_contact_left) {
      s.stance_start_fk_left = fk;
      s.stance_frames_left = 0;
    }
    if (contact) s.stance_frames_left++;

    if (!contact && s.prev_contact_left && s.stance_frames_left > 0) {
      double stance_dt = s.stance_frames_left * s.last_dt;
      if (stance_dt > 0.05) {
        Eigen::Vector3d dfk = fk - s.stance_start_fk_left;
        Eigen::Vector3d v_step = -(s.R * (dfk / stance_dt));
        double sigma = noise.sigma_zupt * 2.0;
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, DIM);
        H.block<3,3>(0, V) = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d R_step = Eigen::Matrix3d::Identity() * sigma * sigma;
        StateHelper::ekf_update(s, noise, v_step - s.v, H, R_step);
      }
    }
    s.prev_contact_left = contact;
  }

  static void step_velocity_right(State& s, const NoiseParams& noise,
                                   const Eigen::Vector3d& fk, bool contact) {
    if (contact && !s.prev_contact_right) {
      s.stance_start_fk_right = fk;
      s.stance_frames_right = 0;
    }
    if (contact) s.stance_frames_right++;

    if (!contact && s.prev_contact_right && s.stance_frames_right > 0) {
      double stance_dt = s.stance_frames_right * s.last_dt;
      if (stance_dt > 0.05) {
        Eigen::Vector3d dfk = fk - s.stance_start_fk_right;
        Eigen::Vector3d v_step = -(s.R * (dfk / stance_dt));
        double sigma = noise.sigma_zupt * 2.0;
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, DIM);
        H.block<3,3>(0, V) = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d R_step = Eigen::Matrix3d::Identity() * sigma * sigma;
        StateHelper::ekf_update(s, noise, v_step - s.v, H, R_step);
      }
    }
    s.prev_contact_right = contact;
  }
};

}  // namespace leg_odom
