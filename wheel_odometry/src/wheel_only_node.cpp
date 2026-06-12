/**
 * @file wheel_only_node.cpp
 * @brief Swerve-only wheel odometry ROS2 node.
 *
 * State:    p ∈ R³, R ∈ SO(3), gyro_bias ∈ R³.
 * Pipeline (driven by /robot/wheel_status, sensor_msgs/JointState):
 *   1. solve LS:                     (vx_b, vy_b, ω_z_LS, residual)
 *   1b. yaw-bias correction:         ω_z ← ω_z_LS − yaw_kappa · vx_b
 *                                    (κ from calibrate_kappa.py; removes the
 *                                    speed-proportional steering/scale bias that
 *                                    otherwise drifts wheel yaw ~1.9°/s. Applied
 *                                    only on the LS-yaw path, not the gyro path.)
 *   2. propagate R:                  yaw_source ∈ {ls, gyro}
 *                                    R ← R · exp_so3((ω_used) · dt)
 *   3. accel-tilt Mahony (gated):    pulls R's roll/pitch toward gravity,
 *                                    keeps yaw free.  Skipped if no IMU.
 *   4. body velocity in world:       v_world = R · (vx_b, vy_b, 0)
 *   5. position integration:         p ← p + v_world · dt
 *   6. FlatZ clamp:                  p.z ← (1-α) · p.z   (flat-floor prior)
 *
 * Subscribes: /robot/wheel_status (JointState; 4 names, position=θ, velocity=v),
 *             /imu                (optional — node operates fine without it).
 * Publishes:  /wheel_odometry, TF odom → base_frame.
 *
 * Wheel name → index mapping: looks for substrings "front_left" / "front_right"
 * / "rear_left" / "rear_right" in JointState.name. Falls back to position 0..3
 * (= FL, FR, RL, RR) if names are absent.
 */

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "wheel_odometry/so3_utils.h"
#include "wheel_odometry/swerve_kinematics.h"

namespace {

constexpr double kGravity = 9.81;
// Default ceiling on chassis stamp dt (s). Overridable via the 'max_dt' ROS
// param. Sized above the sensor's observed worst burst gap (~1.0 s, both on the
// 2026-05-14 bag and on-robot) so real motion across a gap is integrated,
// not dropped. The chassis stream is bursty (sensor-side); see [diag] logs.
constexpr double kDefaultMaxDt = 1.2;

// Find index of a wheel by substring of its joint name. Returns -1 if missing.
int find_wheel(const std::vector<std::string>& names, const char* key) {
  for (size_t i = 0; i < names.size(); ++i) {
    if (names[i].find(key) != std::string::npos) return static_cast<int>(i);
  }
  return -1;
}

}  // namespace

class WheelOnlyNode : public rclcpp::Node {
 public:
  WheelOnlyNode() : Node("wheel_only_odom") {
    declare_parameter("wheelbase",   0.435);   // w2 measured (was 0.6 placeholder)
    declare_parameter("track",       0.400);   // w2 measured (was 0.5 placeholder)
    declare_parameter("wheel_radius", 1.0);    // w2 speed field is already m/s

    declare_parameter("bias_window_sec", 3.0);
    declare_parameter("tilt_kp",         1.0);
    declare_parameter("tilt_accel_band", 0.5);

    declare_parameter("yaw_source",     std::string("ls"));
    declare_parameter("slip_threshold", 0.5);
    // Curvature-bias correction (rad/m): ω_z ← ω_z_LS − yaw_kappa·vx_b. Removes
    // the speed-proportional wheel-yaw drift (~1.9°/s on w2). Per-session value
    // from scripts/calibrate_kappa.py; 0.0 = off. Only used on the LS-yaw path.
    declare_parameter("yaw_kappa",      0.0);

    // ---- covariance / STILL (ZUPT) ----
    // Diagonal cov for the dims wheels never observe (vz, ωx, ωy): ~∞ info → 0.
    declare_parameter("cov_no_observation", 1.0e6);
    // ωz noise std on the gyro-yaw path (rad/s); its variance replaces the LS
    // analytic ωz variance when yaw_source=gyro.
    declare_parameter("gyro_yaw_sigma",     0.01);
    // STILL (zero-velocity) detection thresholds.
    declare_parameter("still_speed_eps",    0.02);   // body speed (m/s)
    declare_parameter("still_gyro_eps",     0.02);   // ‖gyro−bg‖ (rad/s), imu only
    // Floor cov when STILL: wheels assert all 6 dims are 0 (not "unobserved").
    declare_parameter("floor_sigma_v",      0.005);  // m/s
    declare_parameter("floor_sigma_omega",  0.001);  // rad/s
    // Mirror the twist (vx,vy,ωz) variance into pose.covariance so rviz's
    // Odometry "Covariance" display can render it (x/y ellipse + yaw cone). This
    // is a VISUALISATION proxy (velocity variance shown as if it were pose
    // variance); when on, pose.cov no longer carries the residual/is_still
    // diagnostics (the trajectory_plotter residual subplot then reads ~0 — use
    // the CSV's ls_residual column instead). Default off.
    declare_parameter("mirror_twist_cov_to_pose", false);

    // Max chassis stamp dt (s) still integrated; larger gaps drop the tick.
    declare_parameter("max_dt",         kDefaultMaxDt);

    declare_parameter("flatz_enabled", true);
    declare_parameter("flatz_alpha",   0.05);

    declare_parameter("publish_tf",    true);
    declare_parameter("odom_frame",    std::string("odom"));
    declare_parameter("base_frame",    std::string("base_link_wheel_odom"));
    declare_parameter("odom_topic",    std::string("/wheel_odometry"));
    declare_parameter("chassis_topic", std::string("/robot/wheel_status"));
    declare_parameter("imu_topic",     std::string("/imu"));
    declare_parameter("enable_imu",    false);

    declare_parameter("diag_csv_path", std::string(""));

    // Period (s) of the periodic [diag] summary log; <=0 disables it.
    declare_parameter("diag_log_period_sec", 2.0);

    geom_ = wheel_odom::WheelGeometry::from_LW(
        get_parameter("wheelbase").as_double(),
        get_parameter("track").as_double());
    wheel_radius_   = get_parameter("wheel_radius").as_double();
    bias_window_sec_= get_parameter("bias_window_sec").as_double();
    tilt_kp_        = get_parameter("tilt_kp").as_double();
    tilt_accel_band_= get_parameter("tilt_accel_band").as_double();
    yaw_source_     = get_parameter("yaw_source").as_string();
    slip_threshold_ = get_parameter("slip_threshold").as_double();
    yaw_kappa_      = get_parameter("yaw_kappa").as_double();
    cov_no_obs_     = get_parameter("cov_no_observation").as_double();
    gyro_yaw_var_   = std::pow(get_parameter("gyro_yaw_sigma").as_double(), 2);
    still_speed_eps_= get_parameter("still_speed_eps").as_double();
    still_gyro_eps_ = get_parameter("still_gyro_eps").as_double();
    floor_v2_       = std::pow(get_parameter("floor_sigma_v").as_double(), 2);
    floor_w2_       = std::pow(get_parameter("floor_sigma_omega").as_double(), 2);
    mirror_cov_     = get_parameter("mirror_twist_cov_to_pose").as_bool();
    max_dt_         = get_parameter("max_dt").as_double();
    flatz_enabled_  = get_parameter("flatz_enabled").as_bool();
    flatz_alpha_    = get_parameter("flatz_alpha").as_double();
    publish_tf_     = get_parameter("publish_tf").as_bool();
    odom_frame_     = get_parameter("odom_frame").as_string();
    base_frame_     = get_parameter("base_frame").as_string();
    enable_imu_     = get_parameter("enable_imu").as_bool();
    const auto odom_topic    = get_parameter("odom_topic").as_string();
    const auto chassis_topic = get_parameter("chassis_topic").as_string();
    const auto imu_topic     = get_parameter("imu_topic").as_string();

    if (yaw_source_ != "ls" && yaw_source_ != "gyro") {
      RCLCPP_WARN(get_logger(),
          "yaw_source='%s' invalid, falling back to 'ls'", yaw_source_.c_str());
      yaw_source_ = "ls";
    }
    if (!enable_imu_ && yaw_source_ == "gyro") {
      RCLCPP_WARN(get_logger(),
          "enable_imu=false but yaw_source=gyro — forcing yaw_source=ls");
      yaw_source_ = "ls";
    }
    // Without IMU we have no bias-init window; initialise immediately so the
    // first chassis message can do useful work.
    if (!enable_imu_) init_done_ = true;

    const auto diag_path = get_parameter("diag_csv_path").as_string();
    if (!diag_path.empty()) {
      diag_fp_ = std::fopen(diag_path.c_str(), "w");
      if (diag_fp_) {
        std::fprintf(diag_fp_,
            "t_abs,x,y,z,roll,pitch,yaw,vx_w,vy_w,vz_w,vx_b,vy_b,"
            "wz_ls,wz_used,ls_residual,tilt_applied,used_ls_yaw,is_still\n");
      }
    }

    const auto chassis_qos = rclcpp::QoS(rclcpp::KeepLast(2000)).reliable();
    chassis_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        chassis_topic, chassis_qos,
        std::bind(&WheelOnlyNode::chassis_cb, this, std::placeholders::_1));
    if (enable_imu_) {
      const auto imu_qos = rclcpp::SensorDataQoS().keep_last(2000);
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
          imu_topic, imu_qos,
          std::bind(&WheelOnlyNode::imu_cb, this, std::placeholders::_1));
    }
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
    tf_bc_    = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    diag_log_period_sec_ = get_parameter("diag_log_period_sec").as_double();
    if (diag_log_period_sec_ > 0.0) {
      diag_timer_ = create_wall_timer(
          std::chrono::duration<double>(diag_log_period_sec_),
          std::bind(&WheelOnlyNode::diag_timer_cb, this));
    }
    RCLCPP_INFO(get_logger(),
        "[diag] subscribed to '%s' (QoS KeepLast 2000, RELIABLE) — waiting for "
        "first message; [diag] summary every %.1fs",
        chassis_topic.c_str(), diag_log_period_sec_);

    p_.setZero();
    R_.setIdentity();
    bg_.setZero();

    RCLCPP_INFO(get_logger(),
        "wheel_only_odom: L=%.3f W=%.3f imu=%s yaw_source=%s κ=%.4f "
        "slip_thr=%.3f  max_dt=%.2f  flatz=%s α=%.3f  chassis=%s odom=%s",
        get_parameter("wheelbase").as_double(),
        get_parameter("track").as_double(),
        enable_imu_ ? "on" : "off", yaw_source_.c_str(), yaw_kappa_,
        slip_threshold_,
        max_dt_,
        flatz_enabled_ ? "on" : "off", flatz_alpha_,
        chassis_topic.c_str(), odom_topic.c_str());
  }

  ~WheelOnlyNode() { if (diag_fp_) std::fclose(diag_fp_); }

 private:
  // -------------------------------------------------------------------------
  // IMU callback — bias init + cache for chassis_cb. Only wired up when
  // enable_imu=true.
  // -------------------------------------------------------------------------
  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (!first_imu_logged_) {
      RCLCPP_INFO(get_logger(), "[diag] first /imu message received");
      first_imu_logged_ = true;
    }
    const Eigen::Vector3d g(msg->angular_velocity.x,
                            msg->angular_velocity.y,
                            msg->angular_velocity.z);
    const Eigen::Vector3d a(msg->linear_acceleration.x,
                            msg->linear_acceleration.y,
                            msg->linear_acceleration.z);
    const double t_abs = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    if (!init_done_) {
      if (init_window_start_ < 0.0) init_window_start_ = t_abs;
      if (t_abs - init_window_start_ < bias_window_sec_) {
        accum_gyro_  += g;
        accum_accel_ += a;
        n_static_++;
        last_gyro_ = g;
        return;
      }
      if (n_static_ > 0) {
        bg_ = accum_gyro_ / n_static_;
        const Eigen::Vector3d avg_a = accum_accel_ / n_static_;
        const Eigen::Vector3d up_imu = avg_a.normalized();
        const Eigen::Vector3d up_base(0, 0, 1);
        R_base_imu_ = Eigen::Quaterniond::FromTwoVectors(up_imu, up_base).toRotationMatrix();
        R_.setIdentity();
        const auto rpy_mount = R_to_rpy(R_base_imu_);
        RCLCPP_INFO(get_logger(),
            "init done (%d samples): bg=[%+.5f,%+.5f,%+.5f]  "
            "R_base_imu rpy=[%+.2f,%+.2f,%+.2f] deg",
            n_static_, bg_.x(), bg_.y(), bg_.z(),
            rpy_mount[0] * 180.0 / M_PI, rpy_mount[1] * 180.0 / M_PI,
            rpy_mount[2] * 180.0 / M_PI);
      }
      init_done_ = true;
    }
    last_gyro_  = R_base_imu_ * (g - bg_);
    last_accel_ = R_base_imu_ * a;
    has_accel_  = true;
  }

  // -------------------------------------------------------------------------
  // Chassis-state callback — main state update.
  // -------------------------------------------------------------------------
  void chassis_cb(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // ---- arrival / latency diagnostics ----
    const auto wall_now = std::chrono::steady_clock::now();
    ++rx_count_;
    double wall_dt = -1.0;
    if (have_last_chassis_wall_) {
      wall_dt = std::chrono::duration<double>(wall_now - last_chassis_wall_).count();
    }
    last_chassis_wall_ = wall_now;
    have_last_chassis_wall_ = true;
    const double now_ros = this->now().seconds();

    if (!first_chassis_logged_) {
      const double since_start =
          std::chrono::duration<double>(wall_now - node_start_wall_).count();
      const double stamp0 =
          msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
      RCLCPP_INFO(get_logger(),
          "[diag] FIRST /robot/wheel_status arrived %.2fs after node start | "
          "header.stamp=%.3f node.now=%.3f stamp_vs_now=%+.3fs",
          since_start, stamp0, now_ros, now_ros - stamp0);
      first_chassis_logged_ = true;
    }

    if (msg->position.size() < 4 || msg->velocity.size() < 4) {
      ++drop_bad_size_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "JointState has <4 wheels (pos=%zu vel=%zu); dropping",
          msg->position.size(), msg->velocity.size());
      return;
    }

    // Resolve FL/FR/RL/RR indices once (cache after first message).
    if (!idx_resolved_) {
      const auto& names = msg->name;
      idx_fl_ = find_wheel(names, "front_left");
      idx_fr_ = find_wheel(names, "front_right");
      idx_rl_ = find_wheel(names, "rear_left");
      idx_rr_ = find_wheel(names, "rear_right");
      if (idx_fl_ < 0 || idx_fr_ < 0 || idx_rl_ < 0 || idx_rr_ < 0) {
        idx_fl_ = 0; idx_fr_ = 1; idx_rl_ = 2; idx_rr_ = 3;
        RCLCPP_WARN(get_logger(),
            "JointState names not 'front_left'/.../'rear_right'; "
            "falling back to position 0..3 = FL,FR,RL,RR");
      } else {
        RCLCPP_INFO(get_logger(),
            "wheel indices resolved: FL=%d FR=%d RL=%d RR=%d",
            idx_fl_, idx_fr_, idx_rl_, idx_rr_);
      }
      idx_resolved_ = true;
    }

    const double t_abs = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    if (last_t_ < 0.0) {
      last_t_ = t_abs;
      RCLCPP_INFO(get_logger(),
          "[diag] first stamp seen (%.3f); last_t initialised, no odom this "
          "tick (expected — needs two messages to form a dt)", t_abs);
      return;
    }
    const double prev_t = last_t_;
    const double dt = t_abs - prev_t;
    last_t_ = t_abs;

    // stamp_dt  = sensor-clock gap between messages (drives integration)
    // wall_dt   = real time between callbacks (how fast they actually arrive)
    // stamp_vs_now = how far the sensor stamp lags the node clock
    // Compare them: wall_dt small but stamp_dt large => sensor stamps are bad;
    // both large => sensor genuinely publishes slowly; stamp_vs_now large &
    // growing => buffering / clock-domain mismatch.
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "[diag] chassis: stamp_dt=%.3fs wall_dt=%.3fs stamp_vs_now=%+.3fs "
        "(arrival~%.1fHz)",
        dt, wall_dt, now_ros - t_abs,
        wall_dt > 1e-6 ? 1.0 / wall_dt : 0.0);

    if (dt <= 0.0) {
      ++drop_dt_nonpos_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "[diag] DROP (no odom): chassis stamp dt=%.4fs <= 0 "
          "(t_abs=%.3f prev=%.3f) — sensor timestamps not monotonic", dt,
          t_abs, prev_t);
      return;
    }
    if (dt > max_dt_) {
      ++drop_dt_big_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "[diag] DROP (no odom): chassis stamp dt=%.3fs > max_dt=%.2fs — gap "
          "in the sensor stream (low rate / pause / startup); wall_dt=%.3fs",
          dt, max_dt_, wall_dt);
      return;
    }

    // ---- 1. swerve LS ----
    const std::array<double, 4> angles = {
        msg->position[idx_fl_], msg->position[idx_fr_],
        msg->position[idx_rl_], msg->position[idx_rr_],
    };
    const std::array<double, 4> speeds = {
        msg->velocity[idx_fl_] * wheel_radius_,
        msg->velocity[idx_fr_] * wheel_radius_,
        msg->velocity[idx_rl_] * wheel_radius_,
        msg->velocity[idx_rr_] * wheel_radius_,
    };
    const auto sol = wheel_odom::solve_body_twist(angles, speeds, geom_);

    // ---- 2. STILL (ZUPT) detection ----
    // Body linear speed from the LS solve, plus (with IMU) the bias-removed gyro
    // magnitude. When still, wheels assert ALL six dims are exactly zero — a
    // high-value zero-velocity constraint for the downstream factor graph.
    const double body_speed = std::hypot(sol.vx, sol.vy);
    bool is_still = (body_speed < still_speed_eps_);
    if (enable_imu_ && has_accel_) {
      is_still = is_still && (last_gyro_.norm() < still_gyro_eps_);
    }

    // ---- 2b. choose ω_used + decide the published ωz covariance ----
    const bool can_use_ls_yaw =
        (yaw_source_ == "ls") && (sol.residual < slip_threshold_);
    Eigen::Vector3d omega_used = last_gyro_;  // zero if !enable_imu_
    double omega_z_var;              // variance for the published ωz
    bool   keep_yaw_cross = false;   // keep vx/vy ↔ ωz cross-cov (LS-yaw only)
    if (can_use_ls_yaw) {
      // κ-corrected LS yaw: subtract the speed-proportional curvature bias.
      omega_used.z() = sol.omega_z - yaw_kappa_ * sol.vx;
      omega_z_var    = sol.cov(2, 2);
      keep_yaw_cross = true;
    } else if (yaw_source_ == "gyro") {
      omega_z_var = gyro_yaw_var_;   // gyro spec; independent of the wheel LS
    } else {
      // residual gated us out — keep yaw frozen this tick rather than rolling
      // back to a stale gyro that may also be zero; mark it as untrusted.
      omega_used.z() = 0.0;
      omega_z_var    = cov_no_obs_;
    }
    if (is_still) {
      omega_used.setZero();          // freeze attitude when stopped
    }
    if (init_done_) {
      R_ = R_ * wheel_odom::exp_so3(omega_used * dt);
    }

    // ---- 3. accel-tilt Mahony (yaw kept free) ----
    tilt_applied_ = false;
    if (init_done_ && has_accel_) {
      const double a_norm = last_accel_.norm();
      if (a_norm > 1e-3 && std::abs(a_norm - kGravity) < tilt_accel_band_) {
        const Eigen::Vector3d g_body_meas = last_accel_ / a_norm;
        const Eigen::Vector3d g_body_pred = R_.transpose() * Eigen::Vector3d(0, 0, 1);
        Eigen::Vector3d err = g_body_meas.cross(g_body_pred);
        err.z() = 0.0;
        R_ = R_ * wheel_odom::exp_so3(tilt_kp_ * err * dt);
        tilt_applied_ = true;
      }
    }

    // ---- 4. body → world velocity (zeroed when STILL) ----
    const Eigen::Vector3d v_body =
        is_still ? Eigen::Vector3d::Zero() : Eigen::Vector3d(sol.vx, sol.vy, 0.0);
    const Eigen::Vector3d v_world = R_ * v_body;

    // ---- 5. integrate position ----
    p_ += v_world * dt;

    // ---- 6. FlatZ clamp ----
    if (flatz_enabled_) {
      p_.z() = (1.0 - flatz_alpha_) * p_.z();
    }

    // ---- build the 6×6 twist covariance (order: vx,vy,vz,ωx,ωy,ωz) ----
    const std::array<double, 36> tw_cov =
        make_twist_cov(sol.cov, omega_z_var, keep_yaw_cross, is_still);

    if (diag_fp_) {
      const auto rpy = R_to_rpy(R_);
      std::fprintf(diag_fp_,
          "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
          "%.6f,%.6f,%.6f,%.6f,%.6f,"
          "%.6f,%.6f,%.6f,%d,%d,%d\n",
          t_abs, p_.x(), p_.y(), p_.z(),
          rpy[0], rpy[1], rpy[2],
          v_world.x(), v_world.y(), v_world.z(), v_body.x(), v_body.y(),
          sol.omega_z, omega_used.z(), sol.residual,
          static_cast<int>(tilt_applied_),
          static_cast<int>(can_use_ls_yaw),
          static_cast<int>(is_still));
    }

    publish(msg->header.stamp, v_body, omega_used, sol.residual, tw_cov, is_still);
    ++pub_count_;
  }

  // Map the 3×3 LS solution cov (vx,vy,ωz) into a published 6×6 twist cov in the
  // (vx,vy,vz,ωx,ωy,ωz) order. STILL → floor on all six (wheels assert zero);
  // otherwise vx/vy from the LS, ωz per the yaw-source policy, and the three
  // never-observed dims (vz,ωx,ωy) get ~∞ variance to tell GLIM to ignore them.
  std::array<double, 36> make_twist_cov(const Eigen::Matrix3d& Sz,
                                        double omega_z_var, bool keep_cross,
                                        bool still) const {
    std::array<double, 36> C{};  // zero-initialised
    auto at = [&C](int r, int c) -> double& { return C[r * 6 + c]; };
    if (still) {
      at(0, 0) = floor_v2_; at(1, 1) = floor_v2_; at(2, 2) = floor_v2_;
      at(3, 3) = floor_w2_; at(4, 4) = floor_w2_; at(5, 5) = floor_w2_;
      return C;
    }
    // observed: vx, vy (LS 2×2 block)
    at(0, 0) = Sz(0, 0); at(0, 1) = Sz(0, 1);
    at(1, 0) = Sz(1, 0); at(1, 1) = Sz(1, 1);
    // observed: ωz
    at(5, 5) = omega_z_var;
    if (keep_cross) {  // LS-yaw: keep vx/vy ↔ ωz cross terms
      at(0, 5) = Sz(0, 2); at(5, 0) = Sz(2, 0);
      at(1, 5) = Sz(1, 2); at(5, 1) = Sz(2, 1);
    }
    // never observed by wheels: vz, ωx, ωy
    at(2, 2) = cov_no_obs_; at(3, 3) = cov_no_obs_; at(4, 4) = cov_no_obs_;
    return C;
  }

  void publish(const builtin_interfaces::msg::Time& stamp,
               const Eigen::Vector3d& v_body,
               const Eigen::Vector3d& omega_used,
               double ls_residual,
               const std::array<double, 36>& twist_cov,
               bool is_still) {
    Eigen::Quaterniond q(R_);
    q.normalize();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp         = stamp;
    odom.header.frame_id      = odom_frame_;
    odom.child_frame_id       = base_frame_;
    odom.pose.pose.position.x = p_.x();
    odom.pose.pose.position.y = p_.y();
    odom.pose.pose.position.z = p_.z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    // Twist is reported in child_frame_id (body) per ROS convention. The LS
    // solution is already in body frame; no rotation needed.
    odom.twist.twist.linear.x   = v_body.x();
    odom.twist.twist.linear.y   = v_body.y();
    odom.twist.twist.linear.z   = v_body.z();
    odom.twist.twist.angular.x  = omega_used.x();
    odom.twist.twist.angular.y  = omega_used.y();
    odom.twist.twist.angular.z  = omega_used.z();
    // Real 6×6 twist covariance (vx,vy,vz,ωx,ωy,ωz) — first-class output for the
    // downstream GLIM factor graph (see make_twist_cov / plan §Covariance).
    for (size_t i = 0; i < 36; ++i) odom.twist.covariance[i] = twist_cov[i];

    if (mirror_cov_) {
      // Mirror velocity uncertainty into pose.cov so rviz's Odometry
      // "Covariance" display renders it: (x,y) ellipse from (vx,vy), yaw cone
      // from ωz. pose order (x,y,z,roll,pitch,yaw); twist (vx,vy,vz,ωx,ωy,ωz).
      // Proxy viz only (var is m²/s² shown as if m²); enlarge with the display's
      // Position-Covariance "Scale". z,roll,pitch get a tiny floor so the
      // ellipsoid/cone is well-formed (and flat in the ground plane).
      auto& P = odom.pose.covariance;
      P[0]  = twist_cov[0];  P[1]  = twist_cov[1];   // x-x, x-y  <- vx
      P[6]  = twist_cov[6];  P[7]  = twist_cov[7];   // y-x, y-y  <- vy
      P[5]  = twist_cov[5];  P[30] = twist_cov[30];  // x-yaw     <- vx-ωz
      P[11] = twist_cov[11]; P[31] = twist_cov[31];  // y-yaw     <- vy-ωz
      P[35] = twist_cov[35];                         // yaw-yaw   <- ωz
      P[14] = 1.0e-6; P[21] = 1.0e-6; P[28] = 1.0e-6;  // z, roll, pitch floor
    } else {
      // Diagnostics ride in the (GLIM-unused) pose covariance: [0]=LS residual
      // (m/s), [7]=is_still flag. The trajectory_plotter reads pose.cov[0].
      odom.pose.covariance[0]   = ls_residual;
      odom.pose.covariance[7]   = is_still ? 1.0 : 0.0;
    }
    odom_pub_->publish(odom);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header               = odom.header;
      tf.child_frame_id       = base_frame_;
      tf.transform.translation.x = p_.x();
      tf.transform.translation.y = p_.y();
      tf.transform.translation.z = p_.z();
      tf.transform.rotation   = odom.pose.pose.orientation;
      tf_bc_->sendTransform(tf);
    }
  }

  // -------------------------------------------------------------------------
  // Periodic [diag] summary — the headline tool for localising the latency:
  // per interval it reports how many chassis messages arrived vs how many odom
  // messages were published, and (when ticks were dropped) the reason.
  //   * rx=0 for a long stretch, then a FIRST-message line  -> sensor/discovery
  //     side: the publisher itself is late. Not this node's code.
  //   * rx>0 but odom_pub=0 with dt>max_dt high             -> stream is gappy
  //     (sensor low rate / bad stamps); cross-check the per-message [diag] line.
  //   * rx≈odom_pub                                         -> pipeline healthy.
  // -------------------------------------------------------------------------
  void diag_timer_cb() {
    const uint64_t rx   = rx_count_        - diag_prev_rx_;
    const uint64_t pub  = pub_count_       - diag_prev_pub_;
    const uint64_t d_sz = drop_bad_size_   - diag_prev_drop_size_;
    const uint64_t d_np = drop_dt_nonpos_  - diag_prev_drop_nonpos_;
    const uint64_t d_bg = drop_dt_big_     - diag_prev_drop_big_;
    diag_prev_rx_          = rx_count_;
    diag_prev_pub_         = pub_count_;
    diag_prev_drop_size_   = drop_bad_size_;
    diag_prev_drop_nonpos_ = drop_dt_nonpos_;
    diag_prev_drop_big_    = drop_dt_big_;

    if (rx == 0) {
      const auto now_wall = std::chrono::steady_clock::now();
      if (have_last_chassis_wall_) {
        const double ago =
            std::chrono::duration<double>(now_wall - last_chassis_wall_).count();
        RCLCPP_WARN(get_logger(),
            "[diag] NO chassis messages in last %.1fs (last one %.1fs ago) — "
            "sensor stopped, or topic/QoS mismatch; odom cannot update",
            diag_log_period_sec_, ago);
      } else {
        const double since_start =
            std::chrono::duration<double>(now_wall - node_start_wall_).count();
        RCLCPP_WARN(get_logger(),
            "[diag] still NO chassis message %.1fs after node start — "
            "publisher not up yet, or topic name / QoS mismatch",
            since_start);
      }
      return;
    }

    RCLCPP_INFO(get_logger(),
        "[diag] last %.1fs: rx=%llu odom_pub=%llu dropped[bad_size=%llu "
        "dt<=0=%llu dt>%.2fs=%llu]",
        diag_log_period_sec_,
        static_cast<unsigned long long>(rx),
        static_cast<unsigned long long>(pub),
        static_cast<unsigned long long>(d_sz),
        static_cast<unsigned long long>(d_np),
        max_dt_,
        static_cast<unsigned long long>(d_bg));
  }

  // Direct ZYX (yaw-pitch-roll) extraction. NOT Eigen's eulerAngles(2,1,0):
  // that constrains the first angle to [0, pi] and, for a pure yaw rotation
  // outside that range, returns the equivalent branch roll=pi/pitch=pi with a
  // flipped yaw — which corrupts the logged/plotted yaw for |yaw|>90 deg. The
  // published odom is unaffected (it uses Quaterniond(R_) directly); this only
  // fixes the diagnostic RPY. atan2 keeps roll,yaw in [-pi,pi], pitch in
  // [-pi/2,pi/2] with no spurious flips.
  static Eigen::Vector3d R_to_rpy(const Eigen::Matrix3d& R) {
    const double roll  = std::atan2(R(2, 1), R(2, 2));
    const double pitch = std::atan2(-R(2, 0),
                                    std::sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2)));
    const double yaw   = std::atan2(R(1, 0), R(0, 0));
    return Eigen::Vector3d(roll, pitch, yaw);
  }

  // --- config / geometry ---
  wheel_odom::WheelGeometry geom_;
  double wheel_radius_{1.0};
  std::string yaw_source_;
  std::string odom_frame_, base_frame_;
  bool   publish_tf_{true};
  bool   enable_imu_{false};
  double bias_window_sec_{3.0};
  double tilt_kp_{1.0};
  double tilt_accel_band_{0.5};
  double slip_threshold_{0.5};
  double yaw_kappa_{0.0};
  double cov_no_obs_{1.0e6};
  double gyro_yaw_var_{1.0e-4};
  double still_speed_eps_{0.02};
  double still_gyro_eps_{0.02};
  double floor_v2_{2.5e-5};
  double floor_w2_{1.0e-6};
  bool   mirror_cov_{false};
  double max_dt_{kDefaultMaxDt};
  bool   flatz_enabled_{true};
  double flatz_alpha_{0.05};

  // --- wheel index cache ---
  bool idx_resolved_{false};
  int  idx_fl_{0}, idx_fr_{1}, idx_rl_{2}, idx_rr_{3};

  // --- state ---
  Eigen::Vector3d p_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d R_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d bg_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d R_base_imu_{Eigen::Matrix3d::Identity()};

  // --- IMU → chassis_cb cache ---
  Eigen::Vector3d last_gyro_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_accel_{Eigen::Vector3d::Zero()};
  bool has_accel_{false};

  // --- timing / diagnostics ---
  double last_t_{-1.0};
  bool   tilt_applied_{false};

  // --- arrival / latency diagnostics ---
  std::chrono::steady_clock::time_point node_start_wall_{
      std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_chassis_wall_{};
  bool     have_last_chassis_wall_{false};
  bool     first_chassis_logged_{false};
  bool     first_imu_logged_{false};
  uint64_t rx_count_{0};
  uint64_t pub_count_{0};
  uint64_t drop_bad_size_{0};
  uint64_t drop_dt_nonpos_{0};
  uint64_t drop_dt_big_{0};
  uint64_t diag_prev_rx_{0};
  uint64_t diag_prev_pub_{0};
  uint64_t diag_prev_drop_size_{0};
  uint64_t diag_prev_drop_nonpos_{0};
  uint64_t diag_prev_drop_big_{0};
  double   diag_log_period_sec_{2.0};
  rclcpp::TimerBase::SharedPtr diag_timer_;

  // --- init window ---
  double init_window_start_{-1.0};
  Eigen::Vector3d accum_gyro_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accum_accel_{Eigen::Vector3d::Zero()};
  int  n_static_{0};
  bool init_done_{false};

  // --- ROS2 I/O ---
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr chassis_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr        imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr         odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster>                tf_bc_;

  std::FILE* diag_fp_{nullptr};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOnlyNode>());
  rclcpp::shutdown();
  return 0;
}
