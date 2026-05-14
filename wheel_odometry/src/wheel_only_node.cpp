/**
 * @file wheel_only_node.cpp
 * @brief Swerve-only wheel odometry ROS2 node.
 *
 * State:    p ∈ R³, R ∈ SO(3), gyro_bias ∈ R³.
 * Pipeline (driven by /robot/wheel_status, sensor_msgs/JointState):
 *   1. solve LS:                     (vx_b, vy_b, ω_z_LS, residual)
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
#include <cmath>
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
constexpr double kMaxDt   = 0.2;   // s, reject stale chassis dt

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
    declare_parameter("wheelbase",   0.6);
    declare_parameter("track",       0.5);
    declare_parameter("wheel_radius", 1.0);

    declare_parameter("bias_window_sec", 3.0);
    declare_parameter("tilt_kp",         1.0);
    declare_parameter("tilt_accel_band", 0.5);

    declare_parameter("yaw_source",     std::string("ls"));
    declare_parameter("slip_threshold", 0.5);

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

    geom_ = wheel_odom::WheelGeometry::from_LW(
        get_parameter("wheelbase").as_double(),
        get_parameter("track").as_double());
    wheel_radius_   = get_parameter("wheel_radius").as_double();
    bias_window_sec_= get_parameter("bias_window_sec").as_double();
    tilt_kp_        = get_parameter("tilt_kp").as_double();
    tilt_accel_band_= get_parameter("tilt_accel_band").as_double();
    yaw_source_     = get_parameter("yaw_source").as_string();
    slip_threshold_ = get_parameter("slip_threshold").as_double();
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
            "wz_ls,wz_used,ls_residual,tilt_applied,used_ls_yaw\n");
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

    p_.setZero();
    R_.setIdentity();
    bg_.setZero();

    RCLCPP_INFO(get_logger(),
        "wheel_only_odom: L=%.3f W=%.3f r=%.3f  imu=%s yaw_source=%s "
        "slip_thr=%.3f  flatz=%s α=%.3f  chassis=%s odom=%s",
        get_parameter("wheelbase").as_double(),
        get_parameter("track").as_double(), wheel_radius_,
        enable_imu_ ? "on" : "off", yaw_source_.c_str(), slip_threshold_,
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
    if (msg->position.size() < 4 || msg->velocity.size() < 4) {
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
    if (last_t_ < 0.0) { last_t_ = t_abs; return; }
    const double dt = t_abs - last_t_;
    last_t_ = t_abs;
    if (dt <= 0.0 || dt > kMaxDt) return;

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

    // ---- 2. choose ω_used and propagate R ----
    const bool can_use_ls_yaw =
        (yaw_source_ == "ls") && (sol.residual < slip_threshold_);
    Eigen::Vector3d omega_used = last_gyro_;  // zero if !enable_imu_
    if (can_use_ls_yaw) {
      omega_used.z() = sol.omega_z;
    } else if (yaw_source_ == "ls") {
      // residual gated us out — keep yaw frozen this tick rather than rolling
      // back to a stale gyro that may also be zero.
      omega_used.z() = 0.0;
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

    // ---- 4. body → world velocity ----
    const Eigen::Vector3d v_body(sol.vx, sol.vy, 0.0);
    const Eigen::Vector3d v_world = R_ * v_body;

    // ---- 5. integrate position ----
    p_ += v_world * dt;

    // ---- 6. FlatZ clamp ----
    if (flatz_enabled_) {
      p_.z() = (1.0 - flatz_alpha_) * p_.z();
    }

    if (diag_fp_) {
      const auto rpy = R_to_rpy(R_);
      std::fprintf(diag_fp_,
          "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
          "%.6f,%.6f,%.6f,%.6f,%.6f,"
          "%.6f,%.6f,%.6f,%d,%d\n",
          t_abs, p_.x(), p_.y(), p_.z(),
          rpy[0], rpy[1], rpy[2],
          v_world.x(), v_world.y(), v_world.z(), v_body.x(), v_body.y(),
          sol.omega_z, omega_used.z(), sol.residual,
          static_cast<int>(tilt_applied_),
          static_cast<int>(can_use_ls_yaw));
    }

    publish(msg->header.stamp, v_world, omega_used);
  }

  void publish(const builtin_interfaces::msg::Time& stamp,
               const Eigen::Vector3d& v_world,
               const Eigen::Vector3d& omega_used) {
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
    odom.twist.twist.linear.x   = v_world.x();
    odom.twist.twist.linear.y   = v_world.y();
    odom.twist.twist.linear.z   = v_world.z();
    odom.twist.twist.angular.x  = omega_used.x();
    odom.twist.twist.angular.y  = omega_used.y();
    odom.twist.twist.angular.z  = omega_used.z();
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

  static Eigen::Vector3d R_to_rpy(const Eigen::Matrix3d& R) {
    const Eigen::Vector3d ea = R.eulerAngles(2, 1, 0);
    return Eigen::Vector3d(ea[2], ea[1], ea[0]);
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
