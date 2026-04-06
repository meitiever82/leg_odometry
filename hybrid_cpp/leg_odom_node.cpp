/**
 * @file leg_odom_node.cpp
 * @brief ROS2 C++ node: ESKF Hybrid leg odometry.
 *
 * Subscribes: /joint_states (JointState), /imu (Imu)
 * Publishes:  /leg_odometry (Odometry), TF odom->base_link
 * Parameters: urdf from /robot_description or file path
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "state/State.h"
#include "state/Propagator.h"
#include "state/Kinematics.h"
#include "update/UpdaterFK.h"
#include "update/UpdaterZUPT.h"
#include "update/UpdaterFlatZ.h"
#include "smoother/GTSAMSmoother.h"

using namespace leg_odom;

class LegOdomNode : public rclcpp::Node {
 public:
  LegOdomNode() : Node("leg_odom_hybrid") {
    // Parameters
    declare_parameter("urdf_path", "");
    declare_parameter("robot_description", "");

    declare_parameter("accel_noise", 0.1);
    declare_parameter("gyro_noise", 0.01);
    declare_parameter("accel_bias_walk", 0.0);
    declare_parameter("gyro_bias_walk", 0.001);
    declare_parameter("foot_contact_noise", 0.002);
    declare_parameter("foot_swing_noise", 1.0);
    declare_parameter("fk_position_noise", 0.005);
    declare_parameter("zupt_noise", 0.03);
    declare_parameter("flat_z_noise", 0.001);
    declare_parameter("flat_vz_noise", 0.001);

    declare_parameter("effort_threshold", 5.0);
    declare_parameter("effort_hysteresis", 1.0);
    declare_parameter("effort_joint_left", "LJPITCH");
    declare_parameter("effort_joint_right", "RJPITCH");

    declare_parameter("init_frames", 50);
    declare_parameter("smoother_enabled", true);
    declare_parameter("smoother_window", 60);
    declare_parameter("smoother_interval", 20);
    declare_parameter("smoother_kf_interval", 10);
    declare_parameter("smoother_alpha", 0.05);

    // Load parameters
    noise_.sigma_a = get_parameter("accel_noise").as_double();
    noise_.sigma_g = get_parameter("gyro_noise").as_double();
    noise_.sigma_ba = get_parameter("accel_bias_walk").as_double();
    noise_.sigma_bg = get_parameter("gyro_bias_walk").as_double();
    noise_.sigma_contact = get_parameter("foot_contact_noise").as_double();
    noise_.sigma_swing = get_parameter("foot_swing_noise").as_double();
    noise_.sigma_fk = get_parameter("fk_position_noise").as_double();
    noise_.sigma_zupt = get_parameter("zupt_noise").as_double();
    noise_.sigma_flat_z = get_parameter("flat_z_noise").as_double();
    noise_.sigma_flat_vz = get_parameter("flat_vz_noise").as_double();

    effort_threshold_ = get_parameter("effort_threshold").as_double();
    effort_hysteresis_ = get_parameter("effort_hysteresis").as_double();
    effort_joint_left_ = get_parameter("effort_joint_left").as_string();
    effort_joint_right_ = get_parameter("effort_joint_right").as_string();
    init_frames_ = get_parameter("init_frames").as_int();
    smoother_enabled_ = get_parameter("smoother_enabled").as_bool();
    smoother_alpha_ = get_parameter("smoother_alpha").as_double();
    kf_interval_ = get_parameter("smoother_kf_interval").as_int();

    // Init kinematics
    std::string urdf_path = get_parameter("urdf_path").as_string();
    std::string robot_desc = get_parameter("robot_description").as_string();
    if (!urdf_path.empty()) {
      if (!kin_.init(urdf_path)) {
        RCLCPP_FATAL(get_logger(), "Failed to load URDF: %s", urdf_path.c_str());
        return;
      }
    } else {
      RCLCPP_WARN(get_logger(), "No urdf_path set, will try /robot_description parameter");
      // TODO: load from robot_description string parameter
    }

    // Init contact detector
    contact_det_ = std::make_unique<ContactDetector>(effort_threshold_, effort_hysteresis_);

    // Init joint mapping
    joint_mapping_ = default_joint_mapping();

    // Init smoother
    if (smoother_enabled_) {
      smoother_ = std::make_unique<GTSAMSmoother>(
          noise_.sigma_fk, noise_.sigma_contact, noise_.sigma_swing,
          0.005, 0.002, noise_.sigma_flat_z);
      smoother_->window_size_ = get_parameter("smoother_window").as_int();
      smoother_->opt_interval_ = get_parameter("smoother_interval").as_int();
    }

    // TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Subscribers
    auto qos = rclcpp::SensorDataQoS();
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu", qos, std::bind(&LegOdomNode::imu_callback, this, std::placeholders::_1));
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", qos, std::bind(&LegOdomNode::joint_callback, this, std::placeholders::_1));

    // Publisher
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/leg_odometry", 10);

    RCLCPP_INFO(get_logger(), "Leg odometry hybrid node started");
  }

 private:
  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (size_t i = 0; i < msg->name.size(); i++) {
      const auto& name = msg->name[i];

      // Store effort for contact detection
      if (i < msg->effort.size()) {
        latest_efforts_[name] = msg->effort[i];
      }

      // Map to URDF name and store position
      auto it = joint_mapping_.find(name);
      if (it != joint_mapping_.end() && i < msg->position.size()) {
        latest_joints_[it->second] = msg->position[i];
      }
    }
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    Eigen::Vector3d accel(msg->linear_acceleration.x,
                          msg->linear_acceleration.y,
                          msg->linear_acceleration.z);
    Eigen::Vector3d gyro(msg->angular_velocity.x,
                         msg->angular_velocity.y,
                         msg->angular_velocity.z);
    auto stamp = msg->header.stamp;

    // Initialization（方差检测确认静止）
    if (!state_.initialized) {
      init_accel_sum_ += accel;
      init_gyro_buf_.push_back(gyro.norm());
      init_count_++;
      if (init_count_ >= init_frames_ && !latest_joints_.empty()) {
        // 检查 gyro 方差确认静止
        double sum = 0, sum2 = 0;
        int n = init_gyro_buf_.size();
        for (auto v : init_gyro_buf_) { sum += v; sum2 += v * v; }
        double gyro_std = std::sqrt(sum2 / n - (sum / n) * (sum / n));
        if (gyro_std > 0.05) {
          return;  // 还在动，继续等
        }
        Eigen::Vector3d avg = init_accel_sum_ / init_count_;
        auto fl = kin_.fk_left(latest_joints_);
        auto fr = kin_.fk_right(latest_joints_);
        state_.initialize(avg, fl, fr);

        if (smoother_enabled_) {
          current_bias_ = gtsam::imuBias::ConstantBias(state_.b_a, state_.b_g);
          pim_ = smoother_->create_preintegrator(current_bias_);
        }
        RCLCPP_INFO(get_logger(), "ESKF initialized from %d IMU frames (gyro_std=%.4f)", init_count_, gyro_std);
      }
      return;
    }

    // Compute dt
    double t_sec = stamp.sec + stamp.nanosec * 1e-9;
    if (last_imu_time_ < 0) {
      last_imu_time_ = t_sec;
      return;
    }
    double dt = t_sec - last_imu_time_;
    last_imu_time_ = t_sec;
    if (dt <= 0 || dt > 0.1) return;

    // Contact detection
    double eff_l = 0, eff_r = 0;
    auto it_l = latest_efforts_.find(effort_joint_left_);
    if (it_l != latest_efforts_.end()) eff_l = it_l->second;
    auto it_r = latest_efforts_.find(effort_joint_right_);
    if (it_r != latest_efforts_.end()) eff_r = it_r->second;
    auto [cl, cr] = contact_det_->update(eff_l, eff_r);

    // === ESKF ===
    Propagator::predict(state_, noise_, accel, gyro, dt, cl, cr);

    if (!latest_joints_.empty()) {
      auto fl = kin_.fk_left(latest_joints_);
      auto fr = kin_.fk_right(latest_joints_);

      UpdaterFK::update(state_, noise_, fl, fr);
      UpdaterZUPT::update(state_, noise_, fl, fr, cl, cr);
      UpdaterFlatZ::update(state_, noise_);

      // === GTSAM smoother keyframes ===
      if (smoother_enabled_ && pim_) {
        pim_->integrateMeasurement(accel, gyro, dt);
        imu_count_++;

        if (imu_count_ >= kf_interval_) {
          gtsam::Pose3 pose(gtsam::Rot3(state_.R), gtsam::Point3(state_.p));
          gtsam::imuBias::ConstantBias bias(state_.b_a, state_.b_g);

          KeyframeData kf;
          kf.pose = pose;
          kf.velocity = state_.v;
          kf.bias = bias;
          kf.pim = pim_;
          kf.fk_left = fl;
          kf.fk_right = fr;
          kf.contact_left = cl;
          kf.contact_right = cr;
          smoother_->add_keyframe(kf);

          current_bias_ = bias;
          pim_ = smoother_->create_preintegrator(current_bias_);
          imu_count_ = 0;

          if (smoother_->should_optimize()) {
            Eigen::Vector3d opt_ba, opt_bg;
            if (smoother_->optimize(opt_ba, opt_bg)) {
              state_.b_g = (1.0 - smoother_alpha_) * state_.b_g + smoother_alpha_ * opt_bg;
            }
          }
        }
      }
    }

    // === Publish ===
    publish_odometry(stamp);
  }

  void publish_odometry(const builtin_interfaces::msg::Time& stamp) {
    auto [pos, quat] = state_.get_pose();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = pos.x();
    odom.pose.pose.position.y = pos.y();
    odom.pose.pose.position.z = pos.z();
    odom.pose.pose.orientation.x = quat.x();
    odom.pose.pose.orientation.y = quat.y();
    odom.pose.pose.orientation.z = quat.z();
    odom.pose.pose.orientation.w = quat.w();
    odom.twist.twist.linear.x = state_.v.x();
    odom.twist.twist.linear.y = state_.v.y();
    odom.twist.twist.linear.z = state_.v.z();
    odom_pub_->publish(odom);

    // TF
    geometry_msgs::msg::TransformStamped t;
    t.header = odom.header;
    t.child_frame_id = "base_link_leg_odom";
    t.transform.translation.x = pos.x();
    t.transform.translation.y = pos.y();
    t.transform.translation.z = pos.z();
    t.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(t);
  }

  // State
  State state_;
  NoiseParams noise_;
  LegKinematics kin_;
  std::unique_ptr<ContactDetector> contact_det_;
  std::map<std::string, std::string> joint_mapping_;
  std::map<std::string, double> latest_joints_;
  std::map<std::string, double> latest_efforts_;

  // Initialization
  Eigen::Vector3d init_accel_sum_ = Eigen::Vector3d::Zero();
  std::vector<double> init_gyro_buf_;
  int init_count_ = 0;
  int init_frames_ = 50;
  double last_imu_time_ = -1;

  // Contact
  double effort_threshold_, effort_hysteresis_;
  std::string effort_joint_left_, effort_joint_right_;

  // Smoother
  bool smoother_enabled_ = true;
  double smoother_alpha_ = 0.05;
  int kf_interval_ = 10;
  int imu_count_ = 0;
  std::unique_ptr<GTSAMSmoother> smoother_;
  gtsam::imuBias::ConstantBias current_bias_;
  std::shared_ptr<gtsam::PreintegratedImuMeasurements> pim_;

  // ROS2
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LegOdomNode>());
  rclcpp::shutdown();
  return 0;
}
