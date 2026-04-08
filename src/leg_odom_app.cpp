/**
 * @file leg_odom_hybrid.cpp
 * @brief C++ Hybrid: ESKF 200Hz front-end + GTSAM smoother back-end.
 *
 * Reads CSV, runs ESKF at 200Hz with FK/ZUPT/FlatZ updates,
 * periodically runs GTSAM smoother to correct gyro bias.
 *
 * Usage:
 *   ./leg_odom_hybrid <input.csv> <output.csv>
 */

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "leg_odometry/state/State.h"
#include "leg_odometry/state/Propagator.h"
#include "leg_odometry/update/UpdaterFK.h"
#include "leg_odometry/update/UpdaterZUPT.h"
#include "leg_odometry/update/UpdaterFlatZ.h"
#include "leg_odometry/smoother/GTSAMSmoother.h"

using namespace leg_odom;

struct Sample {
  double t;
  Eigen::Vector3d accel, gyro;
  Eigen::Vector3d fk_left, fk_right;
  bool contact_left, contact_right;
  Eigen::Vector3d gt_pos;
  Eigen::Vector4d gt_quat;
};

std::vector<Sample> readCSV(const std::string& path) {
  std::vector<Sample> data;
  std::ifstream f(path);
  std::string line;
  std::getline(f, line);  // skip header
  while (std::getline(f, line)) {
    std::istringstream ss(line);
    std::string tok;
    auto next = [&]() -> double {
      std::getline(ss, tok, ',');
      return std::stod(tok);
    };
    Sample s;
    s.t = next();
    s.accel = {next(), next(), next()};
    s.gyro = {next(), next(), next()};
    s.fk_left = {next(), next(), next()};
    s.fk_right = {next(), next(), next()};
    s.contact_left = next() > 0.5;
    s.contact_right = next() > 0.5;
    s.gt_pos = {next(), next(), next()};
    s.gt_quat = {next(), next(), next(), next()};
    data.push_back(s);
  }
  return data;
}

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <input.csv> <output.csv>" << std::endl;
    return 1;
  }

  auto data = readCSV(argv[1]);
  std::cout << "Loaded " << data.size() << " samples" << std::endl;

  // --- Parameters ---
  NoiseParams noise;
  noise.sigma_a = 0.1;
  noise.sigma_g = 0.01;
  noise.sigma_ba = 0.0;       // accel bias locked in ESKF
  noise.sigma_bg = 0.001;     // gyro bias estimated
  noise.sigma_contact = 0.002;
  noise.sigma_swing = 1.0;
  noise.sigma_fk = 0.005;
  noise.sigma_zupt = 0.03;
  noise.sigma_flat_z = 0.001;
  noise.sigma_flat_vz = 0.001;

  // --- ESKF ---
  State state;

  // --- GTSAM Smoother ---
  GTSAMSmoother smoother(
      noise.sigma_fk, noise.sigma_contact, noise.sigma_swing,
      0.005,  // smoother accel bias walk
      0.002,  // smoother gyro bias walk
      noise.sigma_flat_z);
  smoother.window_size_ = 60;
  smoother.opt_interval_ = 20;

  // --- Initialize from first 50 frames ---
  int init_count = 50;
  Eigen::Vector3d avg_accel = Eigen::Vector3d::Zero();
  for (int i = 0; i < init_count && i < (int)data.size(); i++) {
    avg_accel += data[i].accel;
  }
  avg_accel /= init_count;
  state.initialize(avg_accel, data[init_count].fk_left, data[init_count].fk_right);

  // Init smoother preintegrator
  auto current_bias = gtsam::imuBias::ConstantBias(state.b_a, state.b_g);
  auto pim = smoother.create_preintegrator(current_bias);
  int kf_interval = 10;
  int imu_count = 0;
  int bias_corrections = 0;

  // --- Output ---
  std::ofstream out(argv[2]);
  out << "timestamp,est_x,est_y,est_z,est_qx,est_qy,est_qz,est_qw,"
      << "gt_x,gt_y,gt_z,gt_qx,gt_qy,gt_qz,gt_qw" << std::endl;

  double last_t = data[init_count].t;

  // --- Main loop ---
  for (size_t idx = init_count + 1; idx < data.size(); idx++) {
    const auto& s = data[idx];
    double dt = s.t - last_t;
    last_t = s.t;
    if (dt <= 0 || dt > 0.1) continue;

    // === ESKF predict ===
    Propagator::predict(state, noise, s.accel, s.gyro, dt,
                        s.contact_left, s.contact_right);

    // === ESKF updates ===
    UpdaterFK::update(state, noise, s.fk_left, s.fk_right);
    UpdaterZUPT::update(state, noise, s.fk_left, s.fk_right,
                        s.contact_left, s.contact_right);
    UpdaterFlatZ::update(state, noise);

    // Write output
    auto [pos, quat] = state.get_pose();
    out << s.t << ","
        << pos.x() << "," << pos.y() << "," << pos.z() << ","
        << quat.x() << "," << quat.y() << "," << quat.z() << "," << quat.w() << ","
        << s.gt_pos.x() << "," << s.gt_pos.y() << "," << s.gt_pos.z() << ","
        << s.gt_quat.x() << "," << s.gt_quat.y() << "," << s.gt_quat.z() << "," << s.gt_quat.w()
        << std::endl;

    // === GTSAM smoother keyframes ===
    pim->integrateMeasurement(s.accel, s.gyro, dt);
    imu_count++;

    if (imu_count >= kf_interval) {
      gtsam::Pose3 pose(gtsam::Rot3(state.R), gtsam::Point3(state.p));
      gtsam::imuBias::ConstantBias bias(state.b_a, state.b_g);

      KeyframeData kf;
      kf.pose = pose;
      kf.velocity = state.v;
      kf.bias = bias;
      kf.pim = pim;
      kf.fk_left = s.fk_left;
      kf.fk_right = s.fk_right;
      kf.contact_left = s.contact_left;
      kf.contact_right = s.contact_right;

      smoother.add_keyframe(kf);

      // Reset preintegrator
      current_bias = bias;
      pim = smoother.create_preintegrator(current_bias);
      imu_count = 0;

      // Optimize?
      if (smoother.should_optimize()) {
        Eigen::Vector3d opt_ba, opt_bg;
        if (smoother.optimize(opt_ba, opt_bg)) {
          // Blend gyro bias only (alpha=0.05)
          state.b_g = 0.95 * state.b_g + 0.05 * opt_bg;
          bias_corrections++;
        }
      }
    }
  }

  out.close();
  std::cout << "Done! Bias corrections: " << bias_corrections << std::endl;

  // Quick accuracy check
  auto [pos, quat] = state.get_pose();
  auto& last = data.back();
  Eigen::Vector3d est_disp = pos;
  Eigen::Vector3d gt_disp = last.gt_pos - data[0].gt_pos;
  double err = (est_disp.head<2>() - gt_disp.head<2>()).norm();
  double dist = 0;
  for (size_t i = 1; i < data.size(); i++) {
    dist += (data[i].gt_pos.head<2>() - data[i-1].gt_pos.head<2>()).norm();
  }
  std::cout << "XY drift: " << (dist > 0.1 ? err / dist * 100 : 0) << "%" << std::endl;

  return 0;
}
