#!/usr/bin/env python3
"""ESKF + GTSAM 滑窗平滑器 混合评估。

ESKF 前端 200Hz 实时输出 + GTSAM 后端周期性 bias 校正。

用法:
  python3 evaluate_hybrid.py [--data-dir data/sim] [--scenario SCENARIO|all]
"""

import argparse
import os
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from evaluate_sim import (
    read_bag, deserialize, compute_metrics, plot_results,
    JOINT_MAPPING, load_ekf_config, load_kinematics,
)
from leg_odometry.ekf import BloeSchEKF
from leg_odometry.smoother import SlidingWindowSmoother, KeyframeData
from leg_odometry.contact_detector import ContactDetector

import gtsam


def run_hybrid_offline(bag_dir: str):
    """ESKF + 滑窗平滑器 离线回放。"""

    ekf_cfg = load_ekf_config()
    kinematics = load_kinematics()

    # ESKF 前端 — 解锁 bias（smoother 会校正）
    ekf_params = dict(ekf_cfg.get('ekf', {}))
    # accel_bias_walk 从配置文件读取（现在有 Z 约束 + smoother 兜底，可以解锁）
    ekf_params['gyro_bias_walk'] = 0.001  # gyro bias 保持小量在线估计
    ekf = BloeSchEKF(ekf_params)

    # GTSAM 滑窗后端
    smoother_params = dict(ekf_cfg.get('ekf', {}))
    smoother_params['smoother_accel_bias_walk'] = 0.005
    smoother_params['smoother_gyro_bias_walk'] = 0.002
    smoother_params['smoother_window_size'] = 60
    smoother_params['smoother_interval'] = 20
    smoother = SlidingWindowSmoother(smoother_params)

    contact_cfg = ekf_cfg.get('contact', {})
    contact_det = ContactDetector(
        threshold=contact_cfg.get('effort_threshold', 5.0),
        hysteresis=contact_cfg.get('hysteresis', 1.0),
    )

    messages = read_bag(bag_dir)

    est_times, est_positions, est_orientations = [], [], []
    gt_times, gt_positions, gt_orientations = [], [], []

    latest_joints = {}
    latest_joint_vels = {}
    latest_efforts = {}
    last_imu_ts = None
    init_count = 0
    init_accel_sum = np.zeros(3)
    init_gyro_sum = np.zeros(3)
    init_accel_buf = []
    init_gyro_buf = []
    INIT_FRAMES = 50

    # 关键帧预积分器
    kf_interval = 10  # 每 10 帧 IMU 一个关键帧
    imu_count = 0
    current_bias = gtsam.imuBias.ConstantBias()
    pim = None

    # 统计
    bias_corrections = 0

    for topic, data, ts_ns in messages:
        msg = deserialize(topic, data)
        if msg is None:
            continue

        if topic == '/ground_truth/odom':
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            gt_times.append(ts_ns * 1e-9)
            gt_positions.append([p.x, p.y, p.z])
            gt_orientations.append([o.x, o.y, o.z, o.w])
            continue

        if topic == '/joint_states':
            for i, name in enumerate(msg.name):
                if len(msg.effort) > i:
                    latest_efforts[name] = msg.effort[i]
                urdf_name = JOINT_MAPPING.get(name)
                if urdf_name:
                    if len(msg.position) > i:
                        latest_joints[urdf_name] = msg.position[i]
                    if len(msg.velocity) > i:
                        latest_joint_vels[urdf_name] = msg.velocity[i]
            continue

        if topic == '/imu':
            accel = np.array([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ])
            gyro = np.array([
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ])

            # 初始化（方差检测确认静止）
            if not ekf.initialized:
                init_accel_sum += accel
                init_gyro_sum += gyro
                init_accel_buf.append(accel.copy())
                init_gyro_buf.append(gyro.copy())
                init_count += 1
                if init_count >= INIT_FRAMES and latest_joints:
                    # 检查是否真正静止：gyro 方差小
                    gyro_arr = np.array(init_gyro_buf[-INIT_FRAMES:])
                    gyro_std = np.std(np.linalg.norm(gyro_arr, axis=1))
                    if gyro_std > 0.05:
                        # 还在动，继续等
                        continue
                    avg_accel = init_accel_sum / init_count
                    p_fl = kinematics.fk_left(latest_joints)
                    p_fr = kinematics.fk_right(latest_joints)
                    ekf.initialize(avg_accel, p_fl, p_fr)

                    # 初始化 smoother 预积分器
                    current_bias = gtsam.imuBias.ConstantBias(
                        ekf.b_a, ekf.b_g)
                    pim = smoother.create_preintegrator(current_bias)
                continue

            # dt
            t_sec = ts_ns * 1e-9
            if last_imu_ts is None:
                last_imu_ts = t_sec
                continue
            dt = t_sec - last_imu_ts
            last_imu_ts = t_sec
            if dt <= 0 or dt > 0.1:
                continue

            # 接触检测
            effort_l = latest_efforts.get('LJPITCH', 0.0)
            effort_r = latest_efforts.get('RJPITCH', 0.0)
            contact_l, contact_r = contact_det.update(effort_l, effort_r)

            # === ESKF predict + update (200Hz) ===
            ekf.predict(accel, gyro, dt, contact_l, contact_r)
            if latest_joints:
                p_fl = kinematics.fk_left(latest_joints)
                p_fr = kinematics.fk_right(latest_joints)

                # 关节速度 Jacobian 方式在真实数据上因编码器噪声放大反而更差
                # 保持有限差分 dFK/dt（天然低通滤波）
                ekf.update(p_fl, p_fr, contact_l, contact_r)

            # 记录 ESKF 输出
            pos, quat = ekf.get_pose()
            est_times.append(t_sec)
            est_positions.append(pos.copy())
            est_orientations.append(quat.copy())

            # === 滑窗 smoother 关键帧 ===
            if pim is not None:
                pim.integrateMeasurement(accel, gyro, dt)
                imu_count += 1

                if imu_count >= kf_interval and latest_joints:
                    # 创建关键帧
                    pose, vel, bias = ekf.get_state_for_smoother()
                    p_fl = kinematics.fk_left(latest_joints)
                    p_fr = kinematics.fk_right(latest_joints)

                    kf = KeyframeData(
                        pose=pose, velocity=vel, bias=bias,
                        pim=pim,
                        fk_left=p_fl, fk_right=p_fr,
                        contact_left=contact_l, contact_right=contact_r,
                        timestamp=t_sec,
                    )
                    smoother.add_keyframe(kf)

                    # 重置预积分器
                    current_bias = bias
                    pim = smoother.create_preintegrator(current_bias)
                    imu_count = 0

                    # 检查是否需要优化
                    if smoother.should_optimize():
                        success = smoother.optimize()
                        if success:
                            b_a, b_g = smoother.get_bias_correction()
                            if b_g is not None:
                                # 只注入 gyro bias（改善旋转）
                                ekf.set_bias(ekf.b_a, b_g, alpha=0.05)
                                bias_corrections += 1

    if bias_corrections > 0:
        print(f"  Bias corrections: {bias_corrections}")

    return {
        'est_t': np.array(est_times),
        'est_pos': np.array(est_positions),
        'est_ori': np.array(est_orientations),
        'gt_t': np.array(gt_times),
        'gt_pos': np.array(gt_positions),
        'gt_ori': np.array(gt_orientations),
    }


def main():
    parser = argparse.ArgumentParser(description='ESKF+GTSAM hybrid 评估')
    parser.add_argument('--data-dir', '-d', default='data/sim')
    parser.add_argument('--scenario', '-s', default='all')
    parser.add_argument('--output-dir', '-o', default='data/sim/results_hybrid')
    args = parser.parse_args()

    data_dir = os.path.join(os.getcwd(), args.data_dir)
    output_dir = os.path.join(os.getcwd(), args.output_dir)
    os.makedirs(output_dir, exist_ok=True)

    if args.scenario == 'all':
        scenarios = sorted([
            d for d in os.listdir(data_dir)
            if os.path.isdir(os.path.join(data_dir, d))
            and not d.startswith('results')
        ])
    else:
        scenarios = [args.scenario]

    print(f"[Hybrid ESKF+GTSAM] Evaluating {len(scenarios)} scenarios")
    print()

    all_metrics = {}
    for name in scenarios:
        bag_dir = os.path.join(data_dir, name)
        if not os.path.isdir(bag_dir):
            continue

        print(f"[{name}]")
        try:
            result = run_hybrid_offline(bag_dir)
            metrics, err_xy, err_z, err_3d, yaw_err = compute_metrics(result)
            all_metrics[name] = metrics

            print(f"  Distance:  {metrics['distance_m']:.1f} m")
            print(f"  RMSE XY:   {metrics['rmse_xy_m']*100:.2f} cm")
            print(f"  RMSE Z:    {metrics['rmse_z_m']*100:.2f} cm")
            print(f"  Drift XY:  {metrics.get('drift_percent_xy', float('nan')):.3f} %")
            print(f"  RMSE Yaw:  {metrics['rmse_yaw_deg']:.2f}°")

            plot_results(result, metrics, err_xy, err_z, yaw_err,
                         f'{name}_hybrid', output_dir)
        except Exception as e:
            print(f"  ERROR: {e}")
            import traceback
            traceback.print_exc()
        print()

    if all_metrics:
        print("=" * 90)
        print(f"{'Scenario':<20} {'Dist(m)':>8} {'RMSE_XY(cm)':>12} {'RMSE_Z(cm)':>11} "
              f"{'Final_XY(cm)':>13} {'Drift(%)':>9} {'RMSE_Yaw(°)':>12}")
        print("-" * 90)
        for name, m in all_metrics.items():
            drift = m.get('drift_percent_xy', float('nan'))
            drift_str = f"{drift:.3f}" if not np.isnan(drift) else "N/A"
            print(f"{name:<20} {m['distance_m']:>8.1f} {m['rmse_xy_m']*100:>12.2f} "
                  f"{m['rmse_z_m']*100:>11.2f} {m['final_err_xy_m']*100:>13.2f} "
                  f"{drift_str:>9} {m['rmse_yaw_deg']:>12.2f}")
        print("=" * 90)

        csv_path = os.path.join(output_dir, 'summary_hybrid.csv')
        with open(csv_path, 'w') as f:
            keys = list(next(iter(all_metrics.values())).keys())
            f.write('scenario,' + ','.join(keys) + '\n')
            for name, m in all_metrics.items():
                vals = [str(m[k]) for k in keys]
                f.write(name + ',' + ','.join(vals) + '\n')
        print(f"\nSaved: {csv_path}")


if __name__ == '__main__':
    main()
