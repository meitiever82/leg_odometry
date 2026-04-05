#!/usr/bin/env python3
"""离线评估：回放仿真 rosbag，运行 EKF，对比 ground truth。

不需要启动 ROS2 节点，直接读 rosbag + 调用 EKF。
输出：精度指标 + 轨迹对比图。

用法:
  python3 evaluate_sim.py [--data-dir data/sim] [--scenario SCENARIO|all]
"""

import argparse
import os
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

# rosbag2 读取
try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    import rclpy.serialization
    from sensor_msgs.msg import JointState, Imu
    from nav_msgs.msg import Odometry
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    print("Error: ROS2 required for rosbag playback")
    sys.exit(1)

# EKF
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from leg_odometry.ekf import BloeSchEKF
from leg_odometry.kinematics import LegKinematics
from leg_odometry.contact_detector import ContactDetector

# 绘图
try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec
    HAS_PLT = True
except ImportError:
    HAS_PLT = False
    print("Warning: matplotlib not available, skipping plots")


# ============================================================
# 配置
# ============================================================

SCRIPT_DIR = Path(__file__).resolve().parent
PKG_DIR = SCRIPT_DIR.parent
CONFIG_DIR = PKG_DIR / 'config'
URDF_PATH = PKG_DIR.parent / 'finder_lidar_mapping' / 'glim_ros2' / 'urdf' / 'casbot02_7dof_shell.urdf'

# bag 关节名到 URDF 关节名的映射
JOINT_MAPPING = {
    'LJ0': 'left_leg_pelvic_pitch_joint',
    'LJ1': 'left_leg_pelvic_roll_joint',
    'LJ2': 'left_leg_pelvic_yaw_joint',
    'LJ3': 'left_leg_knee_pitch_joint',
    'LJPITCH': 'left_leg_ankle_pitch_joint',
    'LJROLL': 'left_leg_ankle_roll_joint',
    'RJ6': 'right_leg_pelvic_pitch_joint',
    'RJ7': 'right_leg_pelvic_roll_joint',
    'RJ8': 'right_leg_pelvic_yaw_joint',
    'RJ9': 'right_leg_knee_pitch_joint',
    'RJPITCH': 'right_leg_ankle_pitch_joint',
    'RJROLL': 'right_leg_ankle_roll_joint',
}


def load_ekf_config():
    with open(CONFIG_DIR / 'ekf_params.yaml') as f:
        return yaml.safe_load(f)


def load_kinematics():
    with open(URDF_PATH) as f:
        urdf_str = f.read()
    return LegKinematics(
        urdf_str,
        base_link='base_link',
        left_foot_link='left_leg_ankle_roll_link',
        right_foot_link='right_leg_ankle_roll_link',
    )


# ============================================================
# rosbag 读���
# ============================================================

def read_bag(bag_dir: str):
    """读取 rosbag 中所有消息，按时间排序返回。"""
    storage_options = StorageOptions(uri=bag_dir, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    messages = []
    while reader.has_next():
        topic, data, ts = reader.read_next()
        messages.append((topic, data, ts))

    return messages


def deserialize(topic, data):
    if topic == '/joint_states':
        return rclpy.serialization.deserialize_message(data, JointState)
    elif topic == '/imu':
        return rclpy.serialization.deserialize_message(data, Imu)
    elif topic == '/ground_truth/odom':
        return rclpy.serialization.deserialize_message(data, Odometry)
    return None


# ============================================================
# EKF 离线回放
# ============================================================

def run_ekf_offline(bag_dir: str):
    """回放 bag 并运行 EKF，返回估计轨迹和 ground truth。"""

    ekf_cfg = load_ekf_config()
    kinematics = load_kinematics()
    ekf = BloeSchEKF(ekf_cfg.get('ekf', {}))
    contact_cfg = ekf_cfg.get('contact', {})
    contact_det = ContactDetector(
        threshold=contact_cfg.get('effort_threshold', 5.0),
        hysteresis=contact_cfg.get('hysteresis', 1.0),
    )

    messages = read_bag(bag_dir)

    # 结果存储
    est_times = []
    est_positions = []
    est_orientations = []
    gt_times = []
    gt_positions = []
    gt_orientations = []

    # 缓存
    latest_joints = {}
    latest_efforts = {}
    last_imu_ts = None
    init_count = 0
    init_accel_sum = np.zeros(3)
    INIT_FRAMES = 50

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
                if urdf_name and len(msg.position) > i:
                    latest_joints[urdf_name] = msg.position[i]
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

            # 初始化
            if not ekf.initialized:
                init_accel_sum += accel
                init_count += 1
                if init_count >= INIT_FRAMES and latest_joints:
                    avg_accel = init_accel_sum / init_count
                    p_fl = kinematics.fk_left(latest_joints)
                    p_fr = kinematics.fk_right(latest_joints)
                    ekf.initialize(avg_accel, p_fl, p_fr)
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

            # EKF predict + update
            ekf.predict(accel, gyro, dt, contact_l, contact_r)
            if latest_joints:
                p_fl = kinematics.fk_left(latest_joints)
                p_fr = kinematics.fk_right(latest_joints)
                ekf.update(p_fl, p_fr, contact_l, contact_r)

            # 记录
            pos, quat = ekf.get_pose()
            est_times.append(t_sec)
            est_positions.append(pos.copy())
            est_orientations.append(quat.copy())

    return {
        'est_t': np.array(est_times),
        'est_pos': np.array(est_positions),
        'est_ori': np.array(est_orientations),
        'gt_t': np.array(gt_times),
        'gt_pos': np.array(gt_positions),
        'gt_ori': np.array(gt_orientations),
    }


# ============================================================
# 精度指标
# ============================================================

def compute_metrics(result: dict):
    """计算精度指标。"""
    est_pos = result['est_pos']
    gt_pos = result['gt_pos']

    # 对齐时间（简单截取等长）
    n = min(len(est_pos), len(gt_pos))
    est = est_pos[:n].copy()
    gt = gt_pos[:n].copy()

    # 对齐原点：EKF 从 [0,0,0] 开始，GT 从 [0,0,height] 开始
    # 减去各自的初始位置
    est -= est[0]
    gt -= gt[0]

    # 位置误差
    err = est - gt
    err_xy = np.linalg.norm(err[:, :2], axis=1)
    err_z = np.abs(err[:, 2])
    err_3d = np.linalg.norm(err, axis=1)

    # 行走路径长度（用于漂移百分比计算）
    gt_path_length = np.sum(np.linalg.norm(np.diff(gt[:, :2], axis=0), axis=1))

    # 终点误差
    final_err_xy = err_xy[-1] if len(err_xy) > 0 else 0
    final_err_z = err_z[-1] if len(err_z) > 0 else 0
    final_err_3d = err_3d[-1] if len(err_3d) > 0 else 0

    # 姿态误差 (yaw)
    est_ori = result['est_ori'][:n]
    gt_ori = result['gt_ori'][:n]
    yaw_err = []
    for i in range(n):
        r_est = Rotation.from_quat(est_ori[i])
        r_gt = Rotation.from_quat(gt_ori[i])
        r_err = r_gt.inv() * r_est
        euler_err = r_err.as_euler('xyz')
        yaw_err.append(abs(euler_err[2]))
    yaw_err = np.array(yaw_err)

    metrics = {
        'duration_s': result['est_t'][-1] - result['est_t'][0] if len(result['est_t']) > 1 else 0,
        'distance_m': gt_path_length,
        'rmse_xy_m': float(np.sqrt(np.mean(err_xy**2))),
        'rmse_z_m': float(np.sqrt(np.mean(err_z**2))),
        'rmse_3d_m': float(np.sqrt(np.mean(err_3d**2))),
        'max_err_xy_m': float(np.max(err_xy)),
        'max_err_z_m': float(np.max(err_z)),
        'final_err_xy_m': float(final_err_xy),
        'final_err_z_m': float(final_err_z),
        'final_err_3d_m': float(final_err_3d),
        'drift_percent_xy': float(final_err_xy / gt_path_length * 100) if gt_path_length > 0.1 else float('nan'),
        'rmse_yaw_deg': float(np.degrees(np.sqrt(np.mean(yaw_err**2)))),
        'max_yaw_err_deg': float(np.degrees(np.max(yaw_err))),
    }

    return metrics, err_xy, err_z, err_3d, yaw_err


# ============================================================
# 绘图
# ============================================================

def plot_results(result: dict, metrics: dict, err_xy, err_z, yaw_err,
                 scenario_name: str, output_dir: str):
    """生成评估图表。"""
    if not HAS_PLT:
        return

    est_pos = result['est_pos']
    gt_pos = result['gt_pos']
    n = min(len(est_pos), len(gt_pos))
    est = est_pos[:n]
    gt = gt_pos[:n]
    t = result['est_t'][:n] - result['est_t'][0]

    fig = plt.figure(figsize=(16, 12))
    fig.suptitle(f'{scenario_name}  |  Distance: {metrics["distance_m"]:.1f}m  |  '
                 f'RMSE(XY): {metrics["rmse_xy_m"]*100:.1f}cm  |  '
                 f'Drift: {metrics.get("drift_percent_xy", float("nan")):.2f}%',
                 fontsize=13, fontweight='bold')

    gs = GridSpec(3, 2, figure=fig, hspace=0.35, wspace=0.3)

    # 1. XY 轨迹
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(gt[:, 0], gt[:, 1], 'b-', linewidth=2, label='Ground Truth', alpha=0.7)
    ax1.plot(est[:, 0], est[:, 1], 'r--', linewidth=1.5, label='EKF Estimate')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('XY Trajectory')
    ax1.legend()
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)

    # 2. Z 高度
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(t, gt[:, 2], 'b-', linewidth=2, label='GT', alpha=0.7)
    ax2.plot(t, est[:, 2], 'r--', linewidth=1.5, label='EKF')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Z (m)')
    ax2.set_title('Height')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # 3. XY 误差
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(t, err_xy * 100, 'r-', linewidth=1)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('XY Error (cm)')
    ax3.set_title(f'XY Error  (RMSE: {metrics["rmse_xy_m"]*100:.1f}cm, Max: {metrics["max_err_xy_m"]*100:.1f}cm)')
    ax3.grid(True, alpha=0.3)

    # 4. Z 误差
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(t, err_z * 100, 'g-', linewidth=1)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Z Error (cm)')
    ax4.set_title(f'Z Error  (RMSE: {metrics["rmse_z_m"]*100:.1f}cm, Max: {metrics["max_err_z_m"]*100:.1f}cm)')
    ax4.grid(True, alpha=0.3)

    # 5. Yaw 误差
    ax5 = fig.add_subplot(gs[2, 0])
    ax5.plot(t, np.degrees(yaw_err), 'm-', linewidth=1)
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Yaw Error (°)')
    ax5.set_title(f'Yaw Error  (RMSE: {metrics["rmse_yaw_deg"]:.2f}°, Max: {metrics["max_yaw_err_deg"]:.2f}°)')
    ax5.grid(True, alpha=0.3)

    # 6. 3D XYZ 对比
    ax6 = fig.add_subplot(gs[2, 1])
    for axis_i, (label, color) in enumerate(zip(['X', 'Y', 'Z'], ['r', 'g', 'b'])):
        ax6.plot(t, gt[:, axis_i], f'{color}-', linewidth=1.5, alpha=0.5, label=f'GT {label}')
        ax6.plot(t, est[:, axis_i], f'{color}--', linewidth=1, label=f'Est {label}')
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('Position (m)')
    ax6.set_title('XYZ Components')
    ax6.legend(ncol=3, fontsize=8)
    ax6.grid(True, alpha=0.3)

    fig_path = os.path.join(output_dir, f'{scenario_name}.png')
    plt.savefig(fig_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Plot: {fig_path}")


# ============================================================
# 主程��
# ============================================================

def main():
    parser = argparse.ArgumentParser(description='评估 leg odometry EKF')
    parser.add_argument('--data-dir', '-d', default='data/sim',
                        help='仿真数据目录 (default: data/sim)')
    parser.add_argument('--scenario', '-s', default='all',
                        help='场景名 (或 all)')
    parser.add_argument('--output-dir', '-o', default='data/sim/results',
                        help='结果输出目录')
    args = parser.parse_args()

    data_dir = os.path.join(os.getcwd(), args.data_dir)
    output_dir = os.path.join(os.getcwd(), args.output_dir)
    os.makedirs(output_dir, exist_ok=True)

    if args.scenario == 'all':
        # 找到所有包含 metadata.yaml 的子目录
        scenarios = sorted([
            d for d in os.listdir(data_dir)
            if os.path.isdir(os.path.join(data_dir, d))
            and d != 'results'
        ])
    else:
        scenarios = [args.scenario]

    print(f"Evaluating {len(scenarios)} scenarios")
    print(f"Output: {output_dir}")
    print()

    all_metrics = {}

    for name in scenarios:
        bag_dir = os.path.join(data_dir, name)
        if not os.path.isdir(bag_dir):
            print(f"  [{name}] SKIP - not found")
            continue

        print(f"[{name}]")
        try:
            result = run_ekf_offline(bag_dir)
            metrics, err_xy, err_z, err_3d, yaw_err = compute_metrics(result)
            all_metrics[name] = metrics

            print(f"  Distance:  {metrics['distance_m']:.1f} m")
            print(f"  RMSE XY:   {metrics['rmse_xy_m']*100:.2f} cm")
            print(f"  RMSE Z:    {metrics['rmse_z_m']*100:.2f} cm")
            print(f"  Final XY:  {metrics['final_err_xy_m']*100:.2f} cm")
            print(f"  Drift XY:  {metrics.get('drift_percent_xy', float('nan')):.3f} %")
            print(f"  RMSE Yaw:  {metrics['rmse_yaw_deg']:.2f}°")

            plot_results(result, metrics, err_xy, err_z, yaw_err, name, output_dir)

        except Exception as e:
            print(f"  ERROR: {e}")
            import traceback
            traceback.print_exc()
        print()

    # 汇总表
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

        # 保存 CSV
        csv_path = os.path.join(output_dir, 'summary.csv')
        with open(csv_path, 'w') as f:
            keys = list(next(iter(all_metrics.values())).keys())
            f.write('scenario,' + ','.join(keys) + '\n')
            for name, m in all_metrics.items():
                vals = [str(m[k]) for k in keys]
                f.write(name + ',' + ','.join(vals) + '\n')
        print(f"\nSaved: {csv_path}")


if __name__ == '__main__':
    main()
