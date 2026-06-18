#!/usr/bin/env python3
# lwt/extract.py —— [系统 python3 + rosbag2_py] 把一个 episode 的 bag 抽成 .npz。
# 用法: source /opt/ros/jazzy/setup.bash; python3 lwt/extract.py <episode_dir> <out.npz> [--storage mcap]
#   <episode_dir> 含 rosbag2/(仿真)或直接是真机 bag 目录;真机用 --storage sqlite3 --no-gt
#   IMU 话题:仿真 /sim/imu(base z-up),真机 /rslidar_imu_data(倾斜系)。--imu-topic 覆盖;
#   不给则自动探测(优先 /sim/imu,否则 /rslidar_imu_data)。
#   phase-2:写入 imu_yawrate(N,1)=重力投影 base yaw-rate(+CCW world-up,rad/s),对齐到 t_wheel。
#   --imu-sign 全局符号:sim=+1(up≈+Z),real 经 verify 回归常需 -1(加速度计报重力向下)。
import argparse, sys, numpy as np, yaml
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from lwt.imu_yawrate import project_yawrate, debias_yawrate

WHEEL_KEYS = ("front_left", "front_right", "rear_left", "rear_right")
IMU_TOPICS = ("/sim/imu", "/rslidar_imu_data")

def _stamp(h): return h.stamp.sec + h.stamp.nanosec*1e-9

def read(bag, storage, imu_topic):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage),
           rosbag2_py.ConverterOptions("", ""))
    wt, wp, wv, it, ig, ia, ot, ovx, ovy, owz = ([] for _ in range(10))
    idx = None
    while r.has_next():
        n, d, _ = r.read_next()
        if n == "/robot/wheel_status":
            m = deserialize_message(d, JointState)
            if idx is None:
                idx = [next(i for i, nm in enumerate(m.name) if k in nm) for k in WHEEL_KEYS]
            wt.append(_stamp(m.header)); wp.append([m.position[i] for i in idx]); wv.append([m.velocity[i] for i in idx])
        elif n == imu_topic:
            m = deserialize_message(d, Imu); it.append(_stamp(m.header))
            ig.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
            ia.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
        elif n == "/sim/ground_truth_odom":
            m = deserialize_message(d, Odometry); ot.append(_stamp(m.header))
            tw = m.twist.twist; ovx.append(tw.linear.x); ovy.append(tw.linear.y); owz.append(tw.angular.z)
    return (np.array(wt), np.array(wp), np.array(wv), np.array(it), np.array(ig), np.array(ia),
            np.array(ot), np.array(ovx), np.array(ovy), np.array(owz))

def _detect_imu_topic(bag, storage):
    info = rosbag2_py.Info()
    md = info.read_metadata(str(bag), storage)
    names = {t.topic_metadata.name for t in md.topics_with_message_count}
    for cand in IMU_TOPICS:
        if cand in names:
            return cand
    return None

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("episode_dir"); ap.add_argument("out")
    ap.add_argument("--storage", default="mcap"); ap.add_argument("--no-gt", action="store_true")
    ap.add_argument("--imu-topic", default=None, help="覆盖 IMU 话题;默认自动探测 /sim/imu→/rslidar_imu_data")
    ap.add_argument("--imu-sign", type=float, default=1.0, help="yaw_rate 全局符号(+1 sim,real 常为 -1,见 verify)")
    ap.add_argument("--static-sec", type=float, default=2.0, help="启动静止窗时长(s),用于去陀螺零偏(迭代e)")
    ap.add_argument("--speed-eps", type=float, default=0.05, help="判静止的最大 mean|轮速|(m/s)")
    a = ap.parse_args()
    ep = Path(a.episode_dir).expanduser()
    bag = ep/"rosbag2" if (ep/"rosbag2").exists() else ep
    imu_topic = a.imu_topic or _detect_imu_topic(bag, a.storage)
    wt, wp, wv, it, ig, ia, ot, ovx, ovy, owz = read(bag, a.storage, imu_topic)
    assert len(wt) > 50, "wheel_status 太少"
    d = dict(t_wheel=wt, steer=wp, speed=wv)
    if not a.no_gt and len(ot) > 1:
        d["gt_vx"] = np.interp(wt, ot, ovx); d["gt_vy"] = np.interp(wt, ot, ovy); d["gt_wz"] = np.interp(wt, ot, owz)
    if len(it) > 1:
        d["imu_g"] = np.stack([np.interp(wt, it, ig[:, k]) for k in range(3)], 1)
        d["imu_a"] = np.stack([np.interp(wt, it, ia[:, k]) for k in range(3)], 1)
        # phase-2 yaw-rate 通道:在 IMU 原始时戳上做重力投影,再对齐到 t_wheel。
        yr = project_yawrate(ia, ig, sign=a.imu_sign)        # (N_imu,)
        yr_wheel = np.interp(wt, it, yr)[:, None].astype(np.float64)  # (N_wheel,1)
        # 迭代e:扣陀螺零偏(静止窗内通道均值)——经典 wheel_odometry 一直在做,phase-2 漏了,
        # 导致真机 ~0.019 rad/s 偏置积分成跑飞航向漂移。等价于扣 up·b_g(见 imu_yawrate.debias_yawrate)。
        yr_db, bias, found = debias_yawrate(yr_wheel, wt, wv, a.static_sec, a.speed_eps)
        if not found:
            print(f"WARN 未找到 mean|轮速|<{a.speed_eps} 的静止窗,回退用前 {a.static_sec}s 估零偏 "
                  f"(可能高估零偏);bias={bias:+.5f} rad/s")
        d["imu_yawrate"] = yr_db
        d["imu_yawrate_bias"] = np.float64(bias)  # 已扣除的零偏,供溯源
    pf = ep/"episode_params.yaml"
    if pf.exists():
        d["kappa_theory"] = float(yaml.safe_load(pf.read_text()).get("kappa_theory", np.nan))
    np.savez_compressed(a.out, **d)
    bias_str = f"{float(d['imu_yawrate_bias']):+.5f}" if "imu_yawrate_bias" in d else "n/a"
    print(f"EXTRACT_OK {a.out} wheel={len(wt)} gt={'y' if 'gt_vx' in d else 'n'} "
          f"imu={'y' if 'imu_g' in d else 'n'} yawrate={'y' if 'imu_yawrate' in d else 'n'} "
          f"yawrate_bias={bias_str} topic={imu_topic} sign={a.imu_sign:+.0f}")

if __name__ == "__main__":
    main()
