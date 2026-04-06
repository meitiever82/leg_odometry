# CASBot02 Leg Odometry

ESKF + GTSAM Hybrid 腿式里程计，用于 CASBot02 人形机器人。

## 架构

```
/imu (200Hz) ──► ESKF 前端 (C++)          ──► /leg_odometry
/joint_states ──►  ├── IMU 递推                (Odometry, 200Hz)
                   ├── FK 位置观测
                   ├── ZUPT 速度约束（分轴噪声）
                   ├── 步级速度估计
                   ├── 静止检测（gyro 方差）
                   ├── 平面 Z 约束
                   └── 接触检测（effort + FK Z 辅助）
                        │
                   ┌────▼──────────┐
                   │ GTSAM Smoother│ ← 每 20 关键帧一次
                   │ (LM 滑窗优化) │ → gyro bias 校正 (alpha=0.05)
                   └───────────────┘
```

## 精度

### 仿真数据 (9 场景)

| 场景 | 路径 | XY 漂移 | Z 误差 |
|------|------|---------|--------|
| slope_up_down | 12m | **2.2%** | 32cm |
| turn_in_place | 1.7m | **1.8%** | 1cm |
| straight_medium (0.5m/s) | 15m | **1.8%** (Py) / **5.0%** (C++) | 4cm |
| s_curve | 26m | **7.3%** | 4cm |
| straight_fast (0.8m/s) | 20m | **13.9%** | 5cm |
| straight_slow (0.3m/s) | 9m | **17.4%** | 3cm |
| stop_and_go | 14m | 39.4% | 4cm |
| long_walk (5min) | 142m | 42.6% | 3cm |
| curve_walk (R=3m) | 16m | 51.2% | 4cm |

### 真实数据 (跑步机)

| 指标 | 值 |
|------|------|
| 时长 | 236 秒 |
| 路径 | 149 m |
| XY 漂移 | **6.4%** |
| Z 漂移 | < 1mm |
| 输出频率 | 175 Hz |

## 编译

```bash
cd hybrid_cpp && mkdir build && cd build
source /opt/ros/humble/setup.bash
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

依赖: GTSAM 4.3, Eigen3, KDL, kdl_parser, ROS2 Humble

## 使用

### ROS2 实时节点

```bash
export LD_LIBRARY_PATH=$HOME/Documents/local/gtsam-4.3/lib:$LD_LIBRARY_PATH

./build/leg_odom_node --ros-args \
  -p urdf_path:=<path/to/casbot02.urdf>
```

- 订阅: `/imu` (Imu), `/joint_states` (JointState)
- 发布: `/leg_odometry` (Odometry), TF `odom → base_link_leg_odom`

### 离线评估

```bash
# 生成仿真数据
python3 scripts/generate_sim_data.py --scenario all

# 导出 CSV
python3 scripts/bag_to_csv.py --scenario all

# C++ 评估
./build/leg_odom_hybrid data/sim/csv/straight_medium.csv output.csv
```

## 代码结构

```
├── hybrid_cpp/                    # ★ C++ 主代码
│   ├── state/
│   │   ├── State.h                # 21 维 ESKF 状态定义
│   │   ├── Propagator.h           # IMU 递推
│   │   ├── StateHelper.h          # Kalman 更新
│   │   └── Kinematics.h           # KDL FK + 接触检测
│   ├── update/
│   │   ├── UpdaterFK.h            # FK 位置观测
│   │   ├── UpdaterZUPT.h          # ZUPT + 步级速度 + 静止检测
│   │   └── UpdaterFlatZ.h         # 平面 Z 约束
│   ├── smoother/
│   │   ├── leg_factors.h          # GTSAM 自定义因子 (FK/FlatZ)
│   │   └── GTSAMSmoother.h        # GTSAM LM 滑窗优化
│   ├── leg_odom_hybrid.cpp        # 离线评估 (CSV 输入)
│   ├── leg_odom_node.cpp          # ROS2 实时节点
│   └── CMakeLists.txt
│
├── leg_odometry/                  # Python 参考实现 + 评估依赖
├── scripts/                       # 仿真生成 / 评估 / 数据转换
├── config/                        # 参数配置 (YAML)
├── doc/                           # 文档
└── launch/                        # ROS2 launch 文件
```

## 参考文献

- Bloesch et al. "State Estimation for Legged Robots", RSS 2013
- Leg-KILO: Kinematic-Inertial-Lidar Odometry, RA-L 2024
- Cerberus: Visual-Inertial-Leg Odometry, ICRA 2023
- LVI-Q: LiDAR-Visual-Inertial-Kinematic Odometry, RA-L 2025
