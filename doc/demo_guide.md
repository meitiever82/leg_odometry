# CASBot02 Leg Odometry 演示指南

## 概述

使用 rosbag 回放 + RViz 可视化，展示 CASBot02 腿式里程计的效果：
- RViz 中实时显示机器人 3D 模型随关节数据运动
- EKF 输出的里程计轨迹叠加显示

## 前置条件

- ROS2 Humble
- 已编译 `leg_odometry` 包
- 数据集: `~/Documents/Datasets/CASBOT/leg/rosbag2_2026_03_18-16_35_30/`

## 步骤

### 1. 编译

```bash
cd ~/casbot_ws
colcon build --packages-select leg_odometry --symlink-install
source install/setup.bash
```

### 2. 仅关节映射验证（RViz 看模型动起来）

**终端 1** — 启动 remapper + robot_state_publisher:

```bash
ros2 launch leg_odometry verify_mapping.launch.py
```

**终端 2** — 启动 RViz:

```bash
rviz2
```

RViz 配置:
- Add → RobotModel, topic 设为 `/robot_description`
- Fixed Frame 设为 `base_link`
- 可选: Add → TF 查看坐标系

**终端 3** — 回放 rosbag:

```bash
ros2 bag play ~/Documents/Datasets/CASBOT/leg/rosbag2_2026_03_18-16_35_30 --clock -r 0.5
```

> `-r 0.5` 表示 0.5 倍速回放，便于观察。

此时 RViz 中应该看到 CASBot02 模型的腿部关节随数据运动。

### 3. 完整腿式里程计演示（EKF 轨迹）

**终端 1** — 启动完整 launch（remapper + robot_state_publisher + EKF 节点）:

```bash
ros2 launch leg_odometry leg_odometry.launch.py
```

**终端 2** — 启动 RViz:

```bash
rviz2
```

RViz 配置:
- Add → RobotModel (`/robot_description`)
- Add → Path, topic 设为 `/leg_odometry/path`
- Add → Odometry, topic 设为 `/leg_odometry`
- Fixed Frame 设为 `odom`

**终端 3** — 回放:

```bash
ros2 bag play ~/Documents/Datasets/CASBOT/leg/rosbag2_2026_03_18-16_35_30 --clock -r 1.0
```

### 4. 数据说明

| 项目 | 值 |
|------|------|
| 数据集 | 跑步机行走, 236 秒 |
| 话题 | `/joint_states` (200Hz), `/imu` (200Hz) |
| 关节 | 12 个腿部关节 + 腰/头/手臂 |
| EKF 状态 | 21 维 (位置/速度/姿态/bias/双脚位置) |

## 关键文件

| 文件 | 说明 |
|------|------|
| `config/joint_mapping.yaml` | bag 关节名 → URDF 关节名映射 |
| `config/ekf_params.yaml` | EKF 噪声参数 |
| `launch/verify_mapping.launch.py` | 关节映射验证 launch |
| `launch/leg_odometry.launch.py` | 完整里程计 launch |
| `leg_odometry/ekf.py` | Bloesch EKF 核心实现 |
| `leg_odometry/kinematics.py` | URDF FK 运动学 |
| `leg_odometry/contact_detector.py` | 踝关节力矩接触检测 |

## 常见问题

**Q: RViz 中模型不动？**
- 检查 `/joint_states_remapped` 话题是否有数据: `ros2 topic echo /joint_states_remapped --once`
- 确认 `--clock` 参数已加，或在 RViz 中关闭 use_sim_time

**Q: 模型姿态异常？**
- 检查 `joint_mapping.yaml` 中的 `joint_sign` 和 `joint_offset`
