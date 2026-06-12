# wheel_odometry — 4-Wheel Independent Steering (Swerve) Odometry

> ⚠️ **本 README 是旧版设计,已被 `doc/wheel_odometry_plan.md` 取代,设计以后者为准。**
> 主要差异:plan.md 改为**分层架构(轮速核心 + IMU 可选,`use_imu` 开关)**、输入用标准 `sensor_msgs/JointState`(非自定义 `ChassisState`)、主输出为 `TwistWithCovarianceStamped`(给 GLIM,带完整 6×6 cov)、默认不发 TF、加 L0–L3+STILL 鲁棒分级与 |v_i| 加权 LS。本 README 描述的单体 `nav_msgs/Odometry`+TF、无 cov、`yaw_source=ls` 默认等均为旧实现。下方 w2 实测表(2026-04-29)仍可作历史参考,但新验证以 plan.md §实测数据 的 2026-06-05 两个 bag 为准。

FK-only 风格的轮式里程计,姊妹包 [`leg_odometry`](../leg_odometry/) 的车轮版。

> 设计依据见 `../leg_odometry/doc/fk_only_odometry.md` —— 同一套架构(IMU 给姿态、轮子 FK 给身体速度、无 Kalman),只把腿 FK 模块换成 4 轮 swerve 最小二乘解算。

## 一句话

**4 个轮子的 (steering angle, drive speed) 连立 8x3 最小二乘,直接解出 body twist (vx, vy, ω_z);IMU 给 R 的 pitch/roll(Mahony tilt),yaw 默认用 LS 解(无漂),打滑时回退 gyro。无 Kalman,9 维 state。**

## 与 `leg_odometry` (FK-only) 的差异

| 维度 | 腿 FK-only | 轮 swerve |
|---|---|---|
| Kinematics 模块 | `LegKinematics::foot_velocity_*` (KDL) | `swerve_kinematics::solve_body_twist` (8x3 LS) |
| 接触检测 | `effort_threshold` 切换单/双脚 | **不需要**(4 轮永远接触);LS 残差作 slip 指标 |
| ω_z 来源 | 只能 gyro(yaw 必漂) | **LS 可解出来**,gyro 只作 slip 回退 |
| Z | FlatZ + stance 门控 | FlatZ 始终启用(车体 z 极稳) |
| 失效模式 | 双脚 swing → v=0 | 打滑 → LS 残差大 → 用 gyro yaw + 接受 vx,vy 误差 |
| 标定参数 | `foot_roll_toe_offset` | `wheelbase`、`track`(车体几何) |

## State 9 维

| 变量 | 维度 | 来源 |
|------|------|------|
| p | 3 | v 积分 + FlatZ 钳 z |
| R | SO(3) | gyro_xy 积分 + Mahony tilt(pitch/roll);LS ω_z(默认)或 gyro_z(slip 回退)(yaw) |
| b_g | 3 | 启动前 `bias_window_sec` 静止段平均 |

## 数学

四轮位置 (body frame,FL/FR/RL/RR 顺序):
```
r_FL = ( L/2,  W/2)    r_FR = ( L/2, -W/2)
r_RL = (-L/2,  W/2)    r_RR = (-L/2, -W/2)
```

每个轮 i 给 (θ_i, v_i),刚体约束:
```
[1  0  -y_i]                [v_i cos θ_i]
[0  1   x_i]  · [vx,vy,ω]ᵀ = [v_i sin θ_i]
```

8 个方程 3 个未知 → `colPivHouseholderQr().solve(b)`。残差 `||A z - b||₂` 是 slip 指标。

## 输入消息 (`msg/ChassisState.msg`)

```
builtin_interfaces/Time stamp
float64 front_left_angle  / front_right_angle  / rear_left_angle  / rear_right_angle  # rad
float64 front_left_speed  / front_right_speed  / rear_left_speed  / rear_right_speed  # m/s
```

> 如果实际设备 publish 的 msg 类型/字段名不一样,写一个 ~20 行的 bridge node 把它转成 `wheel_odometry/ChassisState` 即可。

约定:
- `angle` 从 +x_body(车头)起算,绕 +z_body(向上)CCW 为正
- `speed` 是带符号标量,沿 wheel 当前指向方向滚动为正

## 编译

```bash
cd ~/rtabmap_ws
source /opt/ros/humble/setup.bash    # 或 jazzy
colcon build --packages-select wheel_odometry
source install/setup.bash
```

依赖:Eigen3、ROS2 (rclcpp / sensor_msgs / nav_msgs / geometry_msgs / tf2_ros / rosidl_default_generators)。**不需要 KDL / urdf / GTSAM**。

## 使用

```bash
ros2 launch wheel_odometry wheel_only_node.launch.py \
    wheelbase:=0.6 \
    track:=0.5 \
    chassis_topic:=/chassis_state \
    imu_topic:=/imu
```

订阅:
- `/imu` (`sensor_msgs/Imu`)
- `/chassis_state` (`wheel_odometry/ChassisState`)

发布:
- `/wheel_odometry` (`nav_msgs/Odometry`)
- TF: `odom → base_link_wheel_odom`

## 关键参数

| 参数 | 默认 | 作用 |
|---|---|---|
| `wheelbase` | 0.6 | L,前后轮接触点纵向间距 (m),**必须按平台标定** |
| `track` | 0.5 | W,左右轮横向间距 (m),**必须按平台标定** |
| `wheel_radius` | 0.121 | speed 字段乘子,实际是轮 rad/s 时填轮半径 m;若 speed 已是 m/s 填 1.0 |
| `bias_window_sec` | 3.0 | 启动前静止段长度,估 gyro bias + 初始 R |
| `tilt_kp` | 1.0 | Mahony pitch/roll 补偿增益 |
| `tilt_accel_band` | 0.5 | accel 准静态门控 \|\|a\|-9.81\| < band |
| `yaw_source` | `"ls"` | yaw rate 来源:`"ls"` 用 LS 解(默认,无漂),`"gyro"` 强制走陀螺 |
| `slip_threshold` | 0.5 | LS 残差 > 此值时该帧 yaw 回退 gyro (m/s) |
| `flatz_enabled` | true | p.z 向 0 收敛(平地假设);上下坡场景关掉 |
| `flatz_alpha` | 0.05 | 每帧 p.z ← (1-α)·p.z |

## w2 bag 实测(2026-04-29)

`Datasets/w2/rosbag2_2026_04_28-17_06_53` (186s, 9213 wheel_status, 室内/小场地慢速测试,真值 LIO `traj_imu.txt`):

**vs LIO 真值(174.6s overlap, LIO path 60.4m):**

| 配置 | APE RMSE | path_ratio | yaw_align | 备注 |
|---|---|---|---|---|
| r=1.000, gyro yaw | — | 8.28 | — | 把 speed 当 m/s,完全错 |
| r=0.130, gyro yaw | 4.90m | 1.077 | +24.6° | 半标定 |
| **r=0.121, gyro yaw** | **1.22m** | **1.002** | **+4.5°** | **★ 当前默认** |

**Closure-only 测量(无真值)对照(默认 r=1.0,从结果反推 LIO 真值前的旧测试):**

| 配置 | path | closure | 备注 |
|---|---|---|---|
| `yaw_source=ls` slip_thr=0.5 | 542m | 17.5% | LS 解 ω_z,残差超 0.5 退 gyro |
| `yaw_source=ls` slip_thr=100 | 542m | 30.0% | 永远信 LS yaw |
| `yaw_source=gyro` | 542m | 0.83% | 看起来很准,但 path 是 LIO 真值的 9× |

### 三个关键校正

1. **`wheel_radius=0.121`**:`ChassisState.speed` 字段实际是轮角速度 rad/s,**不是 m/s**(spec 文档误标)。乘以 wheel_radius 才是地速。w2 这台车标定到 0.121m。
2. **IMU auto-mount calibration**:`/rslidar_imu_data` 是雷达内置 IMU,装载 ~135° tilt。节点在 init 阶段从 avg_accel 自动算 `R_base_imu`,后续 gyro/accel 全转 base 帧。否则 R 全乱。
3. **`yaw_source=gyro`**:LS 解 ω_z 对 wheelbase/track 标定误差极敏感,几何不准时 gyro yaw(经 mount 校正后)远更可靠。几何精确后才考虑切回 LS。

## IMU mount 自动校正

w2 这台车的 IMU 是 `/rslidar_imu_data`(雷达内置),装载方向不竖直 —— 静止时 `avg_accel = [0.08, 6.89, -7.06]`,重力指向 ~135° 绕 X 轴 tilt。

节点在 `bias_window_sec` 静止段结束时,从 avg_accel 自动算 `R_base_imu`,后续 IMU 读数都先转到 base 帧再用。这样 gyro_z 才是 base 帧的 yaw 速率,Mahony tilt 才把 base 的 pitch/roll 拉回水平。日志输出 `R_base_imu rpy=[...]` 确认装载方向。

## 调参建议

1. **先做 smoke test**:`scripts/run_smoke.sh`(默认指向 w2 bag),输出 closure error
2. **几何标定**:用一段已知直行 / 已知半径转圈的 bag,反推真实 wheelbase/track
3. **几何精确后才切 `yaw_source=ls`**:才能拿到 swerve 的"无 yaw drift"理论优势
4. **打滑场景**(光滑地面、高加速),`yaw_source=ls slip_thr=0.5` 提供回退保护;否则 gyro 模式就够用

## 已知局限

1. **打滑时 vx,vy 仍然错**:LS 残差检测能拒绝 yaw 但拒绝不了 (vx, vy);打滑场景下里程计输出会过估
2. **不处理上下坡**:FlatZ 假设地面平,上下坡需 `flatz_enabled:=false`,代价是 z 方向有 IMU 漂(无 accel 积分,Z 几乎不动,漂得不严重)
3. **假设开机前 3s 静止**
4. **无 covariance**:`/wheel_odometry` 的 twist/pose covariance 留空,接 SLAM 融合时需要补
5. **角度奇异**:`speed` 接近 0 时 `angle` 几乎不约束(0×cos θ 还是 0),解算靠其他轮的杠杆。极慢时残差敏感度差,这是物理硬边界

## 代码结构

```
src/wheel_odometry/
├── package.xml
├── CMakeLists.txt
├── msg/
│   └── ChassisState.msg                          # 设备输入消息
├── include/wheel_odometry/
│   ├── so3_utils.h                               # skew, exp_so3
│   └── swerve_kinematics.h                       # 4 轮 LS 解算 + 几何
├── src/
│   └── wheel_only_node.cpp                       # 主节点 ~280 行
├── launch/
│   └── wheel_only_node.launch.py
├── config/
│   └── wheel_only_params.yaml
└── README.md
```

## 后续可扩展(按需)

1. **Slip detector 输出**:把 `ls_residual` publish 成 `std_msgs/Float64`,下游 SLAM 决定是否信任
2. **Wheel-IMU 融合**:目前 IMU 只贡献 R,可加 ESKF 把 wheel 速度作 measurement,IMU 作 prediction —— 但要先确认 FK-only 不够用再上
3. **Covariance**:LS Jacobian + 单轮速度方差直接给 (vx, vy, ω) 协方差;接 SLAM 时补
