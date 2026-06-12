<!--
 * @Author: meitiever
 * @Date: 2026-05-13 16:00:44
 * @LastEditors: meitiever
 * @LastEditTime: 2026-05-14 16:44:28
 * @Description: content
-->
# Wheel Odometry Plan

## 一句话总结

**核心层(零 IMU 依赖):4 轮的 (steering angle, drive speed) 连立 8×3 加权最小二乘,直接解出完整 body twist (vx, vy, ω_z),平面假设下积分即得完整 (x, y, θ) 轨迹。IMU 是可选增强层(`use_imu` 开关):在场时用 gyro 接管 yaw、Mahony 拉 roll/pitch tilt、提供 gyro-ZUPT 与 bias 估计;缺失时优雅降级回核心层。无 Kalman 滤波,9 维 state(p / R / b_g)。**

> **为什么是分层而不是"必须配 IMU"**:swerve 8×3 LS 的解 `z=(vx,vy,ω_z)` 里 `ω_z` 就是纯轮速解出的 yaw rate —— 它是阿克曼 `ω=v·tanδ/L` 在「4 轮独立转向」下的最小二乘推广(见 §与阿克曼模型的关系)。所以**纯轮速本就能出完整平面轨迹**,IMU 不是出轨迹的必要条件,而是对抗几何标定漂、补 tilt、加 ZUPT 的鲁棒性增强。`use_imu=false` 让本节点退化成一个自洽的平面轮式里程计;`use_imu=true` 拿到面向 GLIM 的全部优点。

跟 `leg_odometry/doc/fk_only_odometry.md` 是姊妹方案,把腿 FK 模块换成 4 轮 swerve LS,信号流和"无 Kalman"哲学一致。性能基线待跑通后补。

---

## 角色 / 消费者

本节点的设计**消费者是 GLIM**(LIO 因子图)。wheel 的独家信息是 **body frame 下的水平速度 (vx, vy)** —— LIO 长期会漂的那个维度,wheel 直接观测。其他维度:

- **ω_z**:GLIM 有 200Hz IMU gyro,wheel 构造约束(引入几何标定误差)，但长时间比 imu gyro 积分稳定。
- **vz**:**不是 wheel 的观测**(wheel 一无所知)→ 不出 vz 约束。"FlatZ" 若需要,是独立的地面 prior 模块,跟 wheel 解耦。
- **p, R**:由速度积分与角度观测获得。

**例外:STILL 状态**。车真静止时 wheel 知道全部 6 维都是 0,是 ZUPT (Zero-velocity UPdaTe) 的天然来源,LIO 静止段 IMU bias 易飘的高价值约束。

输出契约:**twist 一等公民,pose 仅诊断**。因子图构造由配对的 `wheel_constraint` GLIM ext 负责(跟 `leg_constraint` 同构)。

**`use_imu` 开关下的两种姿态**:
- `use_imu=true`(默认,接 GLIM 时):上面这套分工成立 —— ω_z 默认走 gyro,vx/vy 走 wheel,roll/pitch 走 IMU tilt,STILL 有 gyro 交叉校验。
- `use_imu=false`(独立/无 IMU 时):本节点是一个**自洽平面轮式里程计** —— ω_z 走 LS,roll/pitch 钉 0(平面假设),STILL 仅靠轮速判定,诊断 pose 是纯轮速积分的完整 (x,y,θ)。此时仍可发 twist 给 GLIM(只是 ω_z 来自 LS,cov 反映几何敏感性),也可开 `publish_tf` 当传统里程计用。

---

## 核心思路

### 整体信号流

一句话:**两条独立管道,只在 pose 处汇合**。左边 4 轮的 (转向角, 轮速) 进来,纯几何反解出"车身整体在 body frame 下的瞬时速度 (vx, vy, ω_z)";右边 IMU 进来,负责姿态 R(roll / pitch / yaw)和静止段的 bias 估计。两条线在 pose integrator 那里乘起来出诊断 pose。下游 GLIM 因子图只吃 twist + 状态码,不吃 pose。

```
──────  输入  ──────        ───────────  算法核心  ──────────         ──  输出  ──

/robot/wheel_status ─┐
(sensor_msgs/        │  ┌────── input_normalizer ──────┐
 JointState,         ├─▶│  units / sign / offset 归一  │─▶ (θ_i, v_i) i=FL,FR,RL,RR
 4 转向角+4 轮速)     │  └──────────────────────────────┘                    │
                     │                                                     ▼
                     │  ┌────────────── swerve_kinematics ──────────┐
                     │  │  组装 8×3 A 矩阵(只依赖 L,W) + 8×1 b        │
                     │  │  解 z = (vx, vy, ω_z) ← QR(A, b)          │─▶ body twist
                     │  │  残差 r = Az − b → MAD outlier 检测        │   (vx,vy,ω_z_LS)
                     │  │  state ∈ {L0,L1,L2,L3,STILL}              │   + cov_3×3
                     │  │  cov_3×3 = σ²·(AᵀA)⁻¹  (OLS 解析式)        │   + state
                     │  └───────────────────────────────────────────┘
                     │
/rslidar_imu_data ───┤  ┌─ imu_mount_calibrator (启动 3s 静止段, 一次性) ─┐
(sensor_msgs/Imu,    ├─▶│  avg_accel → R_base_imu (拉 gravity 对齐 +z)  │
 gyro + accel)       │  │  mean(gyro_static) → b_g (gyro bias)         │
                     │  └──────────────────────────────────────────────┘
                     │  ┌──────────── attitude_integrator ──────────────┐
                     └─▶│  R ← R · exp_so3((ω_used − b_g) · dt)         │─▶ R (3×3 姿态)
                        │  ω_used = ω_imu  (yaw_source=gyro, 默认)       │
                        │        或 (ω_imu.xy, ω_z_LS)  (yaw_source=ls) │
                        │  准静态时 Mahony 用 accel 拉 roll/pitch         │
                        │  STILL > 1s 时 EMA 慢更新 b_g (τ=30s)          │
                        └───────────────────────────────────────────────┘
                                                │ R
                                                ▼
                              ┌─ pose integrator (诊断, 可关) ─┐
                              │  v_world = R · (vx, vy, 0)_body │
                              │  p ← p + v_world · dt + FlatZ   │─▶ p (诊断)
                              └─────────────────────────────────┘

                          ┌──── /wheel_odometry/twist     (主, 给 wheel_constraint ext)
   三个 topic 输出 ────────┼──── /wheel_odometry/quality   (state, is_still, residual)
                          └──── /wheel_odometry/diag/odom (诊断 pose, 给 rviz)
```

> **层归属(`use_imu` 开关控制)**:上图右侧虚线框 `imu_mount_calibrator` 与 `attitude_integrator` 是 **IMU 增强层**,只有 `use_imu=true` 才实例化并订阅 `/rslidar_imu_data`。`input_normalizer` + `swerve_kinematics`(左侧)是 **核心层**,任何配置都跑。
>
> - `use_imu=false`:不订阅 IMU;`R` 退化为 `R(θ)`,θ 由 LS 的 `ω_z` 积分而来(平面假设 roll=pitch=0);`yaw_source` 强制 `ls`;STILL 仅看轮速;诊断 pose 仍可积分出完整 (x,y,θ)。
> - `use_imu=true`:IMU 层接管 `R` 的全 3D 姿态;`yaw_source` 默认 `gyro`;STILL 加 `‖gyro−b_g‖` 交叉校验。
> - **运行期 IMU 掉线**(stale > `imu_stale_max_ms`)而 `use_imu=true`:本帧优雅降级回核心层行为(平面 + LS yaw),并在 quality msg 标 `yaw_source_actual=LS`。

> **ω_z_LS 的去向**(一个容易忽略的点):LS 算法**必然**会解出 ω_z_LS(它是 3 维解 `z=(vx,vy,ω_z)` 的一个分量,不解就 LS 不完整、残差 r 也算不对)。但**用不用** ω_z_LS 由 `yaw_source` 一个开关同步控制三处:
>
> - **必用(LS 内部)**:`r = Az − b` 依赖 z 全 3 分量,MAD outlier 检测和 L0/L1/L2/L3 状态判定都靠 r。这条路径**任何配置都走**,ω_z_LS 是几何 LS 的内部产物,不可避免。
> - **选用(yaw_source=ls 才走)**:替换 ω_imu.z,既进 `attitude_integrator`(R 积分用),也进 `/wheel_odometry/twist.angular.z`(输出)。两处必须同步,保证 R 跟 ω_z 一致。
> - **默认 yaw_source=gyro**:上面那条分支不走,attitude_integrator 用 ω_imu.z,输出 twist 也发 ω_imu.z;ω_z_LS 只活在 LS 模块内部驱动残差。这是有意的"几何标定不准时用 IMU 兜底"(详见 §"为什么 yaw 默认走 gyro")。

**关键设计选择**(每个维度让最该信的传感器拍板,不做 Kalman 融合):

下表的"由谁拍板"按默认 `use_imu=true`;`use_imu=false` 时 ω_z 与 roll/pitch 改由轮速/平面假设接管(见该列括注)。

| 维度 | 由谁拍板 | 理由 |
|-----|---------|-----|
| vx, vy (body 水平速度) | **wheel LS** | wheel 独家观测,LIO 长期会漂的就是这个(与 use_imu 无关) |
| ω_z (yaw rate) | **IMU gyro_z**(use_imu=true 默认)/ **LS**(use_imu=false) | LS 出的 ω_z 对几何标定误差敏感(见 §"为什么 yaw 默认走 gyro") |
| R 的 roll / pitch | **IMU gyro + Mahony tilt**(use_imu=true)/ **≡0 平面假设**(use_imu=false) | wheel 不观测 |
| v_z, ω_x, ω_y | 没人 | wheel + IMU 都不直接观测,cov 设 1e6 告诉 GLIM 别用 |
| p (位置) | v 积分 + FlatZ | 诊断用,GLIM 自己出 pose |
| b_g (gyro bias) | 启动静止段初始化 + STILL EMA(仅 use_imu=true) | 长 run 漂的源头,必须治 |

跟 `fk_only_odometry.md` 哲学一致 —— 没有 9-D EKF 协方差矩阵,只有几条单向流和一份说清楚"我对哪些维度负责"的 twist covariance。

### 0. 4 轮读数类型与示例：
轮速topic: /robot/wheel_status - sensor_msgs/JointState
sensor_msgs/msg/JointState 是 ROS2 自带的标准类型,完美匹配你的数据结构,而且 echo 时不需要任何自定义 workspace。
std_msgs/Header header
string[] name
float64[] position    # 用于 4 个转向角 (rad)
float64[] velocity    # 用于 4 个轮速 (m/s)
float64[] effort      # 留空或放扭矩

对应的示例：
header:
  stamp: {sec: 1778574055, nanosec: 358339136}
  frame_id: "base_link"
name:     ["front_left", "front_right", "rear_left", "rear_right"]
position: [1.50, -1.50, 1.50, -1.50]        # 转向角
velocity: [-0.04, -1.65, -1.68, -1.66]        # 轮速
effort:   []

- position 记录的是轮子当前的朝向，x 轴是车头方向, 此位置输出为0，逆时针为正，顺时针为负。所以取值范围为 -90度 - 90度。输出为弧度，所以应该是 -1.57079 - 1.57079 之间。
- velocity 输出为轮速(m/s),向前为正,向後为负。最大速度为 1.0 m/s。

### 0.5 输入契约(归一层)

消息 `WheelStatus` 不规定单位/符号,各厂家不一。本节点把这些做成**参数**而非算法假设,**新平台上线只调这层,不动核心**:

| 参数 | 候选 | 含义 |
|---|---|---|
| `wheel_angle_units` | `"deg"` / `"rad"` | 角度字段单位(w2 实测:**rad**) |
| `wheel_angle_sign` | +1 / -1 | +1 = 左转为正(CCW 绕 body +Z) |
| `wheel_angle_offset` | rad,默认 0 | 各轮机械零点对车头的偏移 |
| `wheel_speed_units` | `"rad_s"` / `"m_s"` | 速度字段单位(w2 实测:**m_s**) |
| `wheel_speed_sign` | +1 / -1 | +1 = 车头前进为正 |
| `wheel_radius` | 1.0(w2) | 仅当 `speed_units=rad_s` 时使用,做 rad/s→m/s;w2 已是 m/s 故 1.0 |
| `wheelbase` (L) | 0.435(w2) | 前后轮纵向间距 (m) |
| `track` (W) | 0.400(w2) | 左右轮横向间距 (m) |

入口归一(伪代码):

```
θ_i = wheel_angle_sign · (angle_field_i · (units==deg ? π/180 : 1)) + offset_i
v_i = wheel_speed_sign · (speed_field_i · (units==rad_s ? wheel_radius : 1))
```

之后才进 8×3 LS 解算。

> **w2 实测标定值(2026-06-12)**:`angle_units=rad`(§0 的 position 字段就是弧度,范围 ±1.5708)、`speed_units=m_s`(velocity 字段就是 m/s,max 1.0)、`wheel_radius=1.0`(无需换算)、`L=0.435`、`W=0.400`。这套值在三段 bag 上跑出 SE2 尺度 0.99 vs GLIM、corr 0.996,已验证;旧文档里的 `deg / rad_s / r=0.121 / L=0.6 / W=0.5` 是早期占位值,**已作废**。κ 另由 `calibrate_kappa.py` 每 session 标。

### 1. 4 轮 swerve 反解 body twist (8×3 LS)

#### 1.1 物理图景

车身是刚体,4 个轮子是刚体上 4 个点。给定刚体瞬时 twist `(vx, vy, ω_z)`(body frame, ω_z 绕 +Z 轴 CCW 为正),每个轮接触点的速度由**刚体运动学**唯一决定:

```
v_i_body = v_body + ω × r_i
        = ( vx − ω_z · y_i,
            vy + ω_z · x_i )
```

其中 `r_i = (x_i, y_i)` 是第 i 个轮在 body frame 下的位置(只取水平分量,z 不参与)。

反过来,我们**测得**了每个轮接触点速度的方向(steer angle θ_i)和大小(drive speed v_i,归一后单位 m/s,有符号):

```
v_i_body_meas = v_i · [ cos θ_i,  sin θ_i ]   (2-D body 向量)
```

刚体约束就是 **测得 ≈ 模型出的**。4 轮 → 4 个 2-D 向量等式 → 8 个标量等式,3 个未知 (vx, vy, ω_z)。过定 → 最小二乘解。

#### 1.2 4 个轮的 body 位置

`(x_i, y_i)` 从 wheelbase L 和 track W 直接出(车身坐标系按 ROS REP-103: **+x 车头, +y 左, +z 上**,原点在 4 轮几何中心):

```
                  x (+车头, forward)
                   ▲
                   │
         FL ───────┼─────── FR
          ╳        │        ╳
                   │
      ◀──── y ─────┤              FL = ( +L/2, +W/2 )
      (+左)         │              FR = ( +L/2, −W/2 )
                   │              RL = ( −L/2, +W/2 )
          ╳        │        ╳     RR = ( −L/2, −W/2 )
         RL ───────┼─────── RR
                   │
```

> **命名约定与 JointState 顺序一致**:`name=["front_left", "front_right", "rear_left", "rear_right"]`(见 §0),代码里数组下标 0/1/2/3 对应 FL/FR/RL/RR。新平台命名不同的话,在归一层做 reorder。

> 原点不在几何中心(例如安在 IMU 处)也行,只要 4 个 `(x_i, y_i)` 跟原点定义一致即可 —— LS 出的 (vx, vy, ω_z) 就是"原点这一点"的 twist。

#### 1.3 线性系统的显式形式

把上面 v_i_body = v_i_body_meas 拆成 x、y 两个标量等式,移项后:

```
[ 1   0   −y_i ]   [ vx ]   [ v_i · cos θ_i ]
[ 0   1    x_i ] · [ vy ] = [ v_i · sin θ_i ]
                   [ ω_z]
```

4 轮拼起来得 `A ∈ ℝ^{8×3}`、`b ∈ ℝ^{8×1}`:

```
A = ┌ 1  0  −y_FL ┐    b = ┌ v_FL · cos θ_FL ┐
    │ 0  1   x_FL │        │ v_FL · sin θ_FL │
    │ 1  0  −y_FR │        │ v_FR · cos θ_FR │
    │ 0  1   x_FR │        │ v_FR · sin θ_FR │
    │ 1  0  −y_RL │        │ v_RL · cos θ_RL │
    │ 0  1   x_RL │        │ v_RL · sin θ_RL │
    │ 1  0  −y_RR │        │ v_RR · cos θ_RR │
    └ 0  1   x_RR ┘        └ v_RR · sin θ_RR ┘
```

**关键观察**:
- `A` 只依赖几何 (L, W),**每帧都不变**(可在节点初始化时一次性算好,跑时只更新 b)
- `b` 是测量值,每帧重算
- 前两列 (1,0,…) / (0,1,…) 对应 vx、vy 的常数贡献;第三列 (−y_i, x_i) 是 ω_z 的"杠杆系数"
- 4 个轮一致直行时,第三列内积 vx 列、vy 列都是 0 → ω_z 列正交于另两列,LS 把它压到 0,符合直觉

#### 1.4 解法与数值

用 Eigen 的 `colPivHouseholderQr`:

```cpp
Eigen::Vector3d z = A.colPivHouseholderQr().solve(b);
// z[0]=vx, z[1]=vy, z[2]=ω_z (LS 解, 不一定是最终输出)
```

为什么不直接 `(AᵀA)⁻¹ Aᵀ b`(normal equation):
- normal equation 把 A 的条件数**平方**,几何畸形(L≈0 或 W≈0)时雪上加霜
- QR 不显式构造 `AᵀA`,数值更稳
- col-pivoting 能侦测 rank-deficient(理论上 A 是 rank 3 当且仅当 4 个轮不共线,实务里 W=0 / L=0 会塌成 rank 2)
- 实现是一行,出错率最低

**条件数监控**:`A.colPivHouseholderQr().rank() < 3` 应该报警(几何参数错或单轮失效退化),但一般不发生。

#### 1.5 几何 sanity check(2 个例子)

**Case A — 直行**(全部 θ_i = 0,v_i = v,期望 vx=v, vy=0, ω_z=0):

各 b 行 = (v, 0, v, 0, v, 0, v, 0)ᵀ。代入正规方程或直接看 LS:
- 偶数行 (0, 1, x_i) · z = 0 → vy + x_i ω_z = 0 ∀i → vy = ω_z = 0
- 奇数行 (1, 0, −y_i) · z = v → vx − y_i ω_z = v ∀i → vx = v ✓

**Case B — 原地纯旋转**(swerve 把 4 轮设置成切线方向,期望 vx=0, vy=0, ω_z=ω):

如果硬件支持每个轮独立 90° 转向,θ_i 指向"绕原点的切向",v_i = ω·‖r_i‖。代入:

```
v_i · cos θ_i = ω · (−y_i)    (切向方向 = (−y_i, x_i)/‖r_i‖,乘 ω·‖r_i‖)
v_i · sin θ_i = ω · ( x_i )
```

b 行 = (−ω·y_FL, ω·x_FL, …),完美对应 A 的 ω_z 列,LS 出 (0, 0, ω) ✓

> Case B 要求硬件支持"原地旋转"模式,不是所有平台都有;但 Case A 是任何 wheel 平台都能做的最便宜 sanity check,标定时强烈建议先跑这个。

#### 1.6 残差与协方差

解出 `z*` 后:

```
r = A z* − b                 (8×1 residual, 加权前的原始残差)
σ² = (rᵀ W r) / (n − p)       (加权 unbiased variance, n=有效行数, p=3)
Σ_z = σ² · (AᵀW A)⁻¹          (3×3 解的协方差;W=I 时退回 OLS 的 σ²·(AᵀA)⁻¹)
```

(`W` 是 §1.7 的 |v_i| 加权对角阵,默认开。`wheel_weight_mode=none` 时 `W=I`,以上退化为普通 OLS。)

- **r** 是 slip 指示:刚体假设破裂的程度。某一行 `|r_i|` 大 → 该轮测量跟 LS 出的 twist 不一致(打滑 / 传感器异常 / 几何标定错 / 多体扭动)。第 §"鲁棒分级" 用 MAD 阈值化处理。
- **Σ_z** 给下游因子图的 noise model。GTSAM `noiseModel::Gaussian::Covariance(Σ_z)` 直接吃完整 3×3(包含 vx-vy-ω_z 交叉项,**不要对角化**)。

**n−p 分母**:OLS unbiased variance 用 n−p,不是 n。p=3 因为我们有 3 个自由参数。8 行解 3 未知 → 5 个 dof 用来估方差。L1 状态下 n=6,dof=3;L2 下 n=4,dof=1(估方差信噪比已经很差,所以 L2 强退 gyro)。

**Σ_z 的几何意义**:
- **单轮低速**(其它轮在动):该轮 θ 几乎不约束。**加权开(默认)**时 `w_i∝|v_i|→0`,`AᵀWA` 在对应方向退化 → `(AᵀWA)⁻¹` 自动膨胀,cov 如实反映"这一维我不可信"。**加权关**(`mode=none`)时 `(AᵀA)⁻¹` 恒定不膨胀,cov 会**低估**这种角度奇异 —— 这正是 §1.7 把加权设为默认的原因。
- **4 轮全部低速 / 静止**:这不是靠 cov 表达的,而是走 §STILL 状态判定直接出 0 + floor cov + ZUPT 语义。STILL 是并行轨道,先于 L 级判定,不依赖 `Σ_z` 的数值。
- **几何参数错**(L, W 偏)→ A 错 → ω_z 列的"杠杆"错 → 系统性偏,但残差 r 看不出(其他轮也按错的杠杆走)。这就是 §"为什么 yaw 默认走 gyro" 那一节讲的"残差小不代表 ω_z 对",和 IMU 交叉验证才靠谱。加权也救不了这种**系统性**偏(它只压随机噪声,不识别错杠杆)。

#### 1.7 加权(`wheel_weight_mode=speed`,默认开)

**默认实现是按 |v_i| 加权的 WLS**(不是 unweighted OLS)。这不仅是精度优化,更是**让协方差诚实**的必需 —— 见下面的关键论证。

- **按 |v_i| 加权(默认)**:近静止轮的 θ 测量本身就不约束什么(`v_i·cos θ_i → 0`,该行的 `b` 趋 0 但 `A` 行不变),应当降权;高速轮 SNR 高,应升权。形式上把 8×3 LS 换成 `W^{1/2} A z = W^{1/2} b`,`W=diag(w_i)` 是 8×8 对角,`w_i ∝ |v_i|`(同一轮的 2 行共用一个权,带 floor 防 0)。解仍用 `colPivHouseholderQr` 解加权系统。
- **IRLS(`wheel_weight_mode=irls`,可选)**:跑一遍 WLS,按残差更新权重(Huber / Cauchy),再跑,直到收敛。比 MAD 硬剔除更平滑,但引入"为什么这个轮被降权"的诊断负担,默认不开。
- **`wheel_weight_mode=none`**:退回 unweighted OLS,仅用于和加权做对照实验。

> **为什么加权是 cov 正确性的前提(而非可选优化)**:unweighted OLS 下 `A` 只依赖几何,**每帧恒定**,所以 `(AᵀA)⁻¹` 是**常数矩阵**,不随速度变化。于是低速时 `Σ_z=σ²·(AᵀA)⁻¹` 只靠 `σ²=‖r‖²/(n−p)` 缩放,而低速残差小 → `σ²` 小 → **单轮低速的角度奇异被 cov 低估**(详见 §1.6)。
>
> 加权后系统变成 `Σ_z=σ²·(AᵀWA)⁻¹`,`W` 随 `|v_i|` 变 → `AᵀWA` **每帧都变** → 低速轮自动降权,对应方向的方差**真的膨胀**。这才让"speed≈0 时该轮几乎不约束"这件物理事实被协方差如实表达。所以本设计把加权设为默认,而不是留作可选。

#### 1.8 与阿克曼(单轨)模型的关系

常见疑问:「阿克曼 `θ̇ = v·tanδ/L` 纯靠轮速就给出 yaw,为什么这套要扯上 IMU?」答案是 **swerve 8×3 LS 本就是阿克曼的推广,纯轮速一样出 yaw;IMU 只是可选的鲁棒性增强**。三点说清:

1. **同源**:阿克曼 `ω=v·tanδ/L` 和 LS 解里的 `ω_z` 是**同一套刚体运动学**的两个特例。阿克曼是「单转向角 δ + 单轨 + 后轴无侧滑 (vy=0)」的闭式解;swerve LS 是「4 个独立转向轮」的过定最小二乘解。把 swerve 退化成「前轮共 δ、后轮固定、强制 vy=0」就还原出阿克曼。所以 `use_imu=false` 时本节点 = 一个比阿克曼更一般的纯轮速里程计。

2. **同样的几何敏感性**:阿克曼并没躲过标定漂 —— `ω=v·tanδ/L` 里 L 错 5%,ω 就错 5%,积分照漂,和 §"为什么 yaw 默认走 gyro" 担心的是同一个病。任何积分轮速 yaw 的方案(阿克曼或 LS)都漂;没有绝对参考(磁力计/GPS/回环)就没有无漂 heading。IMU gyro 之所以默认接管 yaw,是因为它的漂只受 bias 稳定度限制,短时(~100s)通常比"几何标定不准的轮速 yaw"更稳 —— 这是工程权衡,不是数学必需。

3. **本平台是 4 轮独立转向,不能套阿克曼**:w2 实测 `position=[1.50,-1.50,1.50,-1.50]`(FL/FR/RL/RR),**后轮也转到 ±86°**,能蟹行、能原地转,vy 是真实自由度。阿克曼单轨模型强制 `vy≡0`,套上去会**抹掉 vy 的可观测性** —— 而 vy(body 横向速度)恰是本节点对 GLIM 的全部独家价值(LIO 长期会漂的那一维)。所以对本平台,阿克曼是降级而非升级,8×3 swerve LS 才是正确模型。

### 2. R 怎么来:gyro 积分 + 重力 Mahony 拉 tilt(IMU 增强层)

> **本节整段属于 IMU 增强层,仅 `use_imu=true` 走。** `use_imu=false` 时不订阅 IMU,`R` 退化为绕 +Z 的平面旋转 `R(θ)`,θ = ∫ω_z_LS dt(roll=pitch≡0),诊断 pose 由它积分。`R` **从不进因子图** —— twist 在 body 帧,GLIM 自己出姿态;`R` 只服务诊断 pose 与 STILL 的 bg EMA。

```
R ← R · exp_so3((gyro_used − bg) · dt)
其中 gyro_used = (gx_imu, gy_imu, gz_imu)        if yaw_source=gyro (use_imu=true 默认)
                (gx_imu, gy_imu, ω_z_LS)         if yaw_source=ls 且 ‖r‖ < slip_thr
```
- pitch/roll:gyro_x、gyro_y 积分,准静态时用 accel 跟重力对齐做 Mahony 修正(误差 cross product,zero out z 保留 yaw 自由度)
- yaw:LS 解出来"理论上"无漂,实测对几何标定误差极敏感,`use_imu=true` 时**默认走 gyro_z**(详见下一节)

### 3. IMU 装载自动校正(IMU 增强层,`use_imu=true` 时必须)

> `use_imu=false` 跳过本节(不订阅 IMU,无 mount 标定,无 bg)。以下仅在 `use_imu=true` 走。

`/rslidar_imu_data` 是雷达内置 IMU,装载方向通常**不竖直**(IMU 在雷达里、雷达在车顶可能有倾角)。具体倾角看本机启动静止段的 `avg_accel`。

节点在启动 `bias_window_sec` 静止段结束时,从 `avg_accel` 自动算
```
R_base_imu_ = FromTwoVectors(avg_accel.normalized(), [0,0,1])
```
后续每条 IMU:
```
gyro_base  = R_base_imu_ · (gyro_imu  − bg)
accel_base = R_base_imu_ · accel_imu
```
不做这步出来的轨迹是个 figure-8 鬼影。

注:`R_base_imu_` 的 yaw 分量未约束(重力对绕 z 的旋转不变)。FromTwoVectors 给最小角度旋转,等价于"yaw mount 默认 0"。实际中 ω_z 关于同一根 z 轴 frame-invariant,所以 OK。

---

## 为什么 yaw 默认走 gyro 而不是 LS

原始设计直觉:swerve 4 轮 LS 直接解 ω_z,无积分→无漂,应完胜 gyro。

理论分析:LS 出的 ω_z 是 (vx, vy, ω_z) 三元组里**对几何标定最敏感**的那个。`ω·r_i` 项的系数直接来自 wheelbase L、track W、wheel_radius r;这三个量错 5%,ω_z 系统性偏 5%,积分 100s 就是几十度量级。vx, vy 在 4 轮一致直行时几乎跟 L, W 无关(几何信息只对 ω_z 敏感),所以 vx, vy 用 LS 是安全的。

gyro_z(经 mount cal 后)的累计漂只受 gyro bias 稳定度限制,在常温短时(100s 量级)预期 ~几度量级,跟"几何标定不准的 LS ω_z"比远更鲁棒。

**实测验证(2026-06-12,w2 三段 bag,GLIM 为真值)**:LS ω_z 的瞬时值跟 IMU/GLIM yaw rate **几乎完美同步(corr 0.996)**,但**积分朝向漂 ~1.9°/s**(4 分钟漂 446°)。诊断很硬:发散 99.9% 线性(R²=0.999),且 LS 残差和 yaw 误差**几乎零相关(corr 0.05)→ 不是打滑,是标定偏置**。根因是**亚度级转向零点(~0.25°)+ 0.4% 左右轮速不对称**,在直行段(本该 ω_z=0)就产生恒定幻像 yaw。而 gyro(mount cal + 去偏后)对 GLIM 真值 **1.6° RMS / 4min**,远胜。

**结论**:理论"swerve 无 yaw drift"优势只有在**精确几何标定**时才拿得到,而实测标定误差小到无法手标却足以让 LS yaw 漂 1.9°/s。所以接 GLIM 时默认 `yaw_source=gyro`。**但这个偏置救得回来 —— 它是一个可标定的常数 κ,见下一节。**

---

## 轮速 yaw 的可救之道:κ 标定(`ω_z ← ω_z − κ·v_x`)

上一节的 1.9°/s 漂不是随机噪声,是一个**速度成正比的恒定曲率偏置**。实测把它压成一个标量参数:

```
ω_z_corrected = ω_z_LS − κ · v_x          (κ 单位 rad/m)
```

**为什么是 1 维**:幻像 yaw 的两个来源(前后轮 toe 零点 + 左右轮速尺度差)**都**产生 `ω ∝ v` 的项,合并成单一系数 κ。实测拟合 8 参数(4 转向零点 + 4 轮速尺度)对着 gyro,跨 bag 不比 1 参数 κ 好(都是 19.5° RMS)→ 证实误差本质 1 维,**8 参数是过拟合,κ 一个数就够**。

**κ 怎么估**:闭式最小二乘,对运动帧(`|v|>speed_min`)解
```
κ = Σ (ω_z_LS − yaw_ref) · v_x  /  Σ v_x²        (yaw_ref = mount校正去偏后的 gyro_z,或激光)
```
工具:`scripts/calibrate_kappa.py <bag>` —— 喂一段带 gyro 运动的 bag,吐出 κ + 验证报告(可 `--output` 写 yaml)。

**κ 的稳定性(实测,决定怎么用)**:

| | 06_05_a | 06_05_b(同天) | 05_22(隔两周) |
|---|---|---|---|
| κ (rad/m) | -0.0358 | -0.0347 | **-0.0444** |
| 度量尺度(≈wheel_radius) | 0.999 | — | 0.988 |

- **度量尺度跨天稳 ≈1.0** → 速度场本就是 m/s,`wheel_radius` 不用反复标。
- **κ 同天稳(3%),跨两周漂 24%** → **必须按 session 标 κ**,不能"标一次永久用"。落地:开机用一段带 gyro 的运动估 κ,或运动时在线持续对 gyro 估 κ;激光/IMU 掉线时用最后的 κ 兜几分钟。

**κ 修正后能到什么程度(实测 vs GLIM 真值,按 session 标定)**:

| bag | 路径 | 纯轮速 raw | **纯轮速+κ** | wheel+IMU yaw |
|---|---|---|---|---|
| 05_22 | 96m | APE 5.87m | **2.51m (2.6%)** | 0.30m |
| 06_05_a | 216m | APE 27.3m | **3.82m (1.8%)** | 4.10m |

heading 漂 1.9°/s → **0.07°/s(好 27×)**。06_05_a 上纯轮速+κ(3.82m)甚至略胜 wheel+IMU-yaw(4.10m)。

**但仍非零漂**:0.07°/s → 10min 漂 43°、1hr 漂 260°。所以 κ 修正让纯轮速胜任**几分钟级 / 百米回路的独立 dead-reckoning 备份**,但长时 standalone 仍需回环/绝对参考兜底。`use_imu=false` 模式因此**必须配 κ** 才有意义(裸 LS yaw 螺旋塌缩,见 §实测数据 轨迹图)。

---

## 鲁棒分级 (L0-L3 + STILL)

8×3 LS 是过定系统(8 行 3 未知,余度 2.67×)。这余度在**单轮失效**(传感器哑 / 单轮打滑 / 通信丢)时用 per-row residual 分级处置。

### 检测准则:MAD (Median Absolute Deviation)

**按轮(而非按行)做 MAD**:每个轮贡献 2 行(x、y),先把同一轮的 2 行残差合成 per-wheel 范数,再对 4 个范数做 MAD。这样语义干净(一个轮整体进/出),也避免"x 行正常 y 行异常"的歧义。

```
r = A z − b                          (8×1 残差)
ρ_j = ‖(r_{2j}, r_{2j+1})‖₂          (j=FL,FR,RL,RR 的 per-wheel 残差范数, 4 个)
med_ρ = median(ρ_j)
mad   = median(|ρ_j − med_ρ|)
outlier_j  iff  |ρ_j − med_ρ| > k · mad      (k=3, 默认)
```

> **MAD 在 4 个样本上很弱,别高估它**:MAD 击穿点 50%,但这里有效样本只有 4 个轮 —— 2 轮同时异常就到极限(中位数本身被带偏)。所以 **MAD 只负责挡"单轮"异常;≥2 轮异常的真正安全网是 `min_valid_wheels=2` 的 L3 硬截断**(见状态机),不是 MAD。文档别让人误以为 MAD 能扛多轮打滑。

### 状态机

| 状态 | 触发 | (vx, vy) | ω | cov 膨胀 | 输出 |
|---|---|---|---|---|---|
| **L0** | 4 轮 outlier-free | 8×3 LS | yaw_source 策略 | ×1 | 正常 |
| **L1** | 1 轮 outlier | 6×3 LS(丢该轮) | yaw_source 策略 | ×4 | warn |
| **L2** | 2 轮 outlier | 4×3 LS | **强制 gyro** | ×16 | warn |
| **L3** | ≥3 轮 outlier 或 valid < `min_valid_wheels`=2 | 不出 | gyro 积分 R 续 | ∞ | **不发 twist**(只发 quality) |
| **STILL** | `use_imu=true`: `‖raw_speed‖<ε_v` AND `‖gyro−b_g‖<ε_ω`<br>`use_imu=false`: 仅 `‖raw_speed‖<ε_v` | 0 | 0 | floor | 正常 + `is_still=true` |

**STILL 跟 L 级是并行轨道**:每帧先判 STILL,STILL 命中直接出 0 不走 L 级。

> **gyro 判据必须去偏**:用 `‖gyro−b_g‖` 而非裸 `‖gyro‖`。典型雷达内置 IMU 的零偏(0.005–0.02 rad/s)和阈值 `still_gyro_eps=0.02` 同数量级,不去偏会让静止的车 STILL 漏判,丢掉最有价值的 ZUPT 帧。`use_imu=false` 无 b_g,只能靠轮速判 STILL(代价:失去 gyro 交叉校验,极慢蠕动可能误判 STILL)。

L2 强退 gyro:4×3 算出来的 ω 信噪比已经很差,不可信。(`use_imu=false` 无 gyro 可退,L2 只能保留 LS ω 并把 inflation 拉到 ×16 + quality 告警,提醒下游这帧 ω 不可信。)

### STILL 状态的特殊价值

车真不动时 wheel 同时知道全部 6 维都是 0,GLIM 的 IMU bias 在静止段最容易飘(无观测信号),这一帧的 ZUPT 是高价值约束。`is_still` flag 在 quality msg 里单发,**让 GLIM ext 显式构造 6-DoF ZUPT factor**(语义比 VelocityFactor 更强)。

STILL 持续 > 1s 时,gyro 平均值是当前 bg 的高保真估计,本节点用慢 EMA(τ=30s)在线刷新 bg,治长 run gyro bias 漂。

### 状态无 hysteresis

每帧独立判定,不做切换抖动平滑。每帧本来就独立 LS,hysteresis 反而掩盖故障;下游 GLIM ext 看到状态切换时 cov 阶跃,正是设计意图(忠实反映 wheel 当前可信度)。

---

## Covariance 模型

twist covariance 是**唯一一等公民**(GLIM 不消费 pose,pose cov 写 0)。

### twist 6×6 covariance 矩阵

输出 `geometry_msgs/TwistWithCovarianceStamped.twist.covariance` 是 6×6,顺序 (vx, vy, vz, ωx, ωy, ωz):

| 块 | 来源 | 值 |
|---|---|---|
| (vx, vy) 2×2 | LS 解析 | `σ² · (AᵀA)⁻¹` 的对应 2×2 块 × inflation |
| (ωz) 1×1 | yaw_source 选择 | LS 模式:LS 解析;gyro 模式:gyro_z 噪声 spec(常数) |
| (vx,vy) ↔ ωz cross | yaw_source 选择 | LS 模式:LS 解析交叉项;gyro 模式:**0**(独立观测) |
| (vz, ωx, ωy) 对角 | 无观测惯例 | **1e6**(信息 ≈ 0,告诉 GLIM 别用) |
| 其他非对角 | | 0 |

**cross-correlation 保留**:GTSAM `noiseModel::Gaussian` 接 full 3×3,对角化等于丢信息。

### inflation 倍数

| 状态 | inflation |
|---|---|
| L0 | ×1 |
| L1 | ×4 |
| L2 | ×16 |

L3 不发 twist;STILL 用 floor 不走 inflation。

### Floor cov(STILL 状态)

STILL 时输出全 0,cov 不能为 0(否则下游认为绝对真值),用传感器底噪:

| 分量 | 值 | 来源 |
|---|---|---|
| σ²_vx, σ²_vy | (0.005 m/s)² = 2.5e-5 | 轮速量化底噪 |
| σ²_vz | (0.005 m/s)² | 静止时所有维度都钉 0 |
| σ²_ωx, σ²_ωy, σ²_ωz | (0.001 rad/s)² = 1e-6 | gyro bias 残差量级 |

注意 STILL 时其他维度(vz, ωx, ωy)用 floor 而**不是** 1e6 —— STILL 下 wheel 知道全部 6 维都是 0,这不是"无观测"。

### 与 yaw_source 的耦合规则(契约)

- `yaw_source=gyro`(use_imu=true 默认):ω cov ← gyro spec,(vx,vy) ↔ ωz cross **强制置 0**(gyro 与 wheel 独立观测)
- `yaw_source=ls`(use_imu=false 恒为此):ω cov ← LS 解析,cross 保留
- L2 强退 gyro:cov 处理同 yaw_source=gyro(`use_imu=false` 无 gyro,L2 只能升 inflation 并在 quality 告警,ω cov 仍走 LS 解析)

---

## 模块责任划分

```
[核心层 — 任何配置都实例化]
input_normalizer.{h,cpp}      纯算法: §0.5 归一层 (units / sign / offset / r)
                              (无 ROS 依赖, 单测)

swerve_kinematics.{h,cpp}     纯算法: 8×3 加权 LS(默认 |v_i| 权)+ MAD 鲁棒分级 + L0-L3
                              输出: (vx, vy, ωz, cov_3x3, state, dropped_mask)
                              (无 ROS 依赖, 单测)

[IMU 增强层 — 仅 use_imu=true 实例化]
imu_mount_calibrator.{h,cpp}  纯算法: 启动静止段 → bg + R_base_imu
                              (无 ROS 依赖, 单测)

attitude_integrator.{h,cpp}   纯算法: gyro 积分 R + Mahony tilt + STILL EMA bg 刷新
                              (无 ROS 依赖, 单测)

[ROS 胶水]
wheel_only_node.cpp           订阅/发布/参数/失效检测/拼装 quality msg
                              读 use_imu: false→只订 chassis,平面 R(θ),yaw=ls
                                          true →加订 imu,实例化 IMU 层,yaw=gyro
                              运行期 imu stale → 本帧降级核心层行为
                              发布 twist (主), diag/odom (诊断), quality
                              publish_tf=true 时才发 odom→base_link
```

算法和 ROS 解耦,4 个算法模块在 unit test 里给定输入数组直接断言输出,不依赖 launch/bag。

> 配对的 `wheel_constraint` GLIM ext 独立 package,见 `doc/wheel_constraint_design.md`。本文不展开因子图侧。

---

## 发布消息契约

| topic | type | 频率 | 用途 |
|---|---|---|---|
| `/wheel_odometry/twist` | `geometry_msgs/TwistWithCovarianceStamped` | 50 Hz | **主输出**,GLIM ext 消费 |
| `/wheel_odometry/quality` | `wheel_odometry/WheelOdometryQuality` | 50 Hz | 状态 / is_still / 残差 / 丢轮 mask |
| `/wheel_odometry/diag/odom` | `nav_msgs/Odometry` | 50 Hz | 诊断 pose(**任何模式都有**完整轨迹:`use_imu=false` 是平面轮速积分,`true` 是 3D),GLIM 不消费 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz | rqt_runtime_monitor 汇总 |
| `TF odom→base_link` | (tf2) | 50 Hz | **仅当 `publish_tf=true`**(默认 false),给"独立里程计"用法;接 GLIM 时务必保持 false |

> **时间戳契约(node↔ext)**:`twist.header.stamp = chassis.stamp`(轮速帧时刻)。`use_imu=true yaw_source=gyro` 时,`ω_z` 用的是最近一条 IMU 的 gyro,允许相对 chassis.stamp 滞后 ≤ `imu_stale_max_ms`(默认 20ms);超过则本帧降级走 LS yaw。GLIM ext 按 `chassis.stamp` 对齐/插值 twist 因子。

```
/wheel_odometry/twist        (主输出)
  header.stamp               同 chassis_state.stamp
  header.frame_id            "base_link"
  twist.twist.linear.x       vx (LS 解, STILL 时 0)
  twist.twist.linear.y       vy (LS 解, STILL 时 0)
  twist.twist.linear.z       0  (wheel 不观测)
  twist.twist.angular.{x,y}  0  (wheel 不观测)
  twist.twist.angular.z      ω_z (yaw_source 选择, STILL 时 0)
  twist.covariance           6×6,见 §Covariance 模型

/wheel_odometry/quality      (自定义 msg)
  header.stamp
  state                      uint8  {L0=0, L1=1, L2=2, L3=3, STILL=4}
  valid_wheels_mask          uint8  低 4 bit: FL/FR/RL/RR 是否有效
  dropped_count              uint8
  is_still                   bool
  residual_after_drop        float32   (m/s, ‖r‖₂ / sqrt(n))
  yaw_source_actual          uint8  {GYRO=0, LS=1}  (运行时实际值,可能跟参数不同)

/wheel_odometry/diag/odom    (诊断 pose, 不进因子图)
  header.frame_id            "odom"
  child_frame_id             "base_link"
  pose.pose                  集成出的诊断 pose
  pose.covariance            全 0
  twist                      同上 twist 输出
```

**默认不发 TF**(避免与 LIO 的 `odom → base_link` 冲突);ext 与 node 通过 topic 解耦。`publish_tf=true` 仅供独立里程计场景(无 LIO 时)发 `odom→base_link`。

---

## 实测数据

### 验证数据集(w2)

`/home/steve/Documents/Datasets/w2/` 下三段 bag,均含 `/robot/wheel_status`(JointState)+ `/rslidar_imu_data`(Imu)+ `/rslidar_points`(PointCloud2)。其中两段已有 GLIM LIO 跑出的 `traj_imu.txt`(TUM 格式)当真值。

| bag | 时长 | GLIM 真值 | 备注 |
|---|---|---|---|
| `rosbag2_2026_05_22-13_58_40` | 154s / 96m | ✓ | **隔两周**,测 κ 跨天稳定性 |
| `rosbag2_2026_06_05-17_37_48` | 246s / 216m | ✓ | κ 标定基准 bag |
| `rosbag2_2026_06_05-17_42_57` | 604s | ✗ | 10min,长时漂观察 |

> 之前 backup.md 里的旧 w2 数据(`rosbag2_2026_04_28-17_06_53`)已确认不可信,**不要参考**。

### 实测结论(2026-06-12)

**1. 轮速 vs IMU vs GLIM 三方朝向(06_05_a,240s)**:

| | 总朝向变化 | vs 真值误差 | RMS |
|---|---|---|---|
| GLIM(真值) | -359.5° | — | — |
| IMU gyro(mount校正+去偏) | -362.9° | -3.4° | **1.6°** |
| 轮速 LS(raw) | -805.7° | **-446°** | 272.8° |
| 轮速 LS **+κ** | -361.6° | +1.1° | **3.0°** |

→ 裸 LS yaw 不可用(1.9°/s 恒定偏置漂);**κ 修正后逼近 IMU**;IMU yaw ≈ GLIM 真值。

**2. 纯轮速完整轨迹(按 session 标 κ,SE2 对齐 GLIM)**:

| bag | 路径 | raw(无κ) | **纯轮速+κ** | wheel+IMU yaw |
|---|---|---|---|---|
| 05_22 | 96m | 5.87m | **2.51m (2.6%)** | 0.30m |
| 06_05_a | 216m | 27.3m | **3.82m (1.8%)** | 4.10m |

→ raw 螺旋塌缩;**+κ 后纯轮速画出闭合回路,APE = 路径的 1.8–2.6%**;06_05_a 上纯轮速+κ 甚至略胜 wheel+IMU-yaw。轨迹图见 `scripts/` 跑出的对比图(GLIM 黑 / 轮速+κ 蓝 / raw 红螺旋)。

**3. κ 跨天稳定性**:同天 -0.0358 vs -0.0347(3%),隔两周 -0.0444(24% 漂)→ **按 session 标 κ**(详见 §"轮速 yaw 的可救之道")。度量尺度跨天稳 ≈1.0,`wheel_radius` 不用反复标。

复现:`python3 scripts/calibrate_kappa.py <bag_dir>`(带 `traj_imu.txt` 的 bag 会附 GLIM 对比)。

---

## 标定流程 (新平台上线检查表)

1. **静态启动**(仅 `use_imu=true` 需要;`use_imu=false` 跳过本步,无 IMU 初始化):开机后 ≥3s 不动,看节点日志 `init done` 输出
   - `bg = [...]` 应 ≤ 0.01 rad/s 量级,否则 IMU 噪声大
   - `R_base_imu rpy = [...]` 显示装载方向,可跟机械图核对
2. **归一层调对**(`wheel_angle_units/sign/offset`、`wheel_speed_units/sign`)
   - echo `/robot/wheel_status` 看一段直行,符号 / 单位都对的话 vx 应跟着方向走
3. **wheel_radius 标定**:跑一段已知尺寸的直行(尺测 or LIO 真值),目标 `path_ratio` ≈ 1.000
   - 错 5% 直接说明 wheel_radius 错 5%,等比例调
4. **κ 标定(`use_imu=false` 必做,每 session 一次)**:跑一段带 gyro 的运动 bag,
   `python3 scripts/calibrate_kappa.py <bag> --output kappa.yaml`,把吐出的 `yaw_kappa` 填进 params
   - 报告里 `heading drift vs gyro` 应从 ~1.9°/s 降到 <0.1°/s;κ 跨天会漂 ~24%,故每 session 重标
5. **wheelbase / track 标定**(选):做一段已知半径的转圈,看 `yaw_source=ls` 时 `ω_z` 是否准
   - 准了再考虑切回 LS yaw,拿"无漂"优势
6. **slip_threshold 调**(选):地面易打滑时,`yaw_source=ls slip_thr=0.5` 提供回退保护

---

## 已知局限

1. **不处理 z 漂**:节点不出 vz,诊断 pose 的 z 靠 FlatZ 收敛(可选,不影响因子图)
2. **gyro yaw 长漂**:当前默认无外部 heading 闭环时,yaw 长时间(分钟级)会有可观累计漂(具体量级取决于 IMU bias 稳定度,跑通后测)。靠几何精确后切 LS,或下游 ext 加 `enable_omega_factor` 用 wheel ω_z 拉回
3. **假设开机前 ≥3s 静止**:估 gyro bias + 初始化 R_base_imu。不静止则 bg 估错,后续每秒进数千度 yaw 漂。STILL 在线 bg EMA 缓解长 run 漂,但救不了初始化
4. **wheel_to_imu translation 不用**:不影响 ω 测量;真要严格融合时由 ext 的 T_imu_base 补
5. **wheel_to_imu yaw 部分未约束**:R_base_imu 只对齐 pitch/roll(重力定义);IMU yaw 多/少装 30° 对 ω_z 无影响,但 gx/gy 数值会错。Mahony tilt 每帧拉回重力 → 小 yaw mount 错位被吸收
6. **角度奇异**:speed ≈ 0 时该轮 angle 几乎不约束 LS(`v_i·cos θ_i → 0`),解算靠其他轮的杠杆。**这要靠 §1.7 的 |v_i| 加权(默认开)才被 cov 如实表达** —— 加权后 `(AᵀWA)⁻¹` 在低速轮方向自动膨胀;若 `wheel_weight_mode=none`(unweighted),`A` 恒定、`(AᵀA)⁻¹` 不膨胀,cov 会低估这种奇异,不要在 none 模式下信任低速段的 cov
7. **wheel slip 单方向无法甄别**:MAD 检测能识别"一轮异常",但若 4 轮一起均匀打滑(沙地 / 冰面),整体残差不大,vx,vy 会系统性偏 → cov 不会膨胀。重打滑场景需要外部 slip 探测(IMU accel 对比)触发

---

## 使用

### Launch

```bash
# 默认:接 GLIM,IMU 增强层开,yaw 走 gyro(w2 实测几何)
ros2 launch wheel_odometry wheel_only_node.launch.py \
    use_imu:=true \
    wheelbase:=0.435 \
    track:=0.400 \
    wheel_radius:=1.0 \
    chassis_topic:=/robot/wheel_status \
    imu_topic:=/rslidar_imu_data

# 纯轮速独立里程计:不订阅 IMU,平面假设,发 TF。
# yaw_kappa 必填(先 calibrate_kappa.py 标出),否则 yaw 漂 ~1.9°/s
ros2 launch wheel_odometry wheel_only_node.launch.py \
    use_imu:=false \
    publish_tf:=true \
    yaw_kappa:=-0.0358 \
    wheelbase:=0.435 track:=0.400 wheel_radius:=1.0 \
    chassis_topic:=/robot/wheel_status
```

### 订阅 / 发布

- 订阅:
  - `/chassis_state` (`sensor_msgs/JointState`,默认 `/robot/wheel_status`)——**始终订阅**
  - `/imu` (`sensor_msgs/Imu`,默认 `/rslidar_imu_data`)——**仅 `use_imu=true` 订阅**
- 发布:
  - `/wheel_odometry/twist` (主, `geometry_msgs/TwistWithCovarianceStamped`, 50 Hz)
  - `/wheel_odometry/quality` (`wheel_odometry/WheelOdometryQuality`, 50 Hz)
  - `/wheel_odometry/diag/odom` (诊断, `nav_msgs/Odometry`, 50 Hz)
  - `/diagnostics` (`diagnostic_msgs/DiagnosticArray`, 1 Hz)
  - TF `odom→base_link` —— **仅 `publish_tf=true`**(默认 false,避免与 LIO 的 `odom → base_link` 冲突)

### Smoke test

```bash
bash src/other_odometry/wheel_odometry/scripts/run_smoke.sh
# env vars: BAG_DIR / WHEELBASE / TRACK / YAW_SOURCE / RATE
# 跑完打印 closure error(同 bag 起止帧诊断 pose 距离)
# 有 LIO 真值时再算 APE,无则只看 closure
```

---

## 关键参数 (按层组织)

**模式层**(分层开关,决定跑哪些模块)

| 参数 | 默认 | 作用 |
|---|---|---|
| **`use_imu`** | **true** | IMU 增强层总开关。`false`=纯轮速平面里程计(不订阅 IMU,yaw 走 LS,roll/pitch≡0);`true`=订阅 IMU,yaw 默认 gyro,加 tilt/ZUPT/bias |
| `imu_topic` | `/rslidar_imu_data` | `use_imu=true` 时订阅的 IMU topic |
| `wheel_weight_mode` | `"speed"` | LS 加权:`speed`=按 \|v_i\|(默认,cov 正确性前提)/ `none`=unweighted OLS(仅对照)/ `irls`=迭代重加权 |
| `publish_tf` | false | 是否发 `odom→base_link` TF。接 GLIM 必须 false;独立里程计可 true |
| **`yaw_kappa`** | **0.0** | 轮速 yaw 曲率偏置修正 `ω_z ← ω_z − κ·v_x`(rad/m)。`use_imu=false` 时**必填**(否则 yaw 漂 ~1.9°/s),由 `scripts/calibrate_kappa.py` 按 session 标定;`use_imu=true yaw_source=gyro` 时不用 |
| `kappa_online` | false | 运动时在线对 gyro 持续估 κ(需 `use_imu=true`);IMU/激光掉线时用最后的 κ 兜住 yaw |

**归一层**(新平台必调,与算法解耦)

| 参数 | 默认 | 作用 |
|---|---|---|
| `wheel_angle_units` | `"rad"` | 角度字段单位(w2 实测为 rad) |
| `wheel_angle_sign` | +1 | +1 = 左转为正 |
| `wheel_angle_offset` | 0 | 各轮机械零点偏移 (rad) |
| `wheel_speed_units` | `"m_s"` | 速度字段单位(**w2 实测速度字段就是 m/s**,非 rad/s) |
| `wheel_speed_sign` | +1 | +1 = 车头前进为正 |
| **`wheel_radius`** | **1.0** | rad/s → m/s 换算半径 (m);w2 速度已是 m/s 故 **1.0**(实测 SE2 尺度 0.99 vs GLIM,跨天稳) |
| `wheelbase` (L) | 0.435 | 前后轮纵向间距 (m,w2 实测值) |
| `track` (W) | 0.400 | 左右轮横向间距 (m,w2 实测值) |

**算法层**(调参慎重,与平台无关)

| 参数 | 默认 | 作用 |
|---|---|---|
| `yaw_source` | `"auto"` | yaw rate 源;`auto`=有 IMU 走 gyro、无 IMU 走 ls;`gyro`/`ls` 强制。`use_imu=false` 时恒为 ls |
| `slip_thr` | 0.5 | yaw_source=ls 时,LS 残差超阈值退 gyro (m/s);`use_imu=false` 无 gyro 可退,仅用于诊断告警 |
| `bias_window_sec` | 3.0 | 启动静止段(估 bg + R_base_imu);`use_imu=false` 不用 |
| `tilt_kp` | 1.0 | Mahony pitch/roll 修正增益 |
| `tilt_accel_band` | 0.5 | accel 准静态门控 (m/s²) |
| `still_bg_ema_tau` | 30.0 | STILL 在线 bg 刷新时常数 (s) |
| `flatz_enabled` | true | 诊断 pose 的 z 收敛(不影响 GLIM 输入) |
| `flatz_alpha` | 0.05 | p.z 收敛比例 |

**鲁棒层**(失效边界)

| 参数 | 默认 | 作用 |
|---|---|---|
| `mad_k` | 3.0 | MAD 阈值;`r_i > k · mad` 判 outlier |
| `min_valid_wheels` | 2 | 有效轮数低于此值进 L3(不发) |
| `still_speed_eps` | 0.02 | STILL 检测:轮速范数阈值 (m/s) |
| `still_gyro_eps` | 0.02 | STILL 检测:gyro 范数阈值 (rad/s) |
| `imu_stale_max_ms` | 20 | chassis 回调时 IMU 缓存最大 stale |

**输出层**(与下游契约)

| 参数 | 默认 | 作用 |
|---|---|---|
| `floor_sigma_v` | 0.005 | STILL 时线速 cov floor (m/s) |
| `floor_sigma_omega` | 0.001 | STILL 时角速 cov floor (rad/s) |
| `cov_no_observation` | 1e6 | (vz, ωx, ωy) 对角 cov(信息 ≈ 0) |
| `inflate_l1` | 4.0 | L1 cov 膨胀倍数 |
| `inflate_l2` | 16.0 | L2 cov 膨胀倍数 |

---

## 代码结构

```
src/other_odometry/wheel_odometry/
├── package.xml
├── CMakeLists.txt
├── include/wheel_odometry/
│   ├── so3_utils.h                       # skew, exp_so3
│   ├── input_normalizer.h                # §0.5 归一层
│   ├── swerve_kinematics.h               # 8×3 LS + MAD + L0-L3
│   ├── imu_mount_calibrator.h            # 静止段 → bg + R_base_imu
│   └── attitude_integrator.h             # gyro 积分 + Mahony tilt + STILL EMA
├── src/
│   ├── input_normalizer.cpp
│   ├── swerve_kinematics.cpp
│   ├── imu_mount_calibrator.cpp
│   ├── attitude_integrator.cpp
│   └── wheel_only_node.cpp               # ROS 胶水节点
├── msg/
│   └── WheelOdometryQuality.msg          # 自定义 quality msg
├── launch/
│   └── wheel_only_node.launch.py
├── config/
│   └── wheel_only_params.yaml
├── scripts/
│   ├── calibrate_kappa.py                # 从 bag 标定 yaw_kappa(对 gyro,可附 GLIM 对比)
│   └── run_smoke.sh                      # 端到端跑 bag,出 closure
├── test/                                  # 算法模块单测 (gtest)
│   ├── test_input_normalizer.cpp
│   ├── test_swerve_kinematics.cpp
│   └── test_imu_mount.cpp
└── doc/
    ├── wheel_odometry_plan.md            # 本文(node 设计)
    └── wheel_constraint_design.md        # 配套 ext 设计(独立文档)
```

依赖:Eigen3 + ROS2 标准包(`rclcpp` / `sensor_msgs` / `geometry_msgs` / `nav_msgs` / `diagnostic_msgs`)。

**wheel_odometry 不依赖 KDL / urdf / GTSAM / GLIM**。算法层与 ROS 解耦,可独立单测。

---

## 配套文档

- **本文** `wheel_odometry_plan.md` —— wheel_odometry node 自身的算法与发布契约
- **`wheel_constraint_design.md`** —— 配对的 GLIM ext 设计:订阅本节点 topic,构造 `PriorFactor<V>` (ZUPT) + `WheelVelocityFactor` (moving) + 可选 `BetweenFactor<Rot3>` (ω_z),写进 GLIM 因子图

两份文档在 node ↔ ext 接口处契约一致:
- topic 名:`/wheel_odometry/twist`、`/wheel_odometry/quality`
- frame 约定:node 输出 `base_link`,ext 用 cfg 给的 `T_imu_base` 变到 GLIM 的 IMU frame
- 状态码:`L0/L1/L2/L3/STILL` 五个枚举值,ext 按 §"状态机" 决定加哪个 factor
