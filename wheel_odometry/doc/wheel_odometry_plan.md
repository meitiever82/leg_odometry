<!--
 * @Author: meitiever
 * @Date: 2026-05-13 16:00:44
 * @LastEditors: meitiever
 * @LastEditTime: 2026-05-14 16:44:28
 * @Description: content
-->
# Wheel Odometry Plan

## 一句话总结

**4 轮的 (steering angle, drive speed) 连立 8×3 最小二乘解出 body twist (vx, vy);IMU 陀螺给 yaw rate;两者积分得位姿。无 Kalman 滤波,9 维 state(p / R / b_g)。**

跟 `leg_odometry/doc/fk_only_odometry.md` 是姊妹方案,把腿 FK 模块换成 4 轮 swerve LS,信号流和"无 Kalman"哲学一致。性能基线待跑通后补。

---

## 角色 / 消费者

本节点的设计**消费者是 GLIM**(LIO 因子图)。wheel 的独家信息是 **body frame 下的水平速度 (vx, vy)** —— LIO 长期会漂的那个维度,wheel 直接观测。其他维度:

- **ω_z**:GLIM 有 200Hz IMU gyro,wheel 构造约束(引入几何标定误差)，但长时间比 imu gyro 积分稳定。
- **vz**:**不是 wheel 的观测**(wheel 一无所知)→ 不出 vz 约束。"FlatZ" 若需要,是独立的地面 prior 模块,跟 wheel 解耦。
- **p, R**:由速度积分与角度观测获得。

**例外:STILL 状态**。车真静止时 wheel 知道全部 6 维都是 0,是 ZUPT (Zero-velocity UPdaTe) 的天然来源,LIO 静止段 IMU bias 易飘的高价值约束。

输出契约:**twist 一等公民,pose 仅诊断**。因子图构造由配对的 `wheel_constraint` GLIM ext 负责(跟 `leg_constraint` 同构)。

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

> **ω_z_LS 的去向**(一个容易忽略的点):LS 算法**必然**会解出 ω_z_LS(它是 3 维解 `z=(vx,vy,ω_z)` 的一个分量,不解就 LS 不完整、残差 r 也算不对)。但**用不用** ω_z_LS 由 `yaw_source` 一个开关同步控制三处:
>
> - **必用(LS 内部)**:`r = Az − b` 依赖 z 全 3 分量,MAD outlier 检测和 L0/L1/L2/L3 状态判定都靠 r。这条路径**任何配置都走**,ω_z_LS 是几何 LS 的内部产物,不可避免。
> - **选用(yaw_source=ls 才走)**:替换 ω_imu.z,既进 `attitude_integrator`(R 积分用),也进 `/wheel_odometry/twist.angular.z`(输出)。两处必须同步,保证 R 跟 ω_z 一致。
> - **默认 yaw_source=gyro**:上面那条分支不走,attitude_integrator 用 ω_imu.z,输出 twist 也发 ω_imu.z;ω_z_LS 只活在 LS 模块内部驱动残差。这是有意的"几何标定不准时用 IMU 兜底"(详见 §"为什么 yaw 默认走 gyro")。

**关键设计选择**(每个维度让最该信的传感器拍板,不做 Kalman 融合):

| 维度 | 由谁拍板 | 理由 |
|-----|---------|-----|
| vx, vy (body 水平速度) | **wheel LS** | wheel 独家观测,LIO 长期会漂的就是这个 |
| ω_z (yaw rate) | **IMU gyro_z** (默认) | LS 出的 ω_z 对几何标定误差敏感(见 §"为什么 yaw 默认走 gyro") |
| R 的 roll / pitch | **IMU gyro + Mahony tilt** | wheel 不观测 |
| v_z, ω_x, ω_y | 没人 | wheel + IMU 都不直接观测,cov 设 1e6 告诉 GLIM 别用 |
| p (位置) | v 积分 + FlatZ | 诊断用,GLIM 自己出 pose |
| b_g (gyro bias) | 启动静止段初始化 + STILL EMA | 长 run 漂的源头,必须治 |

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
| `wheel_angle_units` | `"deg"` / `"rad"` | 角度字段单位(w2:**deg**) |
| `wheel_angle_sign` | +1 / -1 | +1 = 左转为正(CCW 绕 body +Z) |
| `wheel_angle_offset` | rad,默认 0 | 各轮机械零点对车头的偏移 |
| `wheel_speed_units` | `"rad_s"` / `"m_s"` | 速度字段单位(w2:**rad_s**) |
| `wheel_speed_sign` | +1 / -1 | +1 = 车头前进为正 |
| `wheel_radius` | 0.121(w2) | 仅当 `speed_units=rad_s` 时使用,做 rad/s→m/s |
| `wheelbase` (L) | 0.6 | 前后轮纵向间距 (m) |
| `track` (W) | 0.5 | 左右轮横向间距 (m) |

入口归一(伪代码):

```
θ_i = wheel_angle_sign · (angle_field_i · (units==deg ? π/180 : 1)) + offset_i
v_i = wheel_speed_sign · (speed_field_i · (units==rad_s ? wheel_radius : 1))
```

之后才进 8×3 LS 解算。

> **w2 标定**:`wheel_radius`、`wheelbase`、`track` 三个几何量从 SolidWorks 机械图取(当前默认 r=0.121 / L=0.6 / W=0.5 是占位值,跑通后按 §标定流程 校准回填);`angle_units=deg`、`speed_units=rad_s` 可由 echo `/robot/wheel_status` 单位确认,不需要 ground truth。

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
r = A z* − b              (8×1 residual)
σ² = ‖r‖² / (n − p)       (unbiased OLS variance, n=有效行数, p=3)
Σ_z = σ² · (AᵀA)⁻¹        (3×3 解的协方差)
```

- **r** 是 slip 指示:刚体假设破裂的程度。某一行 `|r_i|` 大 → 该轮测量跟 LS 出的 twist 不一致(打滑 / 传感器异常 / 几何标定错 / 多体扭动)。第 §"鲁棒分级" 用 MAD 阈值化处理。
- **Σ_z** 给下游因子图的 noise model。GTSAM `noiseModel::Gaussian::Covariance(Σ_z)` 直接吃完整 3×3(包含 vx-vy-ω_z 交叉项,**不要对角化**)。

**n−p 分母**:OLS unbiased variance 用 n−p,不是 n。p=3 因为我们有 3 个自由参数。8 行解 3 未知 → 5 个 dof 用来估方差。L1 状态下 n=6,dof=3;L2 下 n=4,dof=1(估方差信噪比已经很差,所以 L2 强退 gyro)。

**Σ_z 的几何意义**:
- 4 轮全部 |v_i| 很小时 → b 量级小,但 r 也小;σ² 小但 (AᵀA)⁻¹ 的 ω_z 块没变,所以 ω_z 方差不会自动膨胀。**这是设计上故意的**:静止段的 ω_z 解就该是"低信号但低噪声"的 0,真正的"我不可信" 路径是 §STILL 状态判定,直接走 ZUPT。
- 几何参数错(L, W 偏)→ A 错 → ω_z 列的"杠杆"错 → 系统性偏,但残差 r 看不出(其他轮也按错的杠杆走)。这就是 §"为什么 yaw 默认走 gyro" 那一节讲的"残差小不代表 ω_z 对",和 IMU 交叉验证才靠谱。

#### 1.7 加权(默认关,留作可选)

当前实现是 **unweighted OLS**(每行等权)。两种可选加权方式:

- **按 |v_i| 加权**:近静止轮的 θ 测量本身就不约束什么(`0·cos θ = 0`),应该降权;高速轮 SNR 高,应升权。形式上把 8×3 LS 换成 `W^{1/2} A z = W^{1/2} b`,W 是 8×8 对角。
- **IRLS(iteratively reweighted)**:跑一遍 OLS,按残差更新权重(Huber / Cauchy),再跑一遍,直到收敛。比 MAD 离群剔除更平滑。

**先不实现**,等 unweighted 在实测里看到不够再加。原因:加权会引入 "为什么这个轮被降权了" 的诊断负担,先保持透明。

### 2. R 怎么来:gyro 积分 + 重力 Mahony 拉 tilt

```
R ← R · exp_so3((gyro_used − bg) · dt)
其中 gyro_used = (gx_imu, gy_imu, gz_imu)        if yaw_source=gyro (默认)
                (gx_imu, gy_imu, ω_z_LS)         if yaw_source=ls 且 ‖r‖ < slip_thr
```
- pitch/roll:gyro_x、gyro_y 积分,准静态时用 accel 跟重力对齐做 Mahony 修正(误差 cross product,zero out z 保留 yaw 自由度)
- yaw:LS 解出来"理论上"无漂,实测对几何标定误差极敏感,**默认走 gyro_z**(详见下一节)

### 3. IMU 装载自动校正(必须)

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

**结论**:理论"swerve 无 yaw drift"优势只有在**精确几何标定**时才能拿到。所以默认 `yaw_source=gyro`,LS yaw 留作"几何精确后再开"的可选项。

> 实测验证是 §实测数据 的事(待补)。本节先给设计选择的理由。

---

## 鲁棒分级 (L0-L3 + STILL)

8×3 LS 是过定系统(8 行 3 未知,余度 2.67×)。这余度在**单轮失效**(传感器哑 / 单轮打滑 / 通信丢)时用 per-row residual 分级处置。

### 检测准则:MAD (Median Absolute Deviation)

```
r = A z − b                      (8×1 残差)
med_r = median(r)
mad   = median(|r − med_r|)
outlier_i  iff  |r_i − med_r| > k · mad      (k=3, 默认)
```

每个轮贡献 2 行,outlier 按"轮"为单位丢(同一轮的 2 行同时进/出)。MAD 比固定阈值或速度比例阈值都鲁棒,是 robust statistics 的标准选择。

### 状态机

| 状态 | 触发 | (vx, vy) | ω | cov 膨胀 | 输出 |
|---|---|---|---|---|---|
| **L0** | 4 轮 outlier-free | 8×3 LS | yaw_source 策略 | ×1 | 正常 |
| **L1** | 1 轮 outlier | 6×3 LS(丢该轮) | yaw_source 策略 | ×4 | warn |
| **L2** | 2 轮 outlier | 4×3 LS | **强制 gyro** | ×16 | warn |
| **L3** | ≥3 轮 outlier 或 valid < `min_valid_wheels`=2 | 不出 | gyro 积分 R 续 | ∞ | **不发 twist**(只发 quality) |
| **STILL** | `‖raw_speed‖<ε_v` AND `‖gyro‖<ε_ω` | 0 | 0 | floor | 正常 + `is_still=true` |

**STILL 跟 L 级是并行轨道**:每帧先判 STILL,STILL 命中直接出 0 不走 L 级。

L2 强退 gyro:4×3 算出来的 ω 信噪比已经很差,不可信。

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

- `yaw_source=gyro`(默认):ω cov ← gyro spec,(vx,vy) ↔ ωz cross **强制置 0**
- `yaw_source=ls`:ω cov ← LS 解析,cross 保留
- L2 强退 gyro:cov 处理同 yaw_source=gyro

---

## 模块责任划分

```
input_normalizer.{h,cpp}      纯算法: §0.5 归一层 (units / sign / offset / r)
                              (无 ROS 依赖, 单测)

swerve_kinematics.{h,cpp}     纯算法: 8×3 LS + MAD 鲁棒分级 + L0-L3 状态判定
                              输出: (vx, vy, ωz, cov_3x3, state, dropped_mask)
                              (无 ROS 依赖, 单测)

imu_mount_calibrator.{h,cpp}  纯算法: 启动静止段 → bg + R_base_imu
                              (无 ROS 依赖, 单测)

attitude_integrator.{h,cpp}   纯算法: gyro 积分 R + Mahony tilt + STILL EMA bg 刷新
                              (无 ROS 依赖, 单测)

wheel_only_node.cpp           ROS 胶水: 订阅/发布/参数/失效检测/拼装 quality msg
                              调用上面 4 个算法模块
                              发布 twist (主), diag/odom (诊断), quality
                              不发 TF
```

算法和 ROS 解耦,4 个算法模块在 unit test 里给定输入数组直接断言输出,不依赖 launch/bag。

> 配对的 `wheel_constraint` GLIM ext 独立 package,见 `doc/wheel_constraint_design.md`。本文不展开因子图侧。

---

## 发布消息契约

| topic | type | 频率 | 用途 |
|---|---|---|---|
| `/wheel_odometry/twist` | `geometry_msgs/TwistWithCovarianceStamped` | 50 Hz | **主输出**,GLIM ext 消费 |
| `/wheel_odometry/quality` | `wheel_odometry/WheelOdometryQuality` | 50 Hz | 状态 / is_still / 残差 / 丢轮 mask |
| `/wheel_odometry/diag/odom` | `nav_msgs/Odometry` | 50 Hz | 诊断 pose,rviz/可视化用,GLIM 不消费 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz | rqt_runtime_monitor 汇总 |

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

**不发 TF**(避免与 LIO 的 `odom → base_link` 冲突);ext 与 node 通过 topic 解耦。

---

## 实测数据

**TODO**:节点跑通后,在 w2 平台用一段有 LIO 真值的 bag 跑全套:

- bag 选择 / 时长 / 场景描述
- LIO path vs wheel_odom path / path_ratio
- APE RMSE / APE max(诊断 pose 跟 LIO 对齐)
- SE2 alignment 残差(看是否有系统性偏移)
- 不同 `yaw_source` (gyro / ls) 对比

> 之前 backup.md 里的 w2 测量数据(`rosbag2_2026_04_28-17_06_53`)已确认不可信,**不要参考**。重新跑后再回填本节。

---

## 标定流程 (新平台上线检查表)

1. **静态启动**:开机后 ≥3s 不动,看节点日志 `init done` 输出
   - `bg = [...]` 应 ≤ 0.01 rad/s 量级,否则 IMU 噪声大
   - `R_base_imu rpy = [...]` 显示装载方向,可跟机械图核对
2. **归一层调对**(`wheel_angle_units/sign/offset`、`wheel_speed_units/sign`)
   - echo `/robot/wheel_status` 看一段直行,符号 / 单位都对的话 vx 应跟着方向走
3. **wheel_radius 标定**:跑一段已知尺寸的直行(尺测 or LIO 真值),目标 `path_ratio` ≈ 1.000
   - 错 5% 直接说明 wheel_radius 错 5%,等比例调
4. **wheelbase / track 标定**(选):做一段已知半径的转圈,看 `yaw_source=ls` 时 `ω_z` 是否准
   - 准了再考虑切回 LS yaw,拿"无漂"优势
5. **slip_threshold 调**(选):地面易打滑时,`yaw_source=ls slip_thr=0.5` 提供回退保护

---

## 已知局限

1. **不处理 z 漂**:节点不出 vz,诊断 pose 的 z 靠 FlatZ 收敛(可选,不影响因子图)
2. **gyro yaw 长漂**:当前默认无外部 heading 闭环时,yaw 长时间(分钟级)会有可观累计漂(具体量级取决于 IMU bias 稳定度,跑通后测)。靠几何精确后切 LS,或下游 ext 加 `enable_omega_factor` 用 wheel ω_z 拉回
3. **假设开机前 ≥3s 静止**:估 gyro bias + 初始化 R_base_imu。不静止则 bg 估错,后续每秒进数千度 yaw 漂。STILL 在线 bg EMA 缓解长 run 漂,但救不了初始化
4. **wheel_to_imu translation 不用**:不影响 ω 测量;真要严格融合时由 ext 的 T_imu_base 补
5. **wheel_to_imu yaw 部分未约束**:R_base_imu 只对齐 pitch/roll(重力定义);IMU yaw 多/少装 30° 对 ω_z 无影响,但 gx/gy 数值会错。Mahony tilt 每帧拉回重力 → 小 yaw mount 错位被吸收
6. **角度奇异**:speed ≈ 0 时该轮 angle 几乎不约束 LS(0×cos θ = 0×sin θ = 0),解算靠其他 3 轮的杠杆;`(AᵀA)⁻¹` 自动膨胀对应方差,cov 模型已经表达了
7. **wheel slip 单方向无法甄别**:MAD 检测能识别"一轮异常",但若 4 轮一起均匀打滑(沙地 / 冰面),整体残差不大,vx,vy 会系统性偏 → cov 不会膨胀。重打滑场景需要外部 slip 探测(IMU accel 对比)触发

---

## 使用

### Launch

```bash
ros2 launch wheel_odometry wheel_only_node.launch.py \
    wheelbase:=0.6 \
    track:=0.5 \
    wheel_radius:=0.121 \
    yaw_source:=gyro \
    chassis_topic:=/robot/wheel_status \
    imu_topic:=/rslidar_imu_data
```

### 订阅 / 发布

- 订阅:
  - `/imu` (`sensor_msgs/Imu`,默认 `/rslidar_imu_data`)
  - `/chassis_state` (`sensor_msgs/JointState`,默认 `/robot/wheel_status`)
- 发布:
  - `/wheel_odometry/twist` (主, `geometry_msgs/TwistWithCovarianceStamped`, 50 Hz)
  - `/wheel_odometry/quality` (`wheel_odometry/WheelOdometryQuality`, 50 Hz)
  - `/wheel_odometry/diag/odom` (诊断, `nav_msgs/Odometry`, 50 Hz)
  - `/diagnostics` (`diagnostic_msgs/DiagnosticArray`, 1 Hz)
- **不发 TF**(避免与 LIO 的 `odom → base_link` 冲突)

### Smoke test

```bash
bash src/other_odometry/wheel_odometry/scripts/run_smoke.sh
# env vars: BAG_DIR / WHEELBASE / TRACK / YAW_SOURCE / RATE
# 跑完打印 closure error(同 bag 起止帧诊断 pose 距离)
# 有 LIO 真值时再算 APE,无则只看 closure
```

---

## 关键参数 (按层组织)

**归一层**(新平台必调,与算法解耦)

| 参数 | 默认 | 作用 |
|---|---|---|
| `wheel_angle_units` | `"deg"` | 角度字段单位 |
| `wheel_angle_sign` | +1 | +1 = 左转为正 |
| `wheel_angle_offset` | 0 | 各轮机械零点偏移 (rad) |
| `wheel_speed_units` | `"rad_s"` | 速度字段单位 |
| `wheel_speed_sign` | +1 | +1 = 车头前进为正 |
| **`wheel_radius`** | **0.121** | rad/s → m/s 换算半径 (m) |
| `wheelbase` (L) | 0.6 | 前后轮纵向间距 (m) |
| `track` (W) | 0.5 | 左右轮横向间距 (m) |

**算法层**(调参慎重,与平台无关)

| 参数 | 默认 | 作用 |
|---|---|---|
| `yaw_source` | `"gyro"` | yaw rate 源;`"ls"` 需要 wheelbase/track 精确标定 |
| `slip_thr` | 0.5 | yaw_source=ls 时,LS 残差超阈值退 gyro (m/s) |
| `bias_window_sec` | 3.0 | 启动静止段(估 bg + R_base_imu) |
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
