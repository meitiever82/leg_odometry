# CASBot02 腿式里程计技术方案

## 文档信息

| 项目 | 内容 |
|------|------|
| 项目名称 | CASBot02 Leg Odometry |
| 版本 | v3.0 |
| 日期 | 2026-04-05 |
| 工期 | 6 个工作日 |
| 依赖数据 | `/joint_states` (200Hz) + `/imu` (200Hz) |
| 代码位置 | `/home/steve/casbot_ws/src/leg_odometry/` (独立 ROS2 Python 包) |

---

## 1. 背景与目标

### 1.1 问题

CASBot02 是一款全身人形机器人，配备 LiDAR + IMU 进行 SLAM 建图（基于 GLIM 框架）。当前建图系统仅依赖 LiDAR 和 IMU，存在以下问题：

- **LiDAR 退化场景**（长走廊、空旷区域、粉尘/雨雾）下里程计容易漂移
- **剧烈运动**时点云畸变严重，配准质量下降
- **未利用**机器人自身丰富的本体感知信息（12 个腿部关节编码器）

### 1.2 目标

开发一个独立的腿式里程计模块，利用关节编码器 + IMU 估计机器人本体的位姿，作为 LiDAR SLAM 的补充输入。

**本期目标（6 天）：**
- 验证关节数据与 URDF 模型的映射关系（Isaac Sim 可视化）
- 复现 Bloesch et al. 的 EKF 腿式里程计算法
- 输出独立的位姿估计，与纯 IMU 积分对比验证

**后续目标：**
- 作为 GLIM 扩展模块接入因子图优化（通过 `on_smoother_update` 回调注入 `BetweenFactor`）
- 与 LiDAR 里程计互补融合，提升退化场景鲁棒性

---

## 2. 相关工作

### 2.1 经典方法：Bloesch EKF

**Bloesch M., Hutter M., et al. "State Estimation for Legged Robots - Consistent Fusion of Leg Kinematics and IMU", RSS 2013.**

- **方法**：EKF，状态包含机身位姿/速度/IMU偏置 + 双脚世界系位置
- **预测**：IMU 积分；脚位置按接触状态调制过程噪声（着地小噪声 / 腾空大噪声）
- **更新**：正运动学 FK(q) 计算足端位置，与状态中存储的脚位置比较
- **优点**：实现简单，经典可靠，已在 ANYmal/StarlETH 验证
- **局限**：需要接触检测；不支持预积分，难以直接接入因子图；假设脚不滑动

**选择理由**：实现复杂度适中，6 天内可完成，EKF 框架清晰，便于后续扩展。

### 2.2 进阶方法：LVI-Q 足端预积分

**Marsim K., Oh M., et al. "LVI-Q: Robust LiDAR-Visual-Inertial-Kinematic Odometry for Quadruped Robots", IEEE RA-L, 2025.**

- **方法**：交替使用 ESIKF（LIKO 模块）和滑窗优化（VIKO 模块），融合 LiDAR + 视觉 + IMU + 关节编码器
- **核心创新**：足端速度预积分（foot-preintegration）
  - 用关节速度通过雅可比计算足端速度：$\dot{\mathbf{v}}_{foot} = \mathbf{J}(\mathbf{q}) \dot{\mathbf{q}}$
  - 在关键帧间累积足端位姿变化（类似 IMU 预积分）
  - 生成预积分因子直接注入优化
- **不需要接触检测**：绕过了非滑移假设，每个优化步都使用运动学因子
- **性能**：ATE 优于 Fast-LIO2、STEP、Cerberus 等方法
- **与 GLIM 接入路径**：预积分因子可直接作为 GLIM 的 `BetweenFactor` 注入

**后续价值**：本期完成 Bloesch EKF 后，可参考 LVI-Q 的足端预积分方法升级。

### 2.3 开源参考：Cerberus 2.0

**Yang S., Zhang Z., et al. Columbia University. https://github.com/ShuoYangRobotics/Cerberus2.0**

三篇论文支撑：
- "Multi-IMU Proprioceptive Odometry for Legged Robots" (IROS 2023, **Best Paper Finalist**)
- "Cerberus: Low-Drift Visual-Inertial-Leg Odometry For Agile Locomotion" (ICRA 2023)
- "Online Kinematic Calibration for Legged Robots" (RAL + IROS 2022)

- **方法**：EKF 估计速度 → 因子图滑窗优化（结合视觉）
- **核心创新**：脚部安装额外 IMU（WT901），多 IMU 融合修正漂移；在线运动学标定
- **性能**：漂移 < 0.5%，长距离（3km+）验证
- **开源**：完整 C++ 实现 + 数据集
- **与我们的关系**：CASBot 没有脚部 IMU，多 IMU 部分不适用；但其**在线运动学标定**方法可参考解决 URDF 不准的问题

### 2.4 标定相关：A²I-Calib

**"Anti-noise Active Multi-IMU Spatial-Temporal Calibration for Legged Robots", IROS 2025. https://github.com/DavidGrayrat/A2I-Calib**

- **方法**：通过优化标定运动轨迹（条件数分析），最小化噪声对多 IMU 外参标定的影响
- **与我们的关系**：主要解决多 IMU 标定，CASBot 单 IMU 场景不直接适用；但标定思路（主动激励 + 条件数优化）可借鉴

### 2.5 方法对比总结

| | Bloesch EKF | LVI-Q | Cerberus 2.0 |
|--|------------|-------|-------------|
| **融合框架** | EKF | ESIKF + 滑窗交替 | EKF + 因子图 |
| **运动学利用** | FK 位置法 | 足端速度预积分 | 足端速度 |
| **接触检测** | 需要 | 不需要 | 需要 |
| **额外传感器** | 无 | 无 | 脚部 IMU |
| **LiDAR 融合** | 无 | 有（LIKO 模块） | 无 |
| **开源** | 论文复现 | 未开源 | C++ 开源 |
| **实现复杂度** | 低 | 高 | 中 |
| **本期可行性** | 6天可完成 | 需更多时间 | 需适配 |

---

## 3. 技术方案

### 3.1 算法：Bloesch EKF

### 3.2 状态定义

error-state EKF，状态向量维度 24：

$$\mathbf{x} = \begin{bmatrix} \mathbf{p} \\ \mathbf{v} \\ \boldsymbol{\theta} \\ \mathbf{b}_a \\ \mathbf{b}_g \\ \mathbf{p}_{foot,L} \\ \mathbf{p}_{foot,R} \end{bmatrix}$$

| 符号 | 维度 | 含义 |
|------|------|------|
| $\mathbf{p}$ | 3 | 机身在世界系中的位置 |
| $\mathbf{v}$ | 3 | 机身在世界系中的速度 |
| $\boldsymbol{\theta}$ | 3 | 姿态误差（error-state，对应 SO(3)） |
| $\mathbf{b}_a$ | 3 | 加速度计偏置 |
| $\mathbf{b}_g$ | 3 | 陀螺仪偏置 |
| $\mathbf{p}_{foot,L}$ | 3 | 左脚在世界系中的位置 |
| $\mathbf{p}_{foot,R}$ | 3 | 右脚在世界系中的位置 |

### 3.3 系统流程

```
                 ┌──────────────┐
                 │   IMU 数据    │ 200 Hz
                 │  (加速度/角速度) │
                 └──────┬───────┘
                        │
                        ▼
              ┌─────────────────┐
              │   EKF 预测步骤   │
              │  IMU 积分推算     │
              │  p, v, R 更新    │
              │  偏置随机游走     │
              │  脚位置: 噪声取决 │
              │  于接触状态       │
              └────────┬────────┘
                       │
                       ▼
  ┌──────────────┐    ┌─────────────────┐
  │  关节数据     │───▶│   EKF 更新步骤   │
  │ (位置/力矩)  │    │                  │
  └──────────────┘    │  1. FK(q) 算脚位置│
                      │  2. 接触检测       │
                      │  3. 卡尔曼更新     │
                      └────────┬─────────┘
                               │
                               ▼
                      ┌────────────────┐
                      │  输出: 位姿估计  │
                      │  nav_msgs/Odom  │
                      └────────────────┘
```

### 3.4 预测模型（IMU 驱动）

在每个 IMU 时间步 $\Delta t = 5\text{ms}$ 内：

$$\hat{\mathbf{p}}_{k+1} = \mathbf{p}_k + \mathbf{v}_k \Delta t + \frac{1}{2}(\mathbf{R}_k(\mathbf{a}_m - \mathbf{b}_a) + \mathbf{g})\Delta t^2$$

$$\hat{\mathbf{v}}_{k+1} = \mathbf{v}_k + (\mathbf{R}_k(\mathbf{a}_m - \mathbf{b}_a) + \mathbf{g})\Delta t$$

$$\hat{\mathbf{R}}_{k+1} = \mathbf{R}_k \cdot \text{Exp}((\boldsymbol{\omega}_m - \mathbf{b}_g)\Delta t)$$

$$\hat{\mathbf{b}}_a = \mathbf{b}_a, \quad \hat{\mathbf{b}}_g = \mathbf{b}_g \quad \text{(常值 + 随机游走噪声)}$$

$$\hat{\mathbf{p}}_{foot,i} = \mathbf{p}_{foot,i} \quad \text{(常值 + 接触状态相关噪声)}$$

**脚位置过程噪声：**

$$Q_{foot,i} = \begin{cases} \sigma_{contact}^2 \cdot \mathbf{I}_3 & \text{if foot } i \text{ in contact (小噪声)} \\ \sigma_{swing}^2 \cdot \mathbf{I}_3 & \text{if foot } i \text{ in swing (大噪声)} \end{cases}$$

典型参数：$\sigma_{contact} = 0.01 \text{ m/}\sqrt{\text{s}}$，$\sigma_{swing} = 1.0 \text{ m/}\sqrt{\text{s}}$

### 3.5 更新模型（运动学观测）

观测方程：足端在世界系中的位置应等于状态中存储的脚位置。

$$\mathbf{z}_i = \mathbf{p} + \mathbf{R} \cdot \text{FK}_i(\mathbf{q})$$

$$\text{残差: } \quad \mathbf{r}_i = \mathbf{z}_i - \mathbf{p}_{foot,i}$$

其中 $\text{FK}_i(\mathbf{q})$ 是第 $i$ 只脚在机身坐标系下的位置，由 URDF 运动学链和关节角度计算得到。

**核心思想：** 当脚着地时，$\mathbf{p}_{foot}$ 过程噪声很小，相当于假设脚不动。此时关节角变化导致的 FK 变化，会被 EKF 解释为机身位置 $\mathbf{p}$ 的变化——这就是腿式里程计的原理。

### 3.6 接触检测

采用踝关节力矩阈值法：

$$c_i = |\tau_{ankle\_pitch,i}| > \tau_{threshold}$$

参数选择依据：
- 数据显示支撑期踝关节 pitch 力矩 mean ≈ 9 Nm，std ≈ 15 Nm
- 初始阈值设定 $\tau_{threshold} = 5$ Nm，后续根据实际效果调整
- 加入滞后（hysteresis）防止频繁切换

### 3.7 观测雅可比

更新步骤需要观测方程对状态的雅可比矩阵 $\mathbf{H}$：

$$\mathbf{H}_i = \frac{\partial \mathbf{r}_i}{\partial \mathbf{x}} = \begin{bmatrix} \mathbf{I}_3 & \mathbf{0} & -\mathbf{R}[\text{FK}_i(\mathbf{q})]_\times & \mathbf{0} & \mathbf{0} & \cdots & -\mathbf{I}_3 & \cdots \end{bmatrix}$$

其中 $[\cdot]_\times$ 为反对称矩阵（叉积矩阵），$-\mathbf{I}_3$ 出现在对应脚的位置列。

---

## 4. 数据基础

### 4.1 可用数据

数据集路径：`/home/steve/Documents/Datasets/CASBOT/leg/rosbag2_2026_03_18-16_35_30`

| Topic | 消息类型 | 频率 | 数量 | 时长 |
|-------|----------|------|------|------|
| `/joint_states` | sensor_msgs/JointState | 200 Hz | 47306 | 236s |
| `/imu` | sensor_msgs/Imu | 200 Hz | 47316 | 236s |
| `/joint_control` | sensor_msgs/JointState | 200 Hz | 47326 | 236s |

### 4.2 关节数据结构

每帧包含 53 个关节。腿部关节与 URDF 的映射关系（**待 Day 1-2 验证**）：

| Bag 索引 | Bag 名 | URDF 关节名 | 含义 |
|---------|--------|------------|------|
| 0 | LJ0 | left_leg_pelvic_pitch_joint | 左髋俯仰 |
| 1 | LJ1 | left_leg_pelvic_roll_joint | 左髋横滚 |
| 2 | LJ2 | left_leg_pelvic_yaw_joint | 左髋偏航 |
| 3 | LJ3 | left_leg_knee_pitch_joint | 左膝俯仰 |
| 4 | LJUP | _(并联连杆电机，忽略)_ | 左踝上驱动 |
| 5 | LJDOWN | _(并联连杆电机，忽略)_ | 左踝下驱动 |
| 6 | LJPITCH | left_leg_ankle_pitch_joint | 左踝俯仰 |
| 7 | LJROLL | left_leg_ankle_roll_joint | 左踝横滚 |
| 8 | RJ6 | right_leg_pelvic_pitch_joint | 右髋俯仰 |
| 9 | RJ7 | right_leg_pelvic_roll_joint | 右髋横滚 |
| 10 | RJ8 | right_leg_pelvic_yaw_joint | 右髋偏航 |
| 11 | RJ9 | right_leg_knee_pitch_joint | 右膝俯仰 |
| 12 | RJUP | _(并联连杆电机，忽略)_ | 右踝上驱动 |
| 13 | RJDOWN | _(并联连杆电机，忽略)_ | 右踝下驱动 |
| 14 | RJPITCH | right_leg_ankle_pitch_joint | 右踝俯仰 |
| 15 | RJROLL | right_leg_ankle_roll_joint | 右踝横滚 |

**备注：**
- `LJUP`/`LJDOWN`/`RJUP`/`RJDOWN` 与 `LJPITCH` 高度相关（r=-0.95/+0.94），为踝关节并联连杆驱动电机，不在 URDF 中建模
- 上半身关节（索引 16-52）bag 名与 URDF 名一致（加 `_joint` 后缀即可）

### 4.3 数据特征分析

| 指标 | 数值 |
|------|------|
| 步态频率 | ~1.0 Hz（步周期约 1s） |
| 双支撑期比例 | 15.2% |
| 左单支撑比例 | 38.7% |
| 右单支撑比例 | 34.1% |
| 腾空期比例 | 12.0% |
| 膝关节活动范围 | ~0.7 rad (40 deg) |
| 踝关节力矩范围 | 0~46 Nm |
| IMU 加速度 | [-0.53, 0.50, 9.67] m/s² (静态) |
| IMU 陀螺仪 | [0.03, 0.11, 0.01] rad/s (静态) |

---

## 5. 实现计划

### 5.1 技术栈

| 组件 | 选型 | 理由 |
|------|------|------|
| 语言 | Python 3 | 开发速度快，6 天可完成 |
| 运动学 | PyKDL + urdf_parser_py | ROS2 生态自带 |
| 矩阵运算 | NumPy / SciPy | EKF 矩阵运算 |
| 姿态表示 | scipy.spatial.transform.Rotation | SO(3) 运算 |
| ROS2 接口 | rclpy | 订阅/发布 |
| 可视化 | Isaac Sim (ROS2 Bridge) + NVIDIA Spark | 3D 机器人模型 + 轨迹可视化 |

### 5.2 代码结构

```
leg_odometry/
├── doc/
│   └── leg_odometry_proposal.md      # 本文档
├── config/
│   ├── joint_mapping.yaml            # 关节映射表（含符号/偏移修正）
│   └── ekf_params.yaml               # EKF 参数
├── scripts/
│   ├── joint_state_remapper.py       # 关节名转换节点
│   └── leg_odometry_node.py          # 主节点
├── leg_odometry/                     # Python 模块
│   ├── __init__.py
│   ├── ekf.py                        # EKF 核心实现
│   ├── kinematics.py                 # URDF/KDL 运动学封装
│   └── contact_detector.py           # 接触检测
├── launch/
│   ├── verify_mapping.launch.py      # 阶段1: Isaac Sim 映射验证
│   └── leg_odometry.launch.py        # 阶段2: 里程计运行
├── package.xml
├── setup.py
└── test/
    └── test_ekf.py                   # 单元测试
```

### 5.3 排期

| 日期 | 任务 | 交付物 |
|------|------|--------|
| **Day 1** | 搭建 ROS2 包骨架；实现 `joint_state_remapper`；Isaac Sim 导入 URDF + 配置 ROS2 Bridge | 可运行的 remapper 节点；Isaac Sim 中可见机器人模型 |
| **Day 2** | Isaac Sim 回放 bag 验证映射；修正映射表中的符号/偏移直到动作正常 | 验证通过的 `joint_mapping.yaml` |
| **Day 3** | 实现 KDL 运动学封装（FK 计算）；实现接触检测（力矩阈值 + 滞后）；EKF 框架（状态定义 + 预测步骤 + 协方差传播） | `kinematics.py` + `contact_detector.py` + `ekf.py` 预测部分 |
| **Day 4** | 实现 EKF 更新步骤（观测模型 + 雅可比 + 卡尔曼增益）；串联完整 EKF 流程 | 完整 `ekf.py` |
| **Day 5** | 实现 ROS2 节点封装；回放 bag 跑通全流程；输出 `nav_msgs/Odometry` + TF | `leg_odometry_node.py`；Isaac Sim 中可见轨迹 |
| **Day 6** | 实现纯 IMU 积分基线；对比两条轨迹；调参优化；整理文档和结果 | 对比报告 + 调优后的参数 |

---

## 6. 验证方法

### 6.1 定性验证

- Isaac Sim 中同时显示机器人模型和里程计轨迹
- 轨迹应呈现合理的行走路径（直线/转弯），无跳变
- Z 轴（高度）应保持稳定，不应持续上飘或下沉

### 6.2 定量对比

| 方法 | 预期漂移率 | 说明 |
|------|-----------|------|
| 纯 IMU 积分 | 严重漂移（秒级发散） | 基线，加速度二次积分 |
| Bloesch EKF | < 2% 行走距离 | 运动学约束抑制漂移 |

对比指标：
- 轨迹终点回到原点的闭合误差（如果数据中有回环）
- 与 IMU 积分的漂移对比曲线
- 竖直方向（Z 轴）高度稳定性
- 接触检测准确率（与踝关节力矩变化对比）

### 6.3 关键调参项

| 参数 | 初始值 | 调整依据 |
|------|--------|----------|
| $\sigma_{contact}$ | 0.01 m/$\sqrt{s}$ | Z 轴漂移量 |
| $\sigma_{swing}$ | 1.0 m/$\sqrt{s}$ | 摆动腿是否干扰估计 |
| $\tau_{threshold}$ | 5 Nm | 接触检测准确率 |
| 加速度计噪声密度 | 0.1 m/s² | IMU datasheet |
| 陀螺仪噪声密度 | 0.01 rad/s | IMU datasheet |
| 加速度偏置随机游走 | 0.001 m/s²/$\sqrt{s}$ | 偏置收敛速度 |
| 陀螺偏置随机游走 | 0.0001 rad/s/$\sqrt{s}$ | 偏置收敛速度 |
| FK 观测噪声 | 0.01 m | 运动学精度 |

---

## 7. 标定策略

### 7.1 本期标定（Day 1-2，必须完成）

| 标定项 | 方法 | 工具 |
|--------|------|------|
| 关节名映射 | Isaac Sim 可视化验证 | `joint_mapping.yaml` |
| 关节符号（正负） | 观察关节运动方向 | `joint_mapping.yaml` 中 `joint_sign` 字段 |
| 关节零位偏移 | 观察静态姿态 | `joint_mapping.yaml` 中 `joint_offset` 字段 |

### 7.2 后续标定（本期不做，记录备查）

| 标定项 | 影响 | 参考方法 |
|--------|------|----------|
| URDF 连杆尺寸精度 | FK 系统误差 → 持续漂移 | Cerberus "Online Kinematic Calibration" (IROS 2022) |
| IMU-base_link 外参 | IMU 和 FK 坐标系不一致 | 标准 IMU 外参标定流程 |
| IMU-关节时间偏移 | 预测/更新异步 → 震荡 | 互相关分析 |
| IMU 内参 | 偏置/尺度因子/轴间不正交 | Allan 方差分析 / Kalibr |

---

## 8. 风险与应对

| 风险 | 概率 | 影响 | 应对 |
|------|------|------|------|
| 关节映射有误 | 高 | FK 计算错误，里程计不可用 | Day 1-2 优先验证，Isaac Sim 可视化逐关节确认 |
| URDF 尺寸不准 | 中 | FK 有系统误差，漂移增大 | 本期先容忍；后续参考 Cerberus 在线标定 |
| 接触检测不准 | 中 | 错误接触假设导致位置跳变 | 阈值 + 滞后 + Isaac Sim 可视化调试 |
| Python 性能不足 | 低 | 200Hz 处理不及时 | 降频至 100Hz 或关键路径 NumPy 向量化 |
| PyKDL 安装问题 | 低 | 无法计算 FK | 备选：手写 FK（仅 6 关节链，矩阵连乘） |
| Isaac Sim 配置困难 | 中 | 阻塞 Day 1-2 | 备选：RViz2 可视化（已有现成 launch 文件） |

---

## 9. 后续演进路线

```
本期 (6天)                    近期                         远期
┌─────────────┐      ┌──────────────────┐      ┌─────────────────────┐
│ Bloesch EKF  │      │  接入 GLIM       │      │  完整多模态融合      │
│ Python 实现   │ ───▶ │  ExtensionModule │ ───▶ │  LiDAR+IMU+Leg+视觉  │
│ 独立验证      │      │  BetweenFactor   │      │  因子图联合优化       │
└─────────────┘      └──────────────────┘      └─────────────────────┘
                            │                          │
                     ┌──────┴──────┐            ┌──────┴──────┐
                     │ 足端预积分   │            │ 在线运动学   │
                     │ (参考 LVI-Q) │            │ 标定         │
                     └─────────────┘            │ (参考        │
                                                │  Cerberus)   │
                                                └──────────────┘
```

| 阶段 | 内容 | 参考 |
|------|------|------|
| **本期** | Bloesch EKF，Python，独立 ROS2 节点 | Bloesch 2013 |
| **近期** | 接入 GLIM 因子图；C++ 重写；足端预积分替代位置法 | LVI-Q 2025, STEP 2022 |
| **远期** | 在线运动学标定；软接触概率；多传感器联合优化 | Cerberus IROS 2022, VILENS 2022 |

---

## 10. 参考文献

1. Bloesch M., Hutter M., et al. "State Estimation for Legged Robots - Consistent Fusion of Leg Kinematics and IMU", *Robotics: Science and Systems (RSS)*, 2013.
2. Marsim K., Oh M., et al. "LVI-Q: Robust LiDAR-Visual-Inertial-Kinematic Odometry for Quadruped Robots Using Tightly-Coupled and Efficient Alternating Optimization", *arXiv:2510.15220*, 2025.
3. Yang S., Zhang Z., et al. "Cerberus: Low-Drift Visual-Inertial-Leg Odometry For Agile Locomotion", *IEEE ICRA*, 2023.
4. Yang S., Zhang Z., Bokser B., Manchester Z. "Multi-IMU Proprioceptive Odometry for Legged Robots", *IEEE/RSJ IROS*, 2023. **(Best Paper Finalist)**
5. Yang S., Choset H., Manchester Z. "Online Kinematic Calibration for Legged Robots", *IEEE RA-L + IROS*, 2022.
6. Kim Y., et al. "STEP: State Estimator for Legged Robots Using a Preintegrated Foot Velocity Factor", *IEEE RA-L*, 2022.
7. Wisth D., Camurri M., Fallon M. "VILENS: Visual, Inertial, Lidar, and Leg Extended Navigation System", *IEEE Trans. Robotics*, 2022.
8. Hartley R., et al. "Contact-Aided Invariant Extended Kalman Filter for Legged Robot State Estimation", *Robotics: Science and Systems (RSS)*, 2020.
9. A²I-Calib: "Anti-noise Active Multi-IMU Spatial-Temporal Calibration for Legged Robots", *IEEE/RSJ IROS*, 2025. https://github.com/DavidGrayrat/A2I-Calib

---

## 11. 进展记录

### Day 1 (2026-04-05) — 已完成

#### 完成事项

1. **ROS2 包搭建** — `src/leg_odometry/` 独立包，`colcon build` 通过
2. **关节映射验证** — RViz 回放 bag 数据，确认映射表正确，腿部动作自然
3. **核心代码实现**
   - `ekf.py` — Bloesch error-state EKF，24 维状态
   - `kinematics.py` — PyKDL 正运动学，手写 URDF→KDL 解析（不依赖 kdl_parser_py）
   - `contact_detector.py` — 踝关节力矩阈值 + 滞后
   - `joint_state_remapper.py` — bag 关节名 → URDF 关节名转换
   - `leg_odometry_node.py` — 完整 ROS2 节点
   - launch 文件 × 2
4. **Bug 修复** — 观测方程残差符号错误，导致 EKF 发散。已修复。
5. **仿真验证通过**
   - 模拟匀速直线行走 0.83 m/s × 10s
   - EKF 输出 X=8.287m vs Ground Truth X=8.296m
   - **XY 漂移率 0.12%**，算法正确
6. **真实数据测试**
   - 传送带数据：EKF 不再发散（终点 XY=12m），符合预期
   - Z 轴漂移 17m（偏置锁死导致），后续需放开偏置估计
7. **Isaac Sim 安装** — DGX Spark 上源码编译成功（11 分钟）
8. **文件传输** — URDF、meshes、代码已同步到 Spark

#### 关键发现

- **数据集是传送带录制**：机器人站在传送带上行走，身体实际静止。IMU 和腿部运动"矛盾"，不适合直接验证里程计精度。
- **观测方程符号**：残差应为 `y = -(h(x))` 其中 `h(x) = p + R*FK - p_foot`。H 矩阵中 p 对应 `+I`，p_foot 对应 `-I`。写反会导致 EKF 向错误方向更新。
- **偏置估计**：当前锁死偏置（walk noise = 0），用初始帧估计的固定值。放开后可能改善 Z 漂移，但需要谨慎调参防止发散。

#### 遗留问题

| 问题 | 优先级 | 说明 |
|------|--------|------|
| 需要真实地面行走数据 | 高 | 传送带数据无法验证里程计精度；可用 Isaac Sim 生成仿真数据替代 |
| Z 轴漂移 | 中 | 偏置锁死导致；需放开加速度偏置在线估计 |
| IMU 偏置在线估计 | 中 | 需要合理的初始协方差和游走噪声参数 |
| EKF 初始化 Z 偏移 | 低 | 初始 Z=0 但真实高度=腿长，可在初始化时修正 |

### Day 2+ — 待完成

| 任务 | 说明 |
|------|------|
| Isaac Sim 仿真数据生成 | 导入 URDF，驱动行走，录制带 Ground Truth 的 rosbag |
| 放开 IMU 偏置估计 | 调参让偏置收敛但不发散 |
| 真实数据验证 | 等上班后录地面行走数据 |
| 性能优化 | 当前离线处理 ~4500 msg/s，实时需 ~400 msg/s，已满足 |
