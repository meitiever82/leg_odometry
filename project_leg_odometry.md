---
name: leg_odometry 项目状态与未修复issue清单
description: CASBot02 腿式里程计 (C++ ESKF+GTSAM Hybrid)：未修复 issue、实机测试结果、sim-to-real 假设、构建/工具链注意事项
type: project
---

`src/leg_odometry/` 是 CASBot02 人形机器人腿式里程计，C++ ESKF 200Hz 前端 + GTSAM 滑窗后端。**架构、精度（sim 9 场景）、参数全部在 `src/leg_odometry/CLAUDE.md` 和 `src/leg_odometry/doc/progress_report.md`，需要时直接读这两个文件，不要在 memory 里复述。**

**Why:** 项目自带文档已经覆盖架构和数据，memory 只补不在文档里的东西。

**How to apply:** 用户问 leg_odometry 相关问题时，先 read CLAUDE.md 和 progress_report.md 获取最新状态。

---

## 实机测试结果（point-in-time, 2026-04-08）

第一次跑真实 bag (`rosbag2_2026_04_07-16_12_13`, 103 办公室, 起终点应重合, 334s)：

| 指标 | 值 |
|---|---|
| 估计 XY 路径长度 | **224 m**（办公室不可能这么长） |
| 起终点 XY 距离 | 109 m |
| **drift / path** | **48.7%** |
| Z range | 0.003 m（flat-z 约束生效） |

**关键观察：路径长度严重 over-count**——不只是 drift 大，是速度估计完全飞了。Sim 上同等"难度"是 curve_walk 49%，实机也是 49%——巧合地相同。

**Why:** 第一次有真机数据点。memory 没存的话，下次回来很容易把 sim 的数字当成真机表现。

**How to apply:** 下次讨论"算法精度"时区分 sim 和实机；调参方向应该先做 sim-to-real 而不是改算法结构。

## Sim-to-Real 假设（待验证）

实机 224m 路径长度的可能原因（按嫌疑排序）：

1. **接触检测阈值不匹配**：sim 调出来的 `effort_threshold=5.0` Nm。实机 LJPITCH/RJPITCH 静态都在 7.4 Nm 量级，可能让 contact 状态反常（"双脚一直接触"或"始终 swing"），ZUPT 失效
2. **IMU 噪声远高于 sim**：`accel_noise=0.1` 是 sim 假设；真实 IMU 待测
3. **关节零位/方向未在实机校准**：`config/joint_mapping.yaml` 里 `joint_offset` 全 0、`joint_sign` 全 1，可能从未在真机验证

**Why:** 不验证就改算法等于盲目。

**How to apply:** 下次工作 leg_odometry 实机精度时，先按这三条排查；写诊断脚本看 contact 状态时间序列、ZUPT 触发率、estimated velocity 大小。

---

## 构建/工具链注意事项（2026-04-08）

- **平台**：aarch64 (NVIDIA Spark/Jetson 类)，ROS **Jazzy**（不是 CLAUDE.md 写的 Humble）
- **GTSAM 4.3**：在 `/usr/local/`（aarch64 系统安装），CMakeLists 用 `find_package(GTSAM REQUIRED)` 不要加 HINTS
- **历史坑**：`~/Documents/local/gtsam-4.3/` 曾经是 x86_64 stale 二进制（已清理 2026-04-08）。如果未来 CMakeLists 又出现 GTSAM HINTS 指向某个具体路径，多半是不必要的
- **CMakeCache 污染**：跨多次失败 build 时 CMakeCache 里 `GTSAM_INCLUDE_DIR` 是 CACHE PATH，新的 `find_package` 不会覆盖，必须 `rm -rf build/leg_odometry` 完全清掉再 colcon build。GTSAMConfig.cmake 在第 9 行用 `set(... CACHE PATH)` 写 include dir
- **KDL/URDF**：用 `find_package(kdl_parser)` `find_package(urdf)` + `ament_target_dependencies(... kdl_parser urdf)`，不要硬编码 `/opt/ros/<distro>` 路径

**Why:** 这些都是今天 build leg_odometry 实际踩过的坑，下次再遇到能直接跳到解法。

## 测试基础设施教训

- **`ros2 bag record` 通过 python wrapper 启动，SIGINT 不能可靠传递到底层 recorder**。两个 recorder 同时写一个 mcap 会让文件 corrupt（mcap reader 找不到 footer）
- **替代方案**：写 rclpy subscriber 直接 dump CSV。已实现在 `scripts/dump_leg_odom_csv.py`，配套 `scripts/eval_drift.py` 算 loop-closure drift，`scripts/run_bag_test.sh` 一键串起来
- **进程清理**：测试结束后用 `pkill -KILL -f "leg_odom_node\|robot_state_publisher"` 兜底，否则 orphan recorder 会干扰下次运行

**Why:** 一晚上踩了两次"orphan recorder + corrupt mcap"，下次直接用 rclpy 路径

---

## 上次 review 发现的未修复 issue（2026-04-08 验证仍在代码里）

---

## 上次 review 发现的未修复 issue（2026-04-08 验证仍在代码里）

按优先级排序：

1. **【最有价值】GTSAMSmoother 没用速度类 factor**
   - `include/leg_odometry/smoother/GTSAMSmoother.h:106-145` 的 optimize() 只用了 `FKPositionFactor` + 接触 `BetweenFactor` + `FlatZPositionFactor` + `ImuFactor`
   - 定义在 `leg_factors.h` 的 `FootVelocityFactor` 和 `FlatZVelocityFactor` 没被使用
   - **假设**：long_walk 42% / curve_walk 49% 漂移大，可能是因为前端 ZUPT 强但后端没保留等价约束，gyro bias 优化时被牵走
   - **下一步动作**：在 smoother 加 FootVelocityFactor，验证 long_walk 是否能降下来

2. **leg_odom_node.cpp init bug**
   - `src/leg_odom_node.cpp:156-181` 检测到未静止时只 return（line 167），不重置 `init_accel_sum_` / `init_gyro_buf_`
   - 后果：开机时不静止则初始 R 和 b_a 被污染，且 `init_gyro_buf_` 无限增长
   - 修法：滑窗 reset，或失败时清空重来

3. **smoother bias_walk 写死，没读 yaml**
   - `src/leg_odom_node.cpp:107` 和 `src/leg_odom_hybrid.cpp:91-92` 都硬编码 `0.005, 0.002`
   - 应该走 `declare_parameter`，否则改 yaml 不生效

4. **UpdaterZUPT::compute_v_expected dt_fk 硬编码**
   - `include/leg_odometry/update/UpdaterZUPT.h:74` 写死 `dt_fk = 0.005`
   - 实机 joint_states 不一定刚好 200Hz，jitter 直接进 R*dFK/dt
   - 修法：传 `s.last_dt` 或测一次 joint_states 间隔

5. **State::initialize 的 b_a 表达式不干净**
   - `include/leg_odometry/state/State.h:117` `b_a = accel - R.transpose() * Eigen::Vector3d(0, 0, 9.81)`
   - 因为 R 是从 `g_body = -accel.normalized() * 9.81` 反解的，当 |accel|≈9.81 时这表达式≈0
   - 但 |accel| 偏离 9.81（IMU 标定误差）时会被初始化成非零值，又因为 sigma_ba=0 永久锁住
   - 修法：直接 `b_a.setZero()`

6. **GTSAMSmoother 每次重建整张图**
   - `include/leg_odometry/smoother/GTSAMSmoother.h:78-79` 每次 optimize 从零做 LM
   - 不影响当前精度（20Hz 关键帧够），但提到 "100-200Hz" 关键帧方向时会变瓶颈
   - 升级方向：iSAM2 + marginalization

7. **FootVelocityFactor 用数值微分**
   - `include/leg_odometry/smoother/leg_factors.h:108-120` 用 `numericalDerivative21`
   - 注释写 "Numerical derivative for simplicity"
   - 当前没被 smoother 用上（见 issue 1），如果启用需要换解析雅可比

## 其他观察

- node (leg_odom_node.cpp) 和 hybrid (leg_odom_hybrid.cpp) 的 init 路径不对称：node 有 gyro_std 静止检测但有 reset bug；hybrid 直接平均前 50 帧无检测（仿真里 OK，跑实机会有问题）
- 节点里 FK 用 `noise_.sigma_zupt` 没 declare_parameter（其实 declare 了，是 declare 完但没存到 noise_，行 47 有声明）—— 这条不算 bug，已声明
