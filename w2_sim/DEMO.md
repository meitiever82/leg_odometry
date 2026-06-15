# w2_sim 数据生产可视化 Demo

一句话:**在 Isaac Sim 里仿真一台 w2 swerve 底盘机器人,在仓库自主行驶,实时产出用于训练「学习型轮速里程计」的数据,并在 rviz2 里同步看到这些数据被一点点生产出来。**

## 方法的故事(demo 要讲清的)

1. **完美仿真 → 精确真值。** 机器人在 Isaac 物理引擎里行驶,刚体位姿是完美 ground truth(无漂移、无噪声),作为训练标签。
2. **传感器实时产出。** 轮速(关节)、IMU、雷达点云、前视相机由 Isaac 实时生成,格式与真实采集一致。
3. **退化层注入标定偏差。** `degradation_node` 给「干净」的仿真轮速注入转向零点偏差 / 轮速尺度误差 / IMU 噪声,得到「实车格式」的真实轮速数据 —— 复现真车上观察到的 κ 标定偏差现象(轮速纯航推会以约 1.9°/s 漂移航向)。
4. **录成 ROS2 bag** 用于训练:输入是退化后的轮速/IMU,标签是完美真值轨迹。

学习型里程计要学的,正是「从带标定偏差的轮速 → 真值运动」这个映射。完美仿真让我们能**无限量、零成本**地生产带精确标签的训练数据。

## 怎么跑

前置(每个新 shell 都要):

```bash
export ISAACSIM_DIR=$HOME/Documents/GitHub/system/IsaacSim/_build/linux-aarch64/release
# DISPLAY=:1 由脚本内部设好;本机有物理显示器,GUI 直接上屏
cd ~/rtabmap_ws/src/other_odometry/w2_sim
```

确保 `w2_sim` 已 build 过(degradation_node 需要):

```bash
cd ~/rtabmap_ws && colcon build --packages-select w2_sim && cd -
```

运行 demo(默认 120s,warehouse 场景,不录包):

```bash
./scripts/demo_episode.sh
```

参数(位置无关,可任意组合):

```bash
./scripts/demo_episode.sh 60                 # 跑 60 秒
./scripts/demo_episode.sh ground             # 平地场景(免云资产下载,适合首次冒烟)
./scripts/demo_episode.sh 180 warehouse --record   # 跑 180s 并同时录包
```

`--record` 会把数据写到 `~/Documents/Datasets/w2_sim/demo_<时间戳>/rosbag2`。
退出(Ctrl-C 或仿真到时)会自动清理所有子进程(Isaac / degradation / rviz / static_tf / bag)。

## 打开后你在看什么

### Isaac Sim 窗口(=数据源)
- **仓库场景 + w2 机器人在自主行驶。** 轨迹由 seed 决定的运动 profile 生成,出安全框会自动朝中心回驶。
- **4 个 swerve 模组的轮子在实时转向 + 滚动** —— 这就是轮速/转向角数据的物理来源。盯着轮子看转向角变化,正是退化层后面要「污染」的那个量。
- 机器人位姿是物理引擎算的完美真值。

### rviz2 窗口(=产出的数据)
- **点云(白/彩,`/rslidar_points`)** —— RTX Lidar(OS1 32 线)扫出来的仓库结构。按 Z 轴高度上色。
- **前视相机(`/camera/color/image_raw`)** —— 左下 Image 面板,机器人第一视角 RGB。
- **真值轨迹(`/sim/ground_truth_odom`,红色坐标轴串)** —— 机器人每走一步就在 `odom` 帧里画一个位姿轴。**这串轴一点点延伸出来,就是「数据被生产」最直观的画面。**
- **TF 树** —— `odom → base_link → {rslidar, camera_link, imu_link}`,看机器人各坐标系。

视角默认 Orbit 跟随 `base_link`(镜头跟着机器人走),固定帧是 `odom`(世界不动,真值轨迹在世界里累积)。

### 命令行 / 话题(退化层在做什么)
后台 `degradation_node` 订阅 Isaac 的干净 `/sim/joint_states` + `/sim/imu`,发布:
- `/robot/wheel_status`(50Hz,实车格式轮速,**已注入标定偏差**)
- `/rslidar_imu_data`(200Hz,**已注入 IMU 噪声**)
- 把本次采样的扰动参数写到 `/tmp/demo_episode_params.yaml`(含理论 κ 值)。

对比 `/sim/joint_states`(干净)和 `/robot/wheel_status`(退化后)能直观看到偏差被注入。

## 话题一览

| 话题 | 内容 | 角色 |
|---|---|---|
| `/sim/ground_truth_odom` | 完美真值位姿 | 训练标签 |
| `/sim/joint_states` | 干净轮速/转向(8 关节) | 退化层输入 |
| `/sim/imu` | 干净 IMU | 退化层输入 |
| `/robot/wheel_status` | 退化后实车格式轮速 | 训练输入 |
| `/rslidar_imu_data` | 退化后 IMU | 训练输入 |
| `/rslidar_points` | 雷达点云 | 辅助 / 可视化 |
| `/camera/color/image_raw` | 前视 RGB | 辅助 / 可视化 |
| `/tf`, `/clock` | odom→base_link、仿真时钟 | 系统 |

## 截图 / 录屏建议

- **最佳构图:Isaac 窗口在左、rviz 在右并排。** 左边看机器人/轮子(数据源),右边看点云+真值轨迹(产出),一眼看懂「仿真 → 数据」。
- **录屏挑机器人转弯的片段** —— 此时轮子转向角变化最明显(Isaac 里看轮子摆),同时 rviz 里真值轨迹画出弧线,点云随视角扫过仓库货架。
- **轨迹累积特写:** 让 demo 跑 30–60s 后,rviz 里那串红色坐标轴已铺出一条清晰路径,适合定格说明「这一整条带标签的轨迹就是一条训练样本」。
- **退化对比:** 终端并排 `ros2 topic echo /sim/joint_states` 与 `ros2 topic echo /robot/wheel_status`,展示同一时刻干净 vs 带偏差的轮速数值差异。

## TF / 显示排查

- Isaac 只发 `odom→base_link`。传感器帧(`rslidar`/`camera_link`/`imu_link`)的静态 TF 由 demo 脚本用 `static_transform_publisher` 按 `urdf/W2EVT1Urdf_swerve.urdf.xacro` 的外参补上。
- 若点云不显示("No transform from [rslidar]"):确认那三个 static_transform_publisher 进程在跑(`pgrep -af static_transform_publisher`),且 rviz Fixed Frame = `odom`。
- 点云/相机话题是 BEST_EFFORT QoS;rviz 配置里对应显示已设为 Best Effort,不用改。
