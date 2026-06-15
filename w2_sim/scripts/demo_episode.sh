#!/usr/bin/env bash
# scripts/demo_episode.sh [时长秒,默认120] [ENV,默认warehouse] [--record]
#
# 数据生产可视化 demo:一条命令同时起
#   (1) Isaac Sim GUI  —— w2 在仓库自主行驶、轮子转向(=数据源,看场景)
#   (2) degradation_node —— 给干净轮速注入标定偏差,产实车格式 /robot/wheel_status
#   (3) rviz2(config/demo.rviz) —— 实际录下的传感器数据 + 真值轨迹(=产出,看数据)
#
# 默认不录包(专注观看)。加 --record 同时 ros2 bag record 到
#   ~/Documents/Datasets/w2_sim/demo_<时间戳>/rosbag2 。
#
# 关键:Isaac 只发 odom->base_link 的 TF,传感器帧(rslidar/camera_link/imu_link)
# 是 base_link 的子帧但 Isaac 不发它们的静态 TF。本脚本用 static_transform_publisher
# 按 urdf/W2EVT1Urdf_swerve.urdf.xacro 的外参补这三条,rviz 才能把 rslidar 帧的点云
# 变换进 odom 固定帧显示(否则点云“No transform”不显示)。
set -euo pipefail

DUR=120
ENV=warehouse
RECORD=0
for a in "$@"; do
  case "$a" in
    --record) RECORD=1 ;;
    warehouse|ground) ENV="$a" ;;
    ''|*[!0-9]*) ;;          # 非纯数字、非已知关键字 → 忽略
    *) DUR="$a" ;;
  esac
done

SEED=42
PKG="$(cd "$(dirname "$0")/.." && pwd)"

set +u                       # ROS2 setup.bash 引用未定义变量,与 set -u 冲突
source /opt/ros/jazzy/setup.bash
source "$HOME/rtabmap_ws/install/setup.bash"
set -u
# RMW:统一默认 FastDDS(本机 jazzy 无 cyclonedds),不要 export RMW_IMPLEMENTATION
export DISPLAY=:1            # DGX Spark 有物理显示器,GUI 直接上屏
: "${ISAACSIM_DIR:?需要 export ISAACSIM_DIR (Isaac Sim release 目录)}"

# 退出清理:kill jobs 只杀 wrapper,子进程(degradation/static_tf/rviz/bag)会成孤儿,
# 务必按进程名补杀,别留孤儿污染下一次。
cleanup() {
  kill $(jobs -p) 2>/dev/null || true
  pkill -9 -f "w2_sim.degradation_node" 2>/dev/null || true
  pkill -9 -f "isaac/w2_sim_app.py"     2>/dev/null || true
  pkill -9 -f "demo.rviz"               2>/dev/null || true
  pkill -9 -f "static_transform_publisher" 2>/dev/null || true
}
trap cleanup EXIT

# 开跑前清 stale(多实例会让话题速率成倍叠加 / 残留进程污染显示)
pkill -9 -f "w2_sim.degradation_node"    2>/dev/null || true
pkill -9 -f "isaac/w2_sim_app.py"        2>/dev/null || true
pkill -9 -f "demo.rviz"                  2>/dev/null || true
pkill -9 -f "static_transform_publisher" 2>/dev/null || true
sleep 2

echo "[demo] ENV=$ENV DUR=${DUR}s RECORD=$RECORD SEED=$SEED"

# --- 静态 TF:base_link -> 各传感器帧(外参取自 swerve urdf) ---
# rslidar:  xyz 0.25 0 0.55  rpy 0 1.01 0  (前倾约 58°)
ros2 run tf2_ros static_transform_publisher \
  --x 0.25 --y 0 --z 0.55 --roll 0 --pitch 1.01 --yaw 0 \
  --frame-id base_link --child-frame-id rslidar &
# imu_link: 与 rslidar 同位姿(urdf 里 rslidar->imu 为单位变换)
ros2 run tf2_ros static_transform_publisher \
  --x 0.25 --y 0 --z 0.55 --roll 0 --pitch 1.01 --yaw 0 \
  --frame-id base_link --child-frame-id imu_link &
# camera_link: xyz 0.30 0 0.50  rpy 0 0 0
ros2 run tf2_ros static_transform_publisher \
  --x 0.30 --y 0 --z 0.50 --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id camera_link &

# --- 退化层(数据源 -> 实车格式) ---
ros2 run w2_sim degradation_node --ros-args \
  -p seed:="$SEED" -p params_out:="/tmp/demo_episode_params.yaml" &

# --- Isaac Sim GUI(数据源,上屏) ---
"$ISAACSIM_DIR/python.sh" "$PKG/isaac/w2_sim_app.py" \
  --seed "$SEED" --duration "$DUR" --env "$ENV" --gui &
SIM_PID=$!

echo "[demo] 等待仿真话题 /sim/joint_states 上线 (Isaac GUI 加载较慢)..."
timeout 300 bash -c \
  'until ros2 topic info /sim/joint_states 2>/dev/null | grep -q "Publisher count: [1-9]"; do sleep 2; done' \
  || { echo "[demo] 超时:仿真话题未上线"; exit 1; }
echo "[demo] 话题已上线,启动 rviz2"

# --- rviz2(产出数据可视化) ---
rviz2 -d "$PKG/config/demo.rviz" --ros-args -p use_sim_time:=true &

# --- 可选:录包 ---
if [ "$RECORD" = "1" ]; then
  OUT="$HOME/Documents/Datasets/w2_sim/demo_$(date +%Y%m%d_%H%M%S)"
  mkdir -p "$OUT"
  echo "[demo] 录包 -> $OUT/rosbag2"
  ros2 bag record -o "$OUT/rosbag2" \
    /robot/wheel_status /rslidar_imu_data /rslidar_points \
    /camera/color/image_raw /camera/color/camera_info \
    /sim/ground_truth_odom /sim/joint_states /sim/imu /tf /tf_static /clock &
  BAG_PID=$!
fi

echo "[demo] 运行中 —— 看 Isaac GUI(机器人行驶)+ rviz2(点云/相机/真值轨迹)"
wait $SIM_PID                  # 仿真自然结束(到 duration)
echo "[demo] 仿真结束"
if [ "$RECORD" = "1" ]; then
  sleep 2
  kill -TERM "${BAG_PID:-0}" 2>/dev/null || true   # jazzy recorder 只 catch SIGTERM
  wait "${BAG_PID:-0}" 2>/dev/null || true
  echo "[demo] 录包已写出: $OUT/rosbag2"
fi
echo "[demo] 完成(cleanup 由 trap 处理)"
