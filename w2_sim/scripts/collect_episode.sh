#!/usr/bin/env bash
# scripts/collect_episode.sh <episode编号> [时长秒,默认240] [ENV,默认warehouse]
# 产出 ~/Documents/Datasets/w2_sim/episode_NNNN/{rosbag2,episode_params.yaml,ground_truth_tum.txt}
set -euo pipefail
EP=${1:?用法: collect_episode.sh <episode编号> [时长] [ENV warehouse|ground]}
DUR=${2:-240}
ENV=${3:-warehouse}          # 生产默认 warehouse;冒烟可传 ground(平地,免云资产下载)
SEED=$((1000 + EP))
PKG="$(cd "$(dirname "$0")/.." && pwd)"
# 数据集按场景组织:w2_sim/<场景>/episode_NNNN/
OUT=$HOME/Documents/Datasets/w2_sim/$ENV/$(printf 'episode_%04d' "$EP")
mkdir -p "$OUT"
# ros2 bag record 拒绝写已存在目录(实测 "Output folder ... already exists" 直接退出,
# 旧 bag 原封不动 → 重采集静默失败,采到的还是上一版数据)。重采前先删旧 rosbag2。
rm -rf "$OUT/rosbag2"

set +u                       # ROS2 setup.bash 引用未定义变量,与 set -u 冲突
source /opt/ros/jazzy/setup.bash
source $HOME/rtabmap_ws/install/setup.bash
set -u
# RMW:统一默认 FastDDS(本机 jazzy 无 cyclonedds),不要 export RMW_IMPLEMENTATION
: "${ISAACSIM_DIR:?需要 export ISAACSIM_DIR}"

# 退出清理:kill jobs 只杀 `ros2 run` wrapper,会留下子进程 degradation_node 孤儿
# (它会污染下一次采集),所以再按进程名补杀一遍。
cleanup() {
  kill $(jobs -p) 2>/dev/null || true
  pkill -9 -f "w2_sim.degradation_node"    2>/dev/null || true
  pkill -9 -f "w2_sim.lidar_pc2_publisher" 2>/dev/null || true
  pkill -9 -f "isaac/w2_sim_app.py"        2>/dev/null || true
}
trap cleanup EXIT

# 开跑前清掉任何 stale 实例:多个 degradation_node 会让 /robot/wheel_status 速率成倍叠加
# (N 个实例 → N× 速率);残留 Isaac/topic-pub 同样污染数据。务必单实例。
pkill -9 -f "w2_sim.degradation_node"    2>/dev/null || true
pkill -9 -f "w2_sim.lidar_pc2_publisher" 2>/dev/null || true
pkill -9 -f "isaac/w2_sim_app.py"        2>/dev/null || true
pkill -9 -f "topic pub /sim"             2>/dev/null || true
rm -f /tmp/w2_sim_lidar.sock 2>/dev/null || true
sleep 2

ros2 run w2_sim degradation_node --ros-args \
  -p seed:="$SEED" -p params_out:="$OUT/episode_params.yaml" &

# 雷达点云(带 intensity)sidecar:必须先于 Isaac 起好、监听 UNIX socket,
# Isaac 主程序通过该 socket 推 xyz+intensity,本节点组 PointCloud2 发 /rslidar_points。
# (Isaac 嵌入式 cp311 python 无法干净用系统 cp312 rclpy,详见 isaac/w2_sim_app.py 注释。)
ros2 run w2_sim lidar_pc2_publisher &
sleep 2   # 给 sidecar 时间 bind+listen,避免 Isaac 首帧连不上

"$ISAACSIM_DIR/python.sh" "$PKG/isaac/w2_sim_app.py" \
  --seed "$SEED" --duration "$DUR" --env "$ENV" &
SIM_PID=$!

echo "等待仿真话题上线..."
# 1800s:重场景(full_warehouse/hospital)首次加载要编译大量 MDL 材质,300s 不够
timeout 1800 bash -c \
  'until ros2 topic info /sim/joint_states 2>/dev/null | grep -q "Publisher count: [1-9]"; do sleep 2; done'

ros2 bag record -o "$OUT/rosbag2" \
  /robot/wheel_status /rslidar_imu_data /rslidar_points \
  /camera/color/image_raw /camera/color/camera_info \
  /sim/ground_truth_odom /sim/joint_states /sim/imu /tf /clock &
BAG_PID=$!

wait $SIM_PID                  # 仿真自然结束
sleep 2
# 实测:jazzy 的 `ros2 bag record` CLI 只 catch SIGTERM、不 catch SIGINT
# (SigCgt 不含 bit1)。必须用 SIGTERM 才会优雅刷盘并写 metadata.yaml;
# SIGINT 不会停它,导致 export 读到未 finalize 的 bag(无 metadata→storage 无法初始化)。
kill -TERM "$BAG_PID" 2>/dev/null || true
wait "$BAG_PID" 2>/dev/null || true        # 阻塞直到 recorder 退出
# 安全网:等 metadata.yaml 落盘(recorder 刷大 mcap 可能比进程退出稍慢)
for _ in $(seq 1 30); do
  [ -f "$OUT/rosbag2/metadata.yaml" ] && break
  sleep 1
done

python3 "$PKG/scripts/export_tum.py" "$OUT/rosbag2" "$OUT/ground_truth_tum.txt"
ros2 bag info "$OUT/rosbag2"
echo "EPISODE_OK $OUT"
