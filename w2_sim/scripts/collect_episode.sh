#!/usr/bin/env bash
# scripts/collect_episode.sh <episode编号> [时长秒,默认240] [ENV,默认warehouse]
# 产出 ~/Documents/Datasets/w2_sim/episode_NNNN/{rosbag2,episode_params.yaml,ground_truth_tum.txt}
set -euo pipefail
EP=${1:?用法: collect_episode.sh <episode编号> [时长] [ENV warehouse|ground]}
DUR=${2:-240}
ENV=${3:-warehouse}          # 生产默认 warehouse;冒烟可传 ground(平地,免云资产下载)
SEED=$((1000 + EP))
PKG="$(cd "$(dirname "$0")/.." && pwd)"
OUT=$HOME/Documents/Datasets/w2_sim/$(printf 'episode_%04d' "$EP")
mkdir -p "$OUT"

set +u                       # ROS2 setup.bash 引用未定义变量,与 set -u 冲突
source /opt/ros/jazzy/setup.bash
source $HOME/rtabmap_ws/install/setup.bash
set -u
# RMW:统一默认 FastDDS(本机 jazzy 无 cyclonedds),不要 export RMW_IMPLEMENTATION
: "${ISAACSIM_DIR:?需要 export ISAACSIM_DIR}"

cleanup() { kill $(jobs -p) 2>/dev/null || true; }
trap cleanup EXIT

ros2 run w2_sim degradation_node --ros-args \
  -p seed:="$SEED" -p params_out:="$OUT/episode_params.yaml" &

"$ISAACSIM_DIR/python.sh" "$PKG/isaac/w2_sim_app.py" \
  --seed "$SEED" --duration "$DUR" --env "$ENV" &
SIM_PID=$!

echo "等待仿真话题上线..."
timeout 300 bash -c \
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
