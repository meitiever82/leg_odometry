#!/usr/bin/env bash
# 逐个(顺序,非批量)生成 w2_sim 训练集 episode,自愈丢弃加载失败的场景。
# 用法: gen_training_set.sh [目标好episode数,默认40] [起始EP,默认200] [时长,默认120]
# 产出: ~/Documents/Datasets/w2_sim/episode_0NNN/ + TRAINING_GEN_LOG.txt(汇总,供次日查看)
set -u
TARGET=${1:-40}
EP=${2:-200}
DUR=${3:-120}
PKG="$(cd "$(dirname "$0")/.." && pwd)"
DATA=$HOME/Documents/Datasets/w2_sim
LOG=$DATA/TRAINING_GEN_LOG.txt
export ISAACSIM_DIR=$HOME/Documents/GitHub/system/IsaacSim/_build/linux-aarch64/release

# 候选场景:轻场景(gridroom/warehouse 已验证 + 其余 Grid)+ 重场景(office/hospital/
# digital_twin)。重场景加载期 MDL 材质编译很久,但 Omniverse 编译后缓存——所以每个场景
# 第一次给超长超时(WARMUP_TO,让 MDL 编译完+缓存),之后用正常超时(NORMAL_TO,缓存后快)。
# 第一次就超时 → 丢弃;预热成功但之后仍超时(缓存没帮上)→ 也丢弃。
SCENES=(gridroom warehouse gridroom_black default_environment full_warehouse \
        office hospital digital_twin)
NORMAL_TO=1000      # 正常超时(s):光场景 + 已预热重场景
WARMUP_TO=5400      # 首次超时(s,90min):让重场景 MDL 编译完并缓存
declare -A BAD=()
declare -A WARMED=()

START=$(date +%s)
good=0; i=0
echo "===== 训练集逐个生成 开始 $(date) 目标${TARGET}个 =====" >> "$LOG"

while [ "$good" -lt "$TARGET" ]; do
  # 守卫:磁盘 <200GB 停;运行 >11h 停
  free=$(df --output=avail "$HOME" | tail -1 | tr -d ' ')
  [ "$free" -lt 209715200 ] && { echo "磁盘不足(<200GB)停止 $(date)" >> "$LOG"; break; }
  [ $(( $(date +%s) - START )) -gt 39600 ] && { echo "运行 11h 停止 $(date)" >> "$LOG"; break; }

  scene=${SCENES[$(( i % ${#SCENES[@]} ))]}
  i=$(( i + 1 ))
  # 该场景已被标记失败 → 跳过
  [ "${BAD[$scene]:-0}" = "1" ] && continue

  OUT=$DATA/$(printf 'episode_%04d' "$EP")
  rm -rf "$OUT"
  # 首次该场景给超长超时预热(让 MDL 编译完+缓存),已预热用正常超时
  if [ "${WARMED[$scene]:-0}" = "1" ]; then to=$NORMAL_TO; else to=$WARMUP_TO; fi
  t0=$(date +%s)
  timeout "$to" "$PKG/scripts/collect_episode.sh" "$EP" "$DUR" "$scene" > /tmp/gen_ep${EP}.log 2>&1
  rc=$?
  dt=$(( $(date +%s) - t0 ))

  if [ -f "$OUT/ground_truth_tum.txt" ] && [ -f "$OUT/rosbag2/metadata.yaml" ]; then
    # 场景健康度 + 传感器(用真值,不用易假阳性的"前N帧质心")
    res=$(python3 - "$OUT" "$scene" <<'PY' 2>/dev/null
import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs_py import point_cloud2 as pc2
out, scene = sys.argv[1], sys.argv[2]
t = np.loadtxt(out + "/ground_truth_tum.txt"); xy = t[:, 1:3]; z = t[:, 3]
path = float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))
zrange = float(z.max() - z.min())
import glob
kap = ""
for line in open(out + "/episode_params.yaml"):
    if "kappa_theory" in line: kap = line.split(":")[1].strip()[:9]
# 传感器
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=out + "/rosbag2", storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
fields = None; cam_ok = False; npc = 0
while r.has_next():
    n, d, _ = r.read_next()
    if n == "/rslidar_points" and fields is None:
        fields = [f.name for f in deserialize_message(d, PointCloud2).fields]
    elif n == "/camera/color/image_raw" and not cam_ok:
        a = np.frombuffer(deserialize_message(d, Image).data, dtype=np.uint8)
        cam_ok = int(a.max()) > 0
    if fields and cam_ok: break
six = fields == ["x", "y", "z", "intensity", "ring", "timestamp"]
ok = path > 2.0 and zrange < 1.0 and six and cam_ok
print(f"{'GOOD' if ok else 'BAD'}|path={path:.1f}m|z范围={zrange:.2f}|κ={kap}|雷达6字段={six}|相机={cam_ok}")
PY
)
    verdict=${res%%|*}
    WARMED[$scene]=1   # 成功产出 → 该场景已预热(MDL 已缓存),后续用正常超时
    if [ "$verdict" = "GOOD" ]; then
      good=$(( good + 1 ))
      warm=$([ "$dt" -gt "$NORMAL_TO" ] && echo " [预热]" || echo "")
      echo "[$good/$TARGET] EP$EP $scene ($((dt/60))min)$warm $res" >> "$LOG"
      EP=$(( EP + 1 ))
    else
      echo "[skip] EP$EP $scene 数据异常: $res(已删)" >> "$LOG"
      rm -rf "$OUT"; EP=$(( EP + 1 ))
    fi
  else
    # 未产出:首次(未预热)超时=该重场景连 MDL 编译都跑不完→丢弃;已预热仍超时=缓存没帮上→也丢弃
    echo "[场景失败] EP$EP $scene rc=$rc $((dt/60))min 未产出(预热=${WARMED[$scene]:-0}),丢弃该场景" >> "$LOG"
    rm -rf "$OUT"; BAD[$scene]=1
    # 若所有场景都失败,停
    allbad=1; for s in "${SCENES[@]}"; do [ "${BAD[$s]:-0}" = "1" ] || allbad=0; done
    [ "$allbad" = "1" ] && { echo "所有候选场景均失败,停止 $(date)" >> "$LOG"; break; }
  fi
  pkill -9 -f SimulationApp 2>/dev/null; pkill -9 -f lidar_pc2_publisher 2>/dev/null
  sleep 3
done

echo "===== 结束 $(date) 共生成 $good 个好 episode,磁盘剩 $(df -h "$HOME"|tail -1|awk '{print $4}') =====" >> "$LOG"
pkill -9 -f SimulationApp 2>/dev/null; pkill -9 -f lidar_pc2_publisher 2>/dev/null; pkill -9 -f degradation_node 2>/dev/null
