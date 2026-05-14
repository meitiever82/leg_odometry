#!/usr/bin/env bash
# Run wheel_only_node against a recorded bag and produce a CSV trajectory
# plus a quick path / closure-error printout.
#
# The 2026-05-14 bag has only /robot/wheel_status (sensor_msgs/JointState),
# no IMU and no LIO ground truth, so the script just sanity-checks that the
# pipeline produces a coherent path. Override env vars to point elsewhere.

set -eo pipefail

BAG_DIR="${BAG_DIR:-$HOME/Documents/Datasets/w2/rosbag2_2026_05_14-10_25_09}"
OUT_DIR="${OUT_DIR:-/tmp/wheel_odom_test}"
WHEELBASE="${WHEELBASE:-0.435}"
TRACK="${TRACK:-0.400}"
WHEEL_RADIUS="${WHEEL_RADIUS:-1.0}"
YAW_SOURCE="${YAW_SOURCE:-ls}"
ENABLE_IMU="${ENABLE_IMU:-false}"
RATE="${RATE:-5}"

mkdir -p "$OUT_DIR"
cd "$HOME/rtabmap_ws"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source install/setup.bash

pkill -9 -f wheel_only_node 2>/dev/null || true
pkill -9 -f "ros2 bag play" 2>/dev/null || true
sleep 1
rm -f "$OUT_DIR"/{run.csv,node.log,bag.log}

echo ">> launching wheel_only_node (L=$WHEELBASE W=$TRACK r=$WHEEL_RADIUS yaw=$YAW_SOURCE imu=$ENABLE_IMU)"
ros2 run wheel_odometry wheel_only_node --ros-args \
    -p wheelbase:="$WHEELBASE" \
    -p track:="$TRACK" \
    -p wheel_radius:="$WHEEL_RADIUS" \
    -p yaw_source:="$YAW_SOURCE" \
    -p enable_imu:="$ENABLE_IMU" \
    -p chassis_topic:=/robot/wheel_status \
    -p odom_topic:=/wheel_odometry \
    -p diag_csv_path:="$OUT_DIR/run.csv" \
    > "$OUT_DIR/node.log" 2>&1 &
NODE_PID=$!
sleep 2

echo ">> playing bag at ${RATE}x"
ros2 bag play "$BAG_DIR" --rate "$RATE" > "$OUT_DIR/bag.log" 2>&1
echo ">> bag done"

kill -9 "$NODE_PID" 2>/dev/null || true
wait 2>/dev/null || true

echo ">> stats:"
python3 - "$OUT_DIR/run.csv" <<'PY'
import csv, math, sys
xs, ys, residuals, wz = [], [], [], []
with open(sys.argv[1]) as f:
    r = csv.reader(f); next(r)
    for row in r:
        try:
            xs.append(float(row[1])); ys.append(float(row[2]))
            wz.append(float(row[13]))
            residuals.append(float(row[14]))
        except: pass
n = len(xs)
if n < 2:
    print(f"   only {n} rows — bag likely empty or node didn't subscribe in time"); sys.exit(0)
path = sum(math.hypot(xs[i+1]-xs[i], ys[i+1]-ys[i]) for i in range(n-1))
disp = math.hypot(xs[-1]-xs[0], ys[-1]-ys[0])
print(f"   rows={n}  path={path:.2f}m  disp={disp:.2f}m  "
      f"end=({xs[-1]:+.2f},{ys[-1]:+.2f})  "
      f"closure_rel={disp/path*100 if path > 1e-3 else float('nan'):.2f}%")
print(f"   ls_residual: min={min(residuals):.4f} max={max(residuals):.4f} "
      f"mean={sum(residuals)/n:.4f}")
print(f"   wz_used:     min={min(wz):.4f} max={max(wz):.4f} mean={sum(wz)/n:.4f} rad/s")
PY

echo ">> done. CSV: $OUT_DIR/run.csv"
