#!/usr/bin/env python3
# isaac/validate_scene.py — 逐场景 spawn/行驶/传感器健康检查(不录 bag,纯仿真内自检)。
# 用法:
#   $ISAACSIM_DIR/python.sh isaac/validate_scene.py --env office --duration 25
#
# 检查项(对应任务 §2):
#   * 机器人加载、spawn 不卡障碍:reset 后 z 稳定、不爆飞(|z|<5、|x,y|<box 边界外不远)。
#   * 能行驶 >1m:简单前进+小转弯命令,末端 odom 路径长度 > 1m。
#   * z 全程稳定:整段 z 的 std < 0.05m、min z > 0(不穿地)、max z < spawn_z+1(不弹飞)。
#   * 雷达出数据:IsaacCreateRTXLidarScanBuffer get_data() 点数 > 1000。
#   * 相机非黑:RGB 像素 max > 0。
# 复用主程序 w2_sim_app.py 的场景注册表与机器人/传感器装配逻辑,精简掉 ROS2 发布图、
# 退化节点、sidecar —— 只为验证「该场景机器人能正常工作」。
import argparse
import sys
from pathlib import Path

import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--env", required=True)
parser.add_argument("--duration", type=float, default=25.0)
parser.add_argument("--usd", default=str(Path.home() / "rtabmap_ws/w2/usd/w2_swerve.usd"))
parser.add_argument("--gui", action="store_true")
args = parser.parse_args()

# 从主程序拿场景注册表(单一真相源,改注册表只改一处)。
_app_dir = Path(__file__).resolve().parent
sys.path.insert(0, str(_app_dir))
import importlib.util  # noqa: E402

# 直接解析 w2_sim_app.py 的 SCENE_REGISTRY 字面量(不能 import 它——一 import 就建
# SimulationApp 并跑全程)。用 ast 抽取注册表 dict。
import ast  # noqa: E402

_src = (_app_dir / "w2_sim_app.py").read_text()
_tree = ast.parse(_src)
SCENE_REGISTRY = None
for _node in _tree.body:
    if isinstance(_node, ast.Assign):
        for _t in _node.targets:
            if isinstance(_t, ast.Name) and _t.id == "SCENE_REGISTRY":
                SCENE_REGISTRY = ast.literal_eval(_node.value)
assert SCENE_REGISTRY is not None, "未能从 w2_sim_app.py 解析 SCENE_REGISTRY"
assert args.env in SCENE_REGISTRY, f"未知 env={args.env}, 可选: {sorted(SCENE_REGISTRY)}"
scene = SCENE_REGISTRY[args.env]
SPAWN_XYZ = list(scene["spawn"])
box = list(scene["box"])

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": not args.gui})

import omni.graph.core as og  # noqa: E402  (only to keep parity; unused)
import omni.kit.commands  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from isaacsim.core.utils.xforms import reset_and_set_xform_ops  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402

enable_extension("isaacsim.sensors.rtx")
app.update()

sys.path.insert(0, str(_app_dir.parent))  # 仓库 w2_sim/ 目录,内含 w2_sim/ 包
from w2_sim.swerve_kinematics import WHEEL_NAMES, WHEEL_RADIUS, inverse_kinematics  # noqa: E402

PHYS_HZ, RENDER_HZ = 200.0, 10.0
world = World(stage_units_in_meters=1.0,
              physics_dt=1.0 / PHYS_HZ, rendering_dt=1.0 / RENDER_HZ)

# ---- 场景 ----
if scene["usd"] is None:
    world.scene.add_default_ground_plane()
    print(f"SCENE: env={args.env} default_ground_plane spawn={SPAWN_XYZ} box={box}", flush=True)
else:
    assets = get_assets_root_path()
    assert assets, "云资产不可达"
    add_reference_to_stage(usd_path=assets + scene["usd"], prim_path="/World/env")
    print(f"SCENE: env={args.env} usd={scene['usd']} spawn={SPAWN_XYZ} box={box}", flush=True)

# ---- 机器人 ----
add_reference_to_stage(usd_path=args.usd, prim_path="/World/w2")
stage = get_current_stage()
root_candidates = [
    str(p.GetPath()) for p in stage.Traverse()
    if p.HasAPI(UsdPhysics.ArticulationRootAPI) and str(p.GetPath()).startswith("/World/w2")
]
assert len(root_candidates) == 1, f"ArticulationRoot 候选: {root_candidates}"
ROBOT_ROOT = root_candidates[0]
LINK_PARENT = ROBOT_ROOT.rsplit("/", 1)[0]
RSLIDAR_LINK = LINK_PARENT + "/rslidar"
CAMERA_LINK = LINK_PARENT + "/camera_link"

robot = SingleArticulation(ROBOT_ROOT, name="w2", position=np.array(SPAWN_XYZ, dtype=float))
world.scene.add(robot)

# ---- RTX Lidar(复用 AIRY USD,CopySpec 路线)----
AIRY_USD = str(_app_dir.parent / "config" / "RS_Airy.usd")
assert Path(AIRY_USD).exists(), f"AIRY USD 不存在: {AIRY_USD}"
LIDAR_PRIM_PATH = RSLIDAR_LINK + "/rtx_lidar"
_src_stage = Usd.Stage.Open(AIRY_USD)
_src_default = _src_stage.GetDefaultPrim()
_dst_layer = get_current_stage().GetRootLayer()
Sdf.CreatePrimInLayer(_dst_layer, LIDAR_PRIM_PATH)
Sdf.CopySpec(_src_stage.GetRootLayer(), _src_default.GetPath(), _dst_layer, LIDAR_PRIM_PATH)
lidar_prim = get_current_stage().GetPrimAtPath(LIDAR_PRIM_PATH)
reset_and_set_xform_ops(lidar_prim, Gf.Vec3d(0, 0, 0), Gf.Quatd(1, 0, 0, 0))
lidar_rp = rep.create.render_product(lidar_prim.GetPath(), [1, 1])
_lidar_annot = rep.AnnotatorRegistry.get_annotator("IsaacCreateRTXLidarScanBuffer")
_lidar_annot.initialize(outputIntensity=True, transformPoints=False)
_lidar_annot.attach([lidar_rp.path])

# ---- 前视相机 ----
cam = UsdGeom.Camera.Define(get_current_stage(), CAMERA_LINK + "/front_camera")
cam.GetFocalLengthAttr().Set(18.0)
cam.AddOrientOp().Set(Gf.Quatf(0.5, Gf.Vec3f(0.5, -0.5, -0.5)))
cam_rp = rep.create.render_product(CAMERA_LINK + "/front_camera", (320, 240))
_rgb_annot = rep.AnnotatorRegistry.get_annotator("LdrColor")
_rgb_annot.attach([cam_rp.path])

world.reset()
robot.initialize()
dof = {n: i for i, n in enumerate(robot.dof_names)}
steer_idx = np.array([dof[f"{w}_steer_joint"] for w in WHEEL_NAMES])
wheel_idx = np.array([dof[f"{w}_wheel_joint"] for w in WHEEL_NAMES])
ndof = len(robot.dof_names)
kps = np.zeros(ndof)
kds = np.zeros(ndof)
for i in steer_idx:
    kps[i], kds[i] = 1e4, 1e3
for i in wheel_idx:
    kps[i], kds[i] = 0.0, 120.0
robot.get_articulation_controller().set_gains(kps=kps, kds=kds)

# 让机器人 settle 几帧再读 spawn z(reset 后第一帧可能还在下落)
for _ in range(5):
    world.step(render=True)
spawn_pos, _ = robot.get_world_pose()
print(f"SPAWN_SETTLED: x={spawn_pos[0]:.3f} y={spawn_pos[1]:.3f} z={spawn_pos[2]:.3f}", flush=True)

# 简单巡航命令:慢速前进 + 缓和转弯(画弧),走出 >1m 且测多方向。
prev_angles = np.zeros(4)
t0 = world.current_time
zs = []
path = []
last_xy = None
plen = 0.0
n_lidar_ok = 0
lidar_pts_first = 0
cam_max = 0
step_i = 0
while world.current_time - t0 < args.duration:
    world.step(render=True)
    step_i += 1
    pos, quat = robot.get_world_pose()
    zs.append(float(pos[2]))
    if last_xy is not None:
        plen += float(np.hypot(pos[0] - last_xy[0], pos[1] - last_xy[1]))
    last_xy = (float(pos[0]), float(pos[1]))

    # lidar
    d = _lidar_annot.get_data()
    if isinstance(d, dict) and d.get("data") is not None:
        npts = int(np.asarray(d["data"]).shape[0])
        if npts > 1000:
            n_lidar_ok += 1
            if lidar_pts_first == 0:
                lidar_pts_first = npts
    # camera every 10 frames
    if step_i % 10 == 0:
        rgb = _rgb_annot.get_data()
        if rgb is not None and np.asarray(rgb).size > 0:
            cam_max = max(cam_max, int(np.asarray(rgb).max()))

    # 命令:前 60% 时间直行 vx=0.5,后 40% 弧线 vx=0.5 + wz=0.4
    tnorm = (world.current_time - t0) / args.duration
    if tnorm < 0.6:
        cmd = (0.5, 0.0, 0.0)
    else:
        cmd = (0.5, 0.0, 0.4)
    # safe box 回驶
    if not (box[0] < pos[0] < box[1] and box[2] < pos[1] < box[3]):
        yaw = np.arctan2(2 * (quat[0] * quat[3] + quat[1] * quat[2]),
                         1 - 2 * (quat[2] ** 2 + quat[3] ** 2))
        ctr = np.array([(box[0] + box[1]) * 0.5, (box[2] + box[3]) * 0.5])
        delta = ctr - pos[:2]
        dvec = delta / max(np.linalg.norm(delta), 1e-6) * 0.5
        c, s = np.cos(-yaw), np.sin(-yaw)
        cmd = (c * dvec[0] - s * dvec[1], s * dvec[0] + c * dvec[1], 0.0)
    ang, spd = inverse_kinematics(cmd[0], cmd[1], cmd[2], prev_angles=prev_angles)
    prev_angles = ang
    robot.apply_action(ArticulationAction(joint_positions=ang, joint_indices=steer_idx))
    robot.apply_action(ArticulationAction(joint_velocities=spd / WHEEL_RADIUS,
                                          joint_indices=wheel_idx))

zs = np.array(zs)
z_std = float(zs.std())
z_min = float(zs.min())
z_max = float(zs.max())
n_steps = len(zs)
lidar_frac = n_lidar_ok / max(n_steps, 1)

# 判定。
# spawn_z:机器人 base_footprint 落在地面后,静止高度取决于轮/base 几何;不同场景
#   地面 z 不同(多在 0 附近)。合理区间 [-0.1, spawn_z+1)——下界容许 base 原点贴地
#   (warehouse/ground 实测 settle 到 ~0.00±0.01),上界防悬空未落/弹飞。|z|>5 必爆飞。
ok_spawn = -0.1 < spawn_pos[2] < SPAWN_XYZ[2] + 1.0 and abs(spawn_pos[2]) < 5.0
ok_zstab = z_std < 0.05 and z_min > -0.3 and z_max < SPAWN_XYZ[2] + 1.0
ok_drive = plen > 1.0
# lidar:有结构场景应满圈(~84k pts)。ground/无结构场景 AIRY 半球朝上、平地无回波
#   → 0 点属预期,不据此判废(单独看 lidar_note)。
ok_lidar = lidar_frac > 0.5 and lidar_pts_first > 1000
ok_cam = cam_max > 0
lidar_note = "structured-ok" if ok_lidar else (
    "no-structure(expected on flat ground)" if args.env == "ground" else "NO-LIDAR-DATA")
# 核心判废项:spawn 不卡障碍 + z 稳定 + 能行驶 + 相机出图。lidar 单列(ground 豁免)。
core_ok = ok_spawn and ok_zstab and ok_drive and ok_cam
verdict = core_ok and (ok_lidar or args.env == "ground")

print("==== SCENE_VALIDATION_RESULT ====", flush=True)
print(f"  env={args.env}", flush=True)
print(f"  spawn_z={spawn_pos[2]:.3f}  [{'OK' if ok_spawn else 'FAIL'}]", flush=True)
print(f"  z: std={z_std:.4f} min={z_min:.3f} max={z_max:.3f}  [{'OK' if ok_zstab else 'FAIL'}]",
      flush=True)
print(f"  path_len={plen:.2f}m  [{'OK' if ok_drive else 'FAIL'}]", flush=True)
print(f"  lidar: pts={lidar_pts_first} ok_frac={lidar_frac:.2f} "
      f"[{'OK' if ok_lidar else 'FAIL'}] ({lidar_note})", flush=True)
print(f"  camera_max={cam_max}  [{'OK' if ok_cam else 'FAIL'}]", flush=True)
print(f"  VERDICT: {'PASS' if verdict else 'FAIL'}", flush=True)

try:
    _lidar_annot.detach()
    _rgb_annot.detach()
except Exception:
    pass
app.close()
sys.exit(0 if verdict else 1)
