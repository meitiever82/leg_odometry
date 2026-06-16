#!/usr/bin/env python3
# isaac/w2_sim_app.py — w2 数据采集仿真主程序。
# 用法:
#   source /opt/ros/jazzy/setup.bash
#   $ISAACSIM_DIR/python.sh isaac/w2_sim_app.py --seed 1 --duration 60 \
#       --usd ~/rtabmap_ws/w2/usd/w2_swerve.usd [--env warehouse|ground] [--gui]
#
# 设计要点(预览版 5.1.0-rc.19 实测适配,详见任务报告):
#   * articulation root 不在 /World/w2,而在其子 prim(importer 把
#     UsdPhysics.ArticulationRootAPI 挂到 base_footprint)——运行时扫 stage 找真实
#     root path,用于 SingleArticulation / PubJS.targetPrim / Odom.chassisPrim。
#   * 物理步发布图必须建成 ONDEMAND pipeline_stage,否则 OnPhysicsStep 不触发
#     ("Physics OnSimulationStep node detected in a non on-demand Graph")。
#   * 每个 ROS2 发布节点都接 ROS2Context(沿用 dog-m1 先例写法)。
#   * lidar 用 ROS2RtxLidarHelper og 节点(先例路线),render product + RunOneSimFrame 门控。
#   * IMU 用 isaacsim.sensors.physics.IsaacReadIMU og 节点(本构建有该节点,
#     输出 angVel / linAcc / orientation)。
#   * 所有 print 必须 flush=True(app.close() 会吞缓冲)。
import argparse
import os
import sys
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--seed", type=int, required=True)
parser.add_argument("--duration", type=float, required=True)
parser.add_argument("--usd", default=str(Path.home() / "rtabmap_ws/w2/usd/w2_swerve.usd"))
parser.add_argument("--env", default="warehouse", choices=["warehouse", "ground"])
parser.add_argument("--gui", action="store_true")
parser.add_argument("--safe-box", type=float, nargs=4, default=[-4.0, 4.0, -6.0, 6.0],
                    help="xmin xmax ymin ymax,出界即朝中心回驶")
args = parser.parse_args()

from isaacsim import SimulationApp

app = SimulationApp({"headless": not args.gui})

import numpy as np  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.kit.commands  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from isaacsim.core.utils.xforms import reset_and_set_xform_ops  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade  # noqa: E402

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.sensors.physics")
enable_extension("isaacsim.sensors.rtx")
app.update()

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from w2_sim.swerve_kinematics import WHEEL_NAMES, WHEEL_RADIUS, inverse_kinematics  # noqa: E402
from w2_sim.trajectory_generator import generate_profile  # noqa: E402

PHYS_HZ, CMD_HZ = 200.0, 50.0
# RENDER_HZ 必须 = AIRY 旋转率(10Hz)—— 这是雷达点云在"渲染欠载"下稳定的关键。
# 原值 30Hz 时,RTX 雷达每圈(0.1s)需累积 3 个渲染帧(每帧 ~120° 方位),由
# ROS2RtxLidarHelper(fullScan=True)内部的 IsaacCreateRTXLidarScanBuffer 拼成 360°。
# 渲染欠载时(场景/视角让 RTX 吃力,或 GPU/CPU 被抢占),某些渲染帧的射线追踪结果
# 来不及刷新 → 累积器拿到陈旧/错位的方位扇区 → 拼出的"整圈"实为 121°~360° 漂移覆盖
# (静止时点云质心在多个离散态间循环跳变,实测 ep16 帧间跳动 mean 6.86m/89.9% 帧 >1m)。
# 把 rendering_dt 设为整圈周期(1/10s)后,每条雷达消息恰由**单个渲染帧**产生整圈,
# 不再跨帧拼接,故对渲染欠载结构性免疫(实测:重度 CPU+GPU 压力下静止段质心跳动
# 仍 mean 0.012m / 0% 帧 >1m,等同正常场景)。代价:相机/camera_info 也降到 10Hz
# (与雷达共享渲染 tick);物理率话题(imu 200Hz / odom 100Hz / joint_states 50Hz)
# 走独立 ONDEMAND 物理图,不受影响。10Hz RGB 对视觉闭环足够。
RENDER_HZ = 10.0
CAMERA_FOCAL_LENGTH_MM = 18.0  # ~50° HFoV(aperture 默认 20.955mm),近似前视相机

world = World(stage_units_in_meters=1.0,
              physics_dt=1.0 / PHYS_HZ, rendering_dt=1.0 / RENDER_HZ)

# ---- 场景 ----
GROUND_PRIMS = []  # 用于绑高摩擦材质的地面 prim path(随场景而定)
if args.env == "warehouse":
    assets = get_assets_root_path()
    if assets:
        add_reference_to_stage(
            usd_path=assets + "/Isaac/Environments/Simple_Warehouse/warehouse.usd",
            prim_path="/World/env")
        GROUND_PRIMS.append("/World/env")
    else:
        print("WARN: 云资产不可达,退回平地", file=sys.stderr, flush=True)
        world.scene.add_default_ground_plane()
        GROUND_PRIMS.append("/World/defaultGroundPlane")
else:
    world.scene.add_default_ground_plane()
    GROUND_PRIMS.append("/World/defaultGroundPlane")

# ---- 机器人 ----
add_reference_to_stage(usd_path=args.usd, prim_path="/World/w2")

# articulation root 自动发现:importer 把 ArticulationRootAPI 挂在 /World/w2 的子 prim,
# 不是 /World/w2 本身。扫 stage 找真实 root path(同 verify_usd.py 逻辑)。
stage = get_current_stage()
root_candidates = [
    str(p.GetPath())
    for p in stage.Traverse()
    if p.HasAPI(UsdPhysics.ArticulationRootAPI) and str(p.GetPath()).startswith("/World/w2")
]
print("ARTICULATION_ROOT_CANDIDATES:", root_candidates, flush=True)
assert root_candidates, "stage 里没有带 ArticulationRootAPI 的 prim(USD 缺 articulation root)"
assert len(root_candidates) == 1, \
    f"发现多个 ArticulationRootAPI: {root_candidates},预期只有 1 个"
ROBOT_ROOT = root_candidates[0]
print("ROBOT_ROOT:", ROBOT_ROOT, flush=True)

# importer 把所有 link 拍平到 articulation root 的父 prim 下,例如
# ROBOT_ROOT = /World/w2/base_footprint/base_footprint
# 则 imu_link / camera_link / rslidar 是它的兄弟:/World/w2/base_footprint/<link>。
# 用 ROBOT_ROOT 的父 prim 作为传感器挂载父路径(不要硬编码 /World/w2/<link>)。
LINK_PARENT = ROBOT_ROOT.rsplit("/", 1)[0]
IMU_LINK = LINK_PARENT + "/imu_link"
CAMERA_LINK = LINK_PARENT + "/camera_link"
RSLIDAR_LINK = LINK_PARENT + "/rslidar"
print("LINK_PARENT:", LINK_PARENT, flush=True)

robot = SingleArticulation(ROBOT_ROOT, name="w2",
                           position=np.array([0.0, 0.0, 0.3]))
world.scene.add(robot)

# ===========================================================================
# 压低 wheel-LS ω_z 抖(Task 26,2026-06-16)。baseline RMS(wheel-LS ω_z vs 真值)=0.286,
# 真机 0.050。诊断(scripts/diag_slip.py + 分频)定位真因:**不是 contact slip**——真值 ω_z
# 本身光滑(高频 RMS 0.013)、轮子不纵向打滑(mu≥1 时 speed_ratio=1.000),抖全在 achieved
# 轮速读数(高频 0.18,经 8×3 LS b=v·[cosθ,sinθ] 放大成 ω_z)。真正的旋钮是**轮速 velocity
# drive 的阻尼 kd**(见下方 set_gains):kd 过高 velocity 环对接触力扰动过敏 → 读数振铃。
# 这里的接触项(摩擦/solver/contactOffset)只做两件必需的事,对该抖几乎无影响:
#   1. 高摩擦 PhysicsMaterial(static/dynamic=1.5)绑 4 轮 + 地面,combineMode=max
#      —— **唯一必需**:mu 默认 ~0.5/0.01 时轮子真纵向打滑(speed_ratio 1.8),mu=1.5 归 1.0。
#   2. solver 迭代 pos32/vel4、轮碰撞 contactOffset 2cm —— 稳接触,二阶项(实测对 RMS 影响小,
#      过高 solver 反而更差;保守值)。
# 不破坏 swerve:高摩擦只防纵向打滑,转向后仍纯滚动。
# ===========================================================================
_TUNE_OFF = os.environ.get("W2_TUNE_OFF", "0") == "1"  # A/B 对照:跳过全部接触调优
WHEEL_FRICTION = float(os.environ.get("W2_WHEEL_FRICTION", "1.5"))
SOLVER_POS_ITER = int(os.environ.get("W2_SOLVER_POS_ITER", "32"))
SOLVER_VEL_ITER = int(os.environ.get("W2_SOLVER_VEL_ITER", "4"))


def _make_friction_material(path, mu_s, mu_d):
    """建一个高摩擦 PhysicsMaterial,combineMode=max(取轮/地较大者)。返回 prim path。"""
    mat = UsdShade.Material.Define(stage, path)
    mp = mat.GetPrim()
    UsdPhysics.MaterialAPI.Apply(mp)
    mapi = UsdPhysics.MaterialAPI(mp)
    mapi.CreateStaticFrictionAttr().Set(mu_s)
    mapi.CreateDynamicFrictionAttr().Set(mu_d)
    mapi.CreateRestitutionAttr().Set(0.0)
    pmat = PhysxSchema.PhysxMaterialAPI.Apply(mp)
    pmat.CreateFrictionCombineModeAttr().Set("max")
    pmat.CreateRestitutionCombineModeAttr().Set("min")
    return path


def _bind_physics_material(prim_path, mat_path):
    """把 physics material 绑到 prim(及其碰撞子树)。"""
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        print(f"WARN friction: prim 不存在 {prim_path}", file=sys.stderr, flush=True)
        return False
    mat = UsdShade.Material.Get(stage, mat_path)
    UsdShade.MaterialBindingAPI.Apply(prim)
    UsdShade.MaterialBindingAPI(prim).Bind(
        mat, UsdShade.Tokens.weakerThanDescendants, "physics")
    return True


if _TUNE_OFF:
    print("CONTACT_TUNE: OFF (W2_TUNE_OFF=1) — 用 PhysX 默认接触做 A/B 对照", flush=True)
else:
    _WHEEL_MAT = _make_friction_material(
        "/World/PhysicsMaterials/wheel_hi_fric", WHEEL_FRICTION, WHEEL_FRICTION)
    _GROUND_MAT = _make_friction_material(
        "/World/PhysicsMaterials/ground_hi_fric", WHEEL_FRICTION, WHEEL_FRICTION)

    # 绑 4 轮碰撞体(cylinder 或 capsule,取决于 import 的 W2_WHEEL_CAPSULE)+ 调接触 offset。
    # 碰撞子 prim 名随几何类型变,按带 CollisionAPI 的子 prim 找,不硬编码名。
    _WHEEL_LINKS = ["front_left", "front_right", "rear_left", "rear_right"]
    _n_wheel_bound = 0
    for _w in _WHEEL_LINKS:
        _link = stage.GetPrimAtPath(f"/World/w2/{_w}_wheel_link")
        if not _link or not _link.IsValid():
            print(f"WARN friction: wheel link 不存在 {_w}", file=sys.stderr, flush=True)
            continue
        for _cp in Usd.PrimRange(_link):
            if _cp.HasAPI(UsdPhysics.CollisionAPI):
                _cpath = str(_cp.GetPath())
                if _bind_physics_material(_cpath, "/World/PhysicsMaterials/wheel_hi_fric"):
                    _n_wheel_bound += 1
                    _capi = PhysxSchema.PhysxCollisionAPI.Apply(_cp)
                    _capi.CreateContactOffsetAttr().Set(0.02)
                    _capi.CreateRestOffsetAttr().Set(0.0)
    print(f"WHEEL_FRICTION_BOUND: {_n_wheel_bound}/4 collisions mu={WHEEL_FRICTION}",
          flush=True)
    assert _n_wheel_bound == 4, f"轮摩擦材质只绑上 {_n_wheel_bound}/4"

    # 绑地面(warehouse 整子树 / defaultGroundPlane)
    for _g in GROUND_PRIMS:
        _bind_physics_material(_g, "/World/PhysicsMaterials/ground_hi_fric")
    print(f"GROUND_FRICTION_BOUND: {GROUND_PRIMS}", flush=True)

    # 提高 articulation solver 迭代(更硬接触,减滑移/穿透)
    _root_prim = stage.GetPrimAtPath(ROBOT_ROOT)
    _pxapi = PhysxSchema.PhysxArticulationAPI.Apply(_root_prim)
    _pxapi.CreateSolverPositionIterationCountAttr().Set(SOLVER_POS_ITER)
    _pxapi.CreateSolverVelocityIterationCountAttr().Set(SOLVER_VEL_ITER)
    print(f"SOLVER_ITER: pos={SOLVER_POS_ITER} vel={SOLVER_VEL_ITER}", flush=True)

# ===========================================================================
# steer 关节驱动 maxForce 解锁(Task 26,2026-06-16):importer 把 steer revolute 的
# drive:angular:physics:maxForce 设成 50(acceleration 模式),转向力矩被硬限,大角度
# 转向瞬态时 achieved 角会短暂追不上 IK 命令角。把 4 个 steer drive 的 maxForce/stiffness
# 调高、贴紧命令角,收窄转向瞬态窗口(转向瞬态是 wheel-LS ω_z 残余抖的次要来源,主因是
# 轮速环阻尼,见 set_gains 处)。必须在 world.reset() 前改 USD 属性。
STEER_MAX_FORCE = float(os.environ.get("W2_STEER_MAXFORCE", "5000.0"))
STEER_STIFFNESS = float(os.environ.get("W2_STEER_STIFFNESS", "100000.0"))
STEER_DAMPING = float(os.environ.get("W2_STEER_DAMPING", "3000.0"))
_n_steer = 0
for _p in stage.Traverse():
    _pp = str(_p.GetPath())
    if "steer_joint" in _pp and _p.GetTypeName() == "PhysicsRevoluteJoint":
        _drv = UsdPhysics.DriveAPI.Get(_p, "angular")
        if not _drv:
            _drv = UsdPhysics.DriveAPI.Apply(_p, "angular")
        _drv.CreateMaxForceAttr().Set(STEER_MAX_FORCE)
        _drv.CreateStiffnessAttr().Set(STEER_STIFFNESS)
        _drv.CreateDampingAttr().Set(STEER_DAMPING)
        _n_steer += 1
print(f"STEER_DRIVE_UNLOCK: {_n_steer} joints maxForce={STEER_MAX_FORCE} "
      f"stiffness={STEER_STIFFNESS} damping={STEER_DAMPING}", flush=True)

# ---- IMU 传感器(挂 imu_link)----
# 注:父 prim 必须带 RigidBodyAPI;imu_link 是 rigid body(实测),用拍平后的真实路径。
IMU_SENSOR_PATH = IMU_LINK + "/imu_sensor"
omni.kit.commands.execute(
    "IsaacSensorCreateImuSensor",
    path="imu_sensor", parent=IMU_LINK,
    translation=Gf.Vec3d(0, 0, 0), orientation=Gf.Quatd(1, 0, 0, 0))

# ---- RTX Lidar(挂 rslidar):忠实仿真 RoboSense AIRY ----
# Isaac Sim 5.x 的 RTX lidar 不再用本地 JSON config:IsaacSensorCreateRtxLidar 的
# config= 只解析 SUPPORTED_LIDAR_CONFIGS 里的资产服务器 USD 名(见
# exts/isaacsim.sensors.rtx/.../impl/{commands,supported_lidar_configs}.py)。真实参数
# 存在 OmniLidar prim 的 `omni:sensor:Core:*` 属性里(旧 JSON schema 已迁进 USD)。
#
# 关键实测教训:lidar 插件在 **sensor 创建时** 解析并缓存这些 Core 配置;创建后再覆写
# prim 属性插件不重读 —— 用 OS1 模板创建再覆写成 96 线 / 0~90° 后,prim 属性确实变了
# (probe 读回 96 全对),但渲染时插件仍按模板 32 线缓存,报
# "NumberOfEmitters 96 != param vector length 32" + "Unknown scan type 0" 并崩溃。
# 因此正确做法:离线把 OS1 模板覆写成 AIRY 后导出 flatten 的本地 USD
# (isaac/make_airy_usd.py → config/RS_Airy.usd,AIRY 值是 concrete spec),
# 主程序用 add_reference_to_stage **引用**这个 USD —— 插件创建时直接解析到 AIRY。
# (config= 传本地 USD/JSON 路径都不被 IsaacSensorCreateRtxLidar 接受,只认资产服务器名,
#  故绕开该命令,直接 add_reference_to_stage 引用本地 USD,与命令内部 _add_reference 同理。)
#
# RoboSense AIRY 规格(手册表1 + 真实 bag 实测):
#   96 线,垂直 FOV 0~90°(0.947° 步,均布,半球穹顶,沿传感器 +Z),
#   水平 360° @ 0.4° 分辨率(900 方位步),10Hz 旋转式,
#   测距 0.1~60m,距离精度 1.5cm(1σ 高斯),强度 1~255,单回波 strongest。
#   出点 96×900 = 86,400 pts/帧 ≈ 手册 856,320 pts/s。
#   若 config/RS_Airy.usd 不存在,先跑:
#     $ISAACSIM_DIR/python.sh isaac/make_airy_usd.py
AIRY_USD = str(Path(__file__).resolve().parent.parent / "config" / "RS_Airy.usd")
LIDAR_CONFIG = AIRY_USD
assert Path(AIRY_USD).exists(), \
    f"AIRY USD 不存在: {AIRY_USD}(先跑 isaac/make_airy_usd.py 生成)"

# 关键实测教训:lidar 插件解析 emitter 配置时,**不读 stage 上经 reference/override
# 合成后的属性,而是顺着 prim 的 reference 去读原始资产文件**。所以无论
#   (a) 用 OS1 模板就地覆写(插件读 payload ./OS1_*.usda → 32 线),
#   (b) 用 Example_Rotary 模板就地覆写(插件读 reference Example_Rotary.usda → 128 线),
#   (c) add_reference_to_stage 引用 flatten 好的 RS_Airy.usd(插件仍读到 128 / scanType 0),
# prim 属性读回都对(96),但插件一律报 "NumberOfEmitters 96 != 32/128" + "scanType 0" 崩溃。
# 解决:用 Sdf.CopySpec 把 flatten 好的 RS_Airy.usd 的 OmniLidar prim spec **物理拷贝**
# 进本 stage 的目标路径(无任何 reference/payload,全是 concrete spec),插件顺着 prim
# 找不到外部资产,只能读 prim 自身 concrete 的 96 线 AIRY 配置。
LIDAR_PRIM_PATH = RSLIDAR_LINK + "/rtx_lidar"
_src_stage = Usd.Stage.Open(AIRY_USD)
_src_default = _src_stage.GetDefaultPrim()
assert _src_default and _src_default.GetTypeName() == "OmniLidar", \
    f"RS_Airy.usd defaultPrim 不是 OmniLidar(got {_src_default.GetTypeName() if _src_default else None})"
_src_layer = _src_stage.GetRootLayer()
_dst_layer = get_current_stage().GetRootLayer()
# 目标父 prim 必须先存在;CopySpec 会建目标 prim spec。
Sdf.CreatePrimInLayer(_dst_layer, LIDAR_PRIM_PATH)
ok_copy = Sdf.CopySpec(_src_layer, _src_default.GetPath(),
                       _dst_layer, LIDAR_PRIM_PATH)
assert ok_copy, "Sdf.CopySpec 拷贝 AIRY OmniLidar spec 失败"
lidar_prim = get_current_stage().GetPrimAtPath(LIDAR_PRIM_PATH)
assert lidar_prim.IsValid() and lidar_prim.GetTypeName() == "OmniLidar", \
    f"CopySpec 后 {LIDAR_PRIM_PATH} 不是 OmniLidar(type={lidar_prim.GetTypeName()})"
# 朝向:相对 rslidar link 为单位(rslidar link 已在 urdf 里竖直安装,AIRY 半球朝上)。
reset_and_set_xform_ops(lidar_prim, Gf.Vec3d(0, 0, 0), Gf.Quatd(1, 0, 0, 0))
print(f"AIRY_LIDAR: copyspec<-{AIRY_USD} prim={lidar_prim.GetPath()} "
      f"scanType={lidar_prim.GetAttribute('omni:sensor:Core:scanType').Get()} "
      f"nEmit={lidar_prim.GetAttribute('omni:sensor:Core:numberOfEmitters').Get()} "
      f"elN={len(lidar_prim.GetAttribute('omni:sensor:Core:emitterState:s001:elevationDeg').Get())}",
      flush=True)

lidar_rp = rep.create.render_product(lidar_prim.GetPath(), [1, 1])

# ---- RTX Lidar intensity annotator(替代 ROS2RtxLidarHelper 的 point_cloud 发布)----
# 本构建的 ROS2RtxLidarHelper(type=point_cloud)及其底层 ROS2PublishPointCloud og 节点
# 只发 XYZ(point3f),没有 intensity 字段。真实 AIRY 的 /rslidar_points 带 intensity
# (反射率 1~255)。为补上 intensity,改走 IsaacCreateRTXLidarScanBuffer 标注器:
#   annotator.get_data() → {"data": (N,3) float32 xyz(lidar 系),
#                           "intensity": (N,) float32 反射率 [0,1], ...}
# 然后在主循环里自己用 rclpy 发 sensor_msgs/PointCloud2,字段 [x,y,z,intensity]
# (不含 ring —— AIRY 没有 ring,真实 bag 的 ring 是驱动加的索引,不需要)。
#
# fullScan 稳定性:ScanBuffer 标注器每次 get_data() 返回的是**一整圈完整扫描**;
# 配合 RENDER_HZ=10(rendering_dt=AIRY 整圈周期 0.1s),每个渲染帧恰好产生一整圈,
# 故每条消息=单渲染帧=完整 360° 半球扫描,与原 ROS2RtxLidarHelper(fullScan=True)
# 同样对渲染欠载结构性免疫(质心帧间跳动 <0.1m)。
# transformPoints=False:点保持在 lidar 局部系(frame_id=rslidar),与原行为一致。
#
# 补 ring + per-point timestamp(对齐真实 AIRY 的 6 字段 point_step=26 格式)。
# 本构建 IsaacCreateRTXLidarScanBuffer.get_data() 实测各输出(probe 确认,详见任务报告):
#   * elevation(outputElevation=True):每点仰角,**单位是度**(非文档说的 rad),
#     与几何仰角 atan2(z,√(x²+y²)) 逐点完全相等(max abs diff=0.0)。AIRY 配置表
#     omni:sensor:Core:emitterState:s001:elevationDeg 是 96 个均布值,正好
#     elevation = ring × 0.947°(0, 0.947, …, 89.965)。所以
#     **ring = round(elevation_deg / 0.947) clip[0,95] 即精确线号**(无需 emitterId)。
#   * azimuth(outputAzimuth=True):每点方位角,**单位是度** [-180,180],逐点。
#     用于 per-point timestamp 的圈内偏移。
#   * emitterId(outputEmitterId=True):本构建返回**空数组**(不可用),故走 elevation。
#   * timestamp(outputTimestamp=True):本构建返回数组**大小≠点数**(约半数,
#     uint64 ns),不可逐点对齐,**弃用**;timestamp 改由 azimuth 算圈内偏移。
# per-point timestamp = 本帧 sim_time(= header.stamp)+ 圈内偏移,偏移由方位角线性映射
# 到 [0, 扫描周期 0.1s)。这样首点 ≈ header.stamp,帧内跨度 ≈ 0.1s,且与 /clock、
# IMU、相机同一 sim 时钟(用户强调的关键)。实际 keys 可用 LIDAR_DUMP_KEYS=1 复核。
_lidar_annot = rep.AnnotatorRegistry.get_annotator("IsaacCreateRTXLidarScanBuffer")
_lidar_annot.initialize(outputIntensity=True, transformPoints=False,
                        outputElevation=True, outputAzimuth=True)
_lidar_annot.attach([lidar_rp.path])
# Isaac RTX intensity 是反射率 [0,1];真实 AIRY 是 [1,255](实测 bag:min0 max255
# mean25.8 p95=91,~75% 非零)。线性缩放 ×255 后下限 clip 到 1,量级与真实对齐。
LIDAR_INTENSITY_SCALE = 255.0

# ---- 前视相机(挂 camera_link)----
# 朝向(z-up 世界里的标准前视相机):
#   camera_link 继承 base_link 朝向(x前/y左/z上)。USD 相机默认看自己的 -Z、
#   上方是 +Y;不旋转就朝下看地面。需让相机 -Z 对齐 base +X(前)、+Y 对齐 base +Z(上)。
#   对应旋转矩阵 R(列=相机轴在 base 系)= [[0,0,-1],[-1,0,0],[0,1,0]],
#   四元数 (w,x,y,z)=(0.5,0.5,-0.5,-0.5)。已校验:-Z→(1,0,0),+Y→(0,0,1)。
cam = UsdGeom.Camera.Define(get_current_stage(), CAMERA_LINK + "/front_camera")
cam.GetFocalLengthAttr().Set(CAMERA_FOCAL_LENGTH_MM)
# 用 xformOp 设朝向(Gf.Quatf:实部 + 虚部向量)。AddOrientOp 默认无旋转,显式写入。
cam.AddOrientOp().Set(Gf.Quatf(0.5, Gf.Vec3f(0.5, -0.5, -0.5)))
cam_rp = rep.create.render_product(CAMERA_LINK + "/front_camera", (640, 480))

# ===========================================================================
# 渲染率图(execution 默认 pipeline):相机 RGB + camera_info,lidar 点云。
# OnPlaybackTick 在渲染率触发(~30Hz);lidar 由内部扫描节奏决定发布率(10Hz)。
# ===========================================================================
og.Controller.edit(
    {"graph_path": "/World/render_graph", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("Tick", "omni.graph.action.OnPlaybackTick"),
            ("Ctx", "isaacsim.ros2.bridge.ROS2Context"),
            ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
            ("Rgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            # 本构建 ROS2CameraHelper 不支持 type=camera_info,改用专用 ROS2CameraInfoHelper。
            ("Info", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            # 注:lidar 不再走 ROS2RtxLidarHelper(它只发 XYZ,无 intensity)。
            # 改用上面 attach 的 IsaacCreateRTXLidarScanBuffer 标注器 + 主循环经
            # UNIX socket 推给 sidecar(lidar_pc2_publisher)发 PointCloud2(带 intensity)。
            # 但仍需 RunOnce 强制渲染雷达 render product —— 标注器靠渲染帧驱动。
        ],
        og.Controller.Keys.SET_VALUES: [
            ("Rgb.inputs:renderProductPath", cam_rp.path),
            ("Rgb.inputs:type", "rgb"),
            ("Rgb.inputs:topicName", "/camera/color/image_raw"),
            ("Rgb.inputs:frameId", "camera_link"),
            ("Info.inputs:renderProductPath", cam_rp.path),
            ("Info.inputs:topicName", "/camera/color/camera_info"),
            ("Info.inputs:frameId", "camera_link"),
        ],
        og.Controller.Keys.CONNECT: [
            # 相机 Rgb/Info 必须走 RunOnce(OgnIsaacRunOneSimulationFrame)强制渲染一帧,
            # 否则无头模式下自定义 render product 从不被渲染 → RGB 像素全 0(纯黑)。
            # lidar 此前已走 RunOnce 所以点云正常;相机此前直挂 Tick 故全黑。
            ("Tick.outputs:tick", "RunOnce.inputs:execIn"),
            ("RunOnce.outputs:step", "Rgb.inputs:execIn"),
            ("RunOnce.outputs:step", "Info.inputs:execIn"),
            ("Ctx.outputs:context", "Rgb.inputs:context"),
            ("Ctx.outputs:context", "Info.inputs:context"),
        ],
    })

# ===========================================================================
# 物理步图(ONDEMAND pipeline,OnPhysicsStep 驱动):clock(÷2=100Hz)、
# joint_states(÷4=50Hz)、imu(÷1=200Hz)、ground truth odom + TF(÷2=100Hz)。
# ONDEMAND 必须显式设置,否则 OnPhysicsStep 在非 on-demand 图里不触发。
# ===========================================================================
og.Controller.edit(
    {"graph_path": "/World/ros2_graph", "evaluator_name": "execution",
     "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnStep", "isaacsim.core.nodes.OnPhysicsStep"),
            ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("Ctx", "isaacsim.ros2.bridge.ROS2Context"),
            ("GateClock", "isaacsim.core.nodes.IsaacSimulationGate"),
            ("PubClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("GateJS", "isaacsim.core.nodes.IsaacSimulationGate"),
            ("PubJS", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
            ("PubIMU", "isaacsim.ros2.bridge.ROS2PublishImu"),
            ("Odom", "isaacsim.core.nodes.IsaacComputeOdometry"),
            ("GateOdom", "isaacsim.core.nodes.IsaacSimulationGate"),
            ("PubOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ("PubTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("GateClock.inputs:step", 2),
            ("GateJS.inputs:step", 4),
            ("GateOdom.inputs:step", 2),
            ("PubClock.inputs:topicName", "/clock"),
            ("PubJS.inputs:topicName", "/sim/joint_states"),
            ("PubJS.inputs:targetPrim", ROBOT_ROOT),
            ("ReadIMU.inputs:imuPrim", IMU_SENSOR_PATH),
            ("PubIMU.inputs:topicName", "/sim/imu"),
            ("PubIMU.inputs:frameId", "imu_link"),
            ("Odom.inputs:chassisPrim", ROBOT_ROOT),
            ("PubOdom.inputs:topicName", "/sim/ground_truth_odom"),
            ("PubOdom.inputs:odomFrameId", "odom"),
            ("PubOdom.inputs:chassisFrameId", "base_link"),
            ("PubTF.inputs:parentFrameId", "odom"),
            ("PubTF.inputs:childFrameId", "base_link"),
        ],
        og.Controller.Keys.CONNECT: [
            # context fan-out
            ("Ctx.outputs:context", "PubClock.inputs:context"),
            ("Ctx.outputs:context", "PubJS.inputs:context"),
            ("Ctx.outputs:context", "PubIMU.inputs:context"),
            ("Ctx.outputs:context", "PubOdom.inputs:context"),
            ("Ctx.outputs:context", "PubTF.inputs:context"),
            # clock chain (÷2)
            ("OnStep.outputs:step", "GateClock.inputs:execIn"),
            ("GateClock.outputs:execOut", "PubClock.inputs:execIn"),
            ("SimTime.outputs:simulationTime", "PubClock.inputs:timeStamp"),
            # joint_states chain (÷4)
            ("OnStep.outputs:step", "GateJS.inputs:execIn"),
            ("GateJS.outputs:execOut", "PubJS.inputs:execIn"),
            ("SimTime.outputs:simulationTime", "PubJS.inputs:timeStamp"),
            # imu chain (÷1)
            ("OnStep.outputs:step", "ReadIMU.inputs:execIn"),
            ("ReadIMU.outputs:execOut", "PubIMU.inputs:execIn"),
            ("ReadIMU.outputs:angVel", "PubIMU.inputs:angularVelocity"),
            ("ReadIMU.outputs:linAcc", "PubIMU.inputs:linearAcceleration"),
            ("ReadIMU.outputs:orientation", "PubIMU.inputs:orientation"),
            ("SimTime.outputs:simulationTime", "PubIMU.inputs:timeStamp"),
            # odom + TF chain (÷2)
            ("OnStep.outputs:step", "GateOdom.inputs:execIn"),
            ("GateOdom.outputs:execOut", "Odom.inputs:execIn"),
            ("Odom.outputs:execOut", "PubOdom.inputs:execIn"),
            ("Odom.outputs:position", "PubOdom.inputs:position"),
            ("Odom.outputs:orientation", "PubOdom.inputs:orientation"),
            ("Odom.outputs:linearVelocity", "PubOdom.inputs:linearVelocity"),
            ("Odom.outputs:angularVelocity", "PubOdom.inputs:angularVelocity"),
            ("Odom.outputs:execOut", "PubTF.inputs:execIn"),
            ("Odom.outputs:position", "PubTF.inputs:translation"),
            ("Odom.outputs:orientation", "PubTF.inputs:rotation"),
            ("SimTime.outputs:simulationTime", "PubOdom.inputs:timeStamp"),
            ("SimTime.outputs:simulationTime", "PubTF.inputs:timeStamp"),
        ],
    })

world.reset()
robot.initialize()  # 显式初始化,确保 dof_names 可用

# 驱动增益:steer 位置驱动,wheel 速度驱动
dof = {n: i for i, n in enumerate(robot.dof_names)}
print("DOF_NAMES:", robot.dof_names, flush=True)
missing = [f"{w}_steer_joint" for w in WHEEL_NAMES if f"{w}_steer_joint" not in dof]
missing += [f"{w}_wheel_joint" for w in WHEEL_NAMES if f"{w}_wheel_joint" not in dof]
assert not missing, f"关节名不匹配,缺: {missing}\n实际 DOF: {list(dof)}"
steer_idx = [dof[f"{w}_steer_joint"] for w in WHEEL_NAMES]
wheel_idx = [dof[f"{w}_wheel_joint"] for w in WHEEL_NAMES]
ndof = len(robot.dof_names)
kps = np.zeros(ndof)
kds = np.zeros(ndof)
# steer 位置驱动刚度(W2_STEER_KP/KD 可调)。转向瞬态(achieved 角追命令角的滞后)是
# wheel-LS ω_z 残余抖的次要来源;主因是轮速环阻尼(见下方 WHEEL_KD)。保持原刚度即可。
STEER_KP = float(os.environ.get("W2_STEER_KP", "1e4"))
STEER_KD = float(os.environ.get("W2_STEER_KD", "1e3"))
# 轮速驱动增益 —— Task 26 的核心修复(2026-06-16)。
# wheel-LS ω_z 的 RMS 几乎全是高频抖动(分频:5/11 帧平滑后掉到 ~0.07;真值 ω_z 本身光滑,
# 高频 RMS 仅 0.013),全来自 achieved 轮速读数振荡:轮关节角速度被高 kd 纯阻尼速度环驱成
# 欠阻尼,单轮速度抖 ±0.14 m/s(驱动速度 0.5~1 的 15~25%),经 8×3 LS(b=v·[cosθ,sinθ])
# 直接放大成 ω_z 抖。轮子物理上滚得很稳(真值光滑、speed_ratio=1.0),抖只在**报出来的关节
# 速度**里;与轮地摩擦/碰撞/solver 无关(那些不动速度环刚度)。
# 把轮速环阻尼 kd 降下来消振铃。实测 RMS(wheel-LS ω_z vs 真值,ep19 seed1019,原始 render-步
# 控制环、雷达 10Hz 不变):kd=1e3→0.195, 300→0.109, 120→0.044, 60→见下;
# 太低(<~30)机器人欠驱(achieved 轮速跟不上命令)。kd=120 兼顾低抖(~0.045)与满程行进
# (ratio 1.0,pathlen 13.5m),逼近真机 0.05。W2_WHEEL_KD 可调。
WHEEL_KD = float(os.environ.get("W2_WHEEL_KD", "120.0"))
for i in steer_idx:
    kps[i], kds[i] = STEER_KP, STEER_KD
for i in wheel_idx:
    kps[i], kds[i] = 0.0, WHEEL_KD
robot.get_articulation_controller().set_gains(kps=kps, kds=kds)
print(f"STEER_GAINS: kp={STEER_KP} kd={STEER_KD}  WHEEL_KD={WHEEL_KD}", flush=True)

profile = generate_profile(args.seed, args.duration, rate_hz=CMD_HZ)
prev_angles = np.zeros(4)
box = args.safe_box


def yaw_of(quat_wxyz):
    w, x, y, z = quat_wxyz
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


steer_idx_arr = np.array(steer_idx)
wheel_idx_arr = np.array(wheel_idx)

# ===========================================================================
# /rslidar_points(带 intensity)发布 —— 走 sidecar 进程,不在 Isaac 内用 rclpy。
#
# 为什么不在 Isaac 进程里直接 rclpy 发:Isaac 嵌入式 Python 是 cp311,而
# `source /opt/ros/jazzy` 的系统 ROS2 python 栈是 cp312。两套 rosidl typesupport
# (rclpy._rclpy / rcl_interfaces 的 *_s.c .so)ABI 不兼容:
#   * 直接 import 系统 cp312 rclpy → "No module named 'rclpy._rclpy_pybind11'"
#     (找不到 cp311 .so)崩溃;
#   * 改用 bridge 自带的 cp311 rclpy(<ext>/jazzy/rclpy)→ Node() 创建时 cp311
#     的 _rclpy 撞上经 LD_LIBRARY_PATH/AMENT 解析到的 /opt/ros cp312 typesupport,
#     在 rcl_interfaces.ParameterEvent 转换处触发 C 断言 abort(core dumped,实测)。
# 这是经典双 ROS 安装混装冲突,在单进程内难以干净化解。
#
# 同时本构建的 ROS2RtxLidarHelper / ROS2PublishPointCloud og 节点只发 XYZ
# (point3f),没有 intensity 字段;真实 AIRY 的 /rslidar_points 带 intensity。
#
# 方案:Isaac 主循环把每帧 xyz+intensity 通过 UNIX 域 socket(SOCK_STREAM,长度前缀
# 分帧)发给一个**系统 cp312 python 的 sidecar 节点**(scripts/lidar_pc2_publisher.py,
# 由 collect_episode.sh 用 ros2 run 同 degradation_node 一样拉起),由它组
# sensor_msgs/PointCloud2(字段 [x,y,z,intensity],无 ring)发 /rslidar_points。
# sidecar 跑在干净的系统 ROS2 环境里,rclpy 正常工作。
#
# fullScan 稳定性:IsaacCreateRTXLidarScanBuffer 每次 get_data() 返回一整圈完整扫描;
# 配合 RENDER_HZ=10(rendering_dt=AIRY 整圈周期 0.1s),每渲染帧=一整圈,故每条消息
# =单渲染帧=完整 360° 半球扫描,对渲染欠载结构性免疫(质心帧间跳动 <0.1m)。
# 时间戳用 sim-time(world.current_time),与 /clock、相机、imu 一致。
import socket  # noqa: E402
import struct  # noqa: E402

_LIDAR_SOCK_PATH = os.environ.get("W2_LIDAR_SOCK", "/tmp/w2_sim_lidar.sock")
# 帧头:magic(u32) seq(u32) sim_time(f64) n_points(u32) point_step(u32)。
# 之后紧跟 n*point_step 字节的已交错打包载荷,每点 26 字节,对齐真实 AIRY:
#   x f32@0  y f32@4  z f32@8  intensity f32@12  ring u16@16  timestamp f64@18。
# timestamp 已是绝对 sim-time(= 本帧 sim_time + 圈内偏移),sidecar 无需再加工。
_LIDAR_HDR = struct.Struct("<IIdII")
_LIDAR_MAGIC = 0x52534C44  # 'RSLD'
_LIDAR_POINT_STEP = 26
_LIDAR_DUMP_KEYS = os.environ.get("LIDAR_DUMP_KEYS", "0") == "1"
_lidar_sock = None
_lidar_seq = 0
_lidar_keys_dumped = False

# numpy structured dtype 用于把混合类型字段交错打包成真实 AIRY 的 26 字节/点布局。
# 不能用单一 float32 (n,6):ring 是 u16、timestamp 是 f64,类型/字宽不同。
_LIDAR_REC_DTYPE = np.dtype({
    "names": ["x", "y", "z", "intensity", "ring", "timestamp"],
    "formats": [np.float32, np.float32, np.float32, np.float32, np.uint16, np.float64],
    "offsets": [0, 4, 8, 12, 16, 18],
    "itemsize": 26,
}, align=False)


def _lidar_connect():
    """连 sidecar 的 UNIX socket(sidecar 先起、监听)。失败返回 None,主循环重试。"""
    global _lidar_sock
    if _lidar_sock is not None:
        return _lidar_sock
    try:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.connect(_LIDAR_SOCK_PATH)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 8 << 20)
        _lidar_sock = s
        print(f"LIDAR_SOCK: connected {_LIDAR_SOCK_PATH}", flush=True)
    except (FileNotFoundError, ConnectionRefusedError, OSError):
        _lidar_sock = None
    return _lidar_sock


def publish_lidar(sim_time):
    """从标注器取一整圈 xyz+intensity+ring+per-point-timestamp,交错打包成真实 AIRY
    的 26 字节/点布局,经 socket 发给 sidecar。

    ring 来源:elevation(度)→ round(el_deg/0.947) clip[0,95](AIRY 96 线均布,精确线号)。
    timestamp 来源:azimuth(度 [-180,180])归一化到 [0,1)×扫描周期(0.1s)得圈内偏移,
      加帧 sim_time 基准得绝对 sim-time(与 /clock/IMU/相机同钟,首点≈header.stamp)。
    """
    global _lidar_sock, _lidar_seq, _lidar_keys_dumped
    data = _lidar_annot.get_data()
    if not isinstance(data, dict):
        return 0
    if _LIDAR_DUMP_KEYS and not _lidar_keys_dumped:
        _lidar_keys_dumped = True
        info = []
        for k, v in data.items():
            arr = np.asarray(v) if not isinstance(v, np.ndarray) else v
            info.append(f"{k}{getattr(arr,'shape',None)}/{getattr(arr,'dtype',None)}")
        print("LIDAR_GET_DATA_KEYS:", " ".join(info), flush=True)
    xyz = data.get("data")
    inten = data.get("intensity")
    if xyz is None or inten is None:
        return 0
    xyz = np.asarray(xyz, dtype=np.float32)
    inten = np.asarray(inten, dtype=np.float32).ravel()
    if xyz.ndim != 2 or xyz.shape[0] == 0 or inten.shape[0] != xyz.shape[0]:
        return 0
    n = int(xyz.shape[0])
    # Isaac 反射率 [0,1] → AIRY 量级 [1,255]:×255 后 clip 到 [1,255]。
    inten = np.clip(inten * LIDAR_INTENSITY_SCALE, 1.0, 255.0).astype(np.float32)

    # ---- ring(UINT16,0..95):elevation(度)→ round(/0.947),AIRY 96 线精确线号 ----
    elev = data.get("elevation")
    if elev is not None and np.asarray(elev).ravel().shape[0] == n:
        el_deg = np.asarray(elev, dtype=np.float64).ravel()  # 本构建 elevation 已是度
    else:
        # 兜底:从 xyz 反算仰角(度)。
        el_deg = np.degrees(np.arctan2(
            xyz[:, 2], np.sqrt(xyz[:, 0] ** 2 + xyz[:, 1] ** 2)))
    ring = np.clip(np.round(el_deg / 0.947), 0, 95).astype(np.uint16)

    # ---- per-point timestamp(绝对 sim-time,FLOAT64):azimuth(度)→ 圈内偏移 ----
    SCAN_PERIOD = 1.0 / RENDER_HZ  # 0.1s = 一整圈周期
    az = data.get("azimuth")
    if az is not None and np.asarray(az).ravel().shape[0] == n:
        a_deg = np.asarray(az, dtype=np.float64).ravel()  # 本构建 azimuth 已是度 [-180,180]
    else:
        a_deg = np.degrees(np.arctan2(
            xyz[:, 1].astype(np.float64), xyz[:, 0].astype(np.float64)))
    # 方位角 [-180,180] 归一化到 [0,1) × 扫描周期 → 圈内偏移 [0, 0.1)。
    off = ((a_deg + 180.0) / 360.0) * SCAN_PERIOD
    ts_abs = float(sim_time) + off

    # ---- 交错打包 26 字节/点 ----
    rec = np.empty(n, dtype=_LIDAR_REC_DTYPE)
    rec["x"] = xyz[:, 0]
    rec["y"] = xyz[:, 1]
    rec["z"] = xyz[:, 2]
    rec["intensity"] = inten
    rec["ring"] = ring
    rec["timestamp"] = ts_abs
    payload = rec.tobytes()
    s = _lidar_connect()
    if s is None:
        return 0
    _lidar_seq += 1
    try:
        s.sendall(_LIDAR_HDR.pack(_LIDAR_MAGIC, _lidar_seq, float(sim_time),
                                  n, _LIDAR_POINT_STEP) + payload)
    except (BrokenPipeError, ConnectionResetError, OSError) as e:
        print(f"WARN lidar sock send failed ({e}); will reconnect", file=sys.stderr, flush=True)
        try:
            s.close()
        except OSError:
            pass
        _lidar_sock = None
        return 0
    return n

# 时间驱动:world.step(render=True) 每次推进一个 rendering_dt(实测此 build 渲染步进,
# 不是 physics_dt),所以用 world.current_time 计时,对任意 per-step dt 都正确。
# t0 锚定 reset 后初值,t = current_time - t0 为本次仿真已经过的秒数。
t0 = world.current_time
print(f"LOOP_START current_time={t0:.3f} duration={args.duration}", flush=True)
# ---------------------------------------------------------------------------
# Task 26 真因(2026-06-16):wheel-LS ω_z 的 RMS 几乎全是高频抖动——achieved 轮速读数在
# 8×3 LS(b=v·[cosθ,sinθ])里被放大成 ω_z 抖。真值 ω_z 本身光滑(高频 RMS 仅 0.013),
# 轮子也不纵向打滑(mu=1.5 时 speed_ratio=1.000;mu=0.01 时 1.8 才是真打滑)。抖源是轮速
# velocity-drive 的阻尼 kd(见 set_gains):kd 过高 velocity 环对接触力扰动过敏 → 读数振铃。
# 摩擦/碰撞/solver 对该抖几乎无效;唯一必需接触项是 mu≥~1(防纵向打滑)。
# 控制环:每个 render 步(world.step(render=True) 推进 1/RENDER_HZ=0.1s,内含 20 个 200Hz
# 物理子步)下发一次命令,命令在 20 子步内保持、velocity drive 有充分时间 settle。
# (注:此 build 里 world.step(render=True) 一次推进整 render_dt,不能在 render=False 子步
#  之间穿插单次 render=True 来提高命令率——那样 render=True 仍跳 0.1s、会扭曲 sim 时间轴
#  并把雷达率腰斩。命令率=render 率=10Hz 是此 build 的硬约束。)
# RENDER_HZ=10 不动 → AIRY 雷达每帧一整圈、相机 10Hz、joint_states/imu/odom 物理图节奏不变。
_lidar_msg_count = 0
while world.current_time - t0 < args.duration:
    world.step(render=True)
    # 发布雷达点云(带 intensity)。每渲染帧标注器=一整圈,sim-time 戳。
    if publish_lidar(world.current_time) > 0:
        _lidar_msg_count += 1
    t = world.current_time - t0
    cmd = profile[min(int(t * CMD_HZ), len(profile) - 1), 1:]
    pos, quat = robot.get_world_pose()
    if not (box[0] < pos[0] < box[1] and box[2] < pos[1] < box[3]):
        # 出安全框:世界系朝原点 0.5m/s,转到 base 系
        yaw = yaw_of(quat)
        d = -pos[:2] / max(np.linalg.norm(pos[:2]), 1e-6) * 0.5
        c, s = np.cos(-yaw), np.sin(-yaw)
        cmd = np.array([c * d[0] - s * d[1], s * d[0] + c * d[1], 0.0])
    ang, spd = inverse_kinematics(cmd[0], cmd[1], cmd[2], prev_angles=prev_angles)
    prev_angles = ang
    robot.apply_action(ArticulationAction(
        joint_positions=ang, joint_indices=steer_idx_arr))
    robot.apply_action(ArticulationAction(
        joint_velocities=spd / WHEEL_RADIUS, joint_indices=wheel_idx_arr))

print(f"SIM_DONE lidar_msgs_sent={_lidar_msg_count}", flush=True)
try:
    _lidar_annot.detach()
    if _lidar_sock is not None:
        _lidar_sock.close()
except Exception as _e:
    print(f"WARN lidar cleanup: {_e}", file=sys.stderr, flush=True)
app.close()
