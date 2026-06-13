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
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from pxr import Gf, UsdGeom, UsdPhysics  # noqa: E402

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.sensors.physics")
enable_extension("isaacsim.sensors.rtx")
app.update()

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from w2_sim.swerve_kinematics import WHEEL_NAMES, WHEEL_RADIUS, inverse_kinematics  # noqa: E402
from w2_sim.trajectory_generator import generate_profile  # noqa: E402

PHYS_HZ, CMD_HZ = 200.0, 50.0
RENDER_HZ = 30.0
CAMERA_FOCAL_LENGTH_MM = 18.0  # ~50° HFoV(aperture 默认 20.955mm),近似前视相机

world = World(stage_units_in_meters=1.0,
              physics_dt=1.0 / PHYS_HZ, rendering_dt=1.0 / RENDER_HZ)

# ---- 场景 ----
if args.env == "warehouse":
    assets = get_assets_root_path()
    if assets:
        add_reference_to_stage(
            usd_path=assets + "/Isaac/Environments/Simple_Warehouse/warehouse.usd",
            prim_path="/World/env")
    else:
        print("WARN: 云资产不可达,退回平地", file=sys.stderr, flush=True)
        world.scene.add_default_ground_plane()
else:
    world.scene.add_default_ground_plane()

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

# ---- IMU 传感器(挂 imu_link)----
# 注:父 prim 必须带 RigidBodyAPI;imu_link 是 rigid body(实测),用拍平后的真实路径。
IMU_SENSOR_PATH = IMU_LINK + "/imu_sensor"
omni.kit.commands.execute(
    "IsaacSensorCreateImuSensor",
    path="imu_sensor", parent=IMU_LINK,
    translation=Gf.Vec3d(0, 0, 0), orientation=Gf.Quatd(1, 0, 0, 0))

# ---- RTX Lidar(挂 rslidar)----
# 内置配置(Ouster/OS1/OS1_REV6_32ch10hz1024res.json),传 basename。
LIDAR_CONFIG = "OS1_REV6_32ch10hz1024res"
_, lidar_prim = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path="rtx_lidar", parent=RSLIDAR_LINK,
    config=LIDAR_CONFIG,
    translation=Gf.Vec3d(0, 0, 0), orientation=Gf.Quatd(1, 0, 0, 0))
lidar_rp = rep.create.render_product(lidar_prim.GetPath(), [1, 1])

# ---- 前视相机(挂 camera_link)----
cam = UsdGeom.Camera.Define(get_current_stage(), CAMERA_LINK + "/front_camera")
cam.GetFocalLengthAttr().Set(CAMERA_FOCAL_LENGTH_MM)
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
            ("Lidar", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("Rgb.inputs:renderProductPath", cam_rp.path),
            ("Rgb.inputs:type", "rgb"),
            ("Rgb.inputs:topicName", "/camera/color/image_raw"),
            ("Rgb.inputs:frameId", "camera_link"),
            ("Info.inputs:renderProductPath", cam_rp.path),
            ("Info.inputs:topicName", "/camera/color/camera_info"),
            ("Info.inputs:frameId", "camera_link"),
            ("Lidar.inputs:renderProductPath", lidar_rp.path),
            ("Lidar.inputs:type", "point_cloud"),
            ("Lidar.inputs:topicName", "/rslidar_points"),
            ("Lidar.inputs:frameId", "rslidar"),
            # fullScan=True:累积满一整圈再发(≈10Hz,匹配 OS1 旋转率),
            # 而非每渲染帧发部分扫描(默认 False → 30Hz 稀疏帧)。
            ("Lidar.inputs:fullScan", True),
        ],
        og.Controller.Keys.CONNECT: [
            ("Tick.outputs:tick", "Rgb.inputs:execIn"),
            ("Tick.outputs:tick", "Info.inputs:execIn"),
            ("Tick.outputs:tick", "RunOnce.inputs:execIn"),
            ("RunOnce.outputs:step", "Lidar.inputs:execIn"),
            ("Ctx.outputs:context", "Rgb.inputs:context"),
            ("Ctx.outputs:context", "Info.inputs:context"),
            ("Ctx.outputs:context", "Lidar.inputs:context"),
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
for i in steer_idx:
    kps[i], kds[i] = 1e4, 1e3
for i in wheel_idx:
    kps[i], kds[i] = 0.0, 1e3
robot.get_articulation_controller().set_gains(kps=kps, kds=kds)

profile = generate_profile(args.seed, args.duration, rate_hz=CMD_HZ)
prev_angles = np.zeros(4)
box = args.safe_box


def yaw_of(quat_wxyz):
    w, x, y, z = quat_wxyz
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


steer_idx_arr = np.array(steer_idx)
wheel_idx_arr = np.array(wheel_idx)

# 用显式步数计数推进仿真(每步 1/PHYS_HZ 秒),不依赖 world.current_time 的初值——
# 仿真时间 t = step / PHYS_HZ。world.step(render=True) 每调用一次推进一个 physics_dt。
n_steps = int(round(args.duration * PHYS_HZ))
print(f"LOOP_START current_time={world.current_time:.3f} duration={args.duration} "
      f"n_steps={n_steps}", flush=True)
for step in range(n_steps):
    world.step(render=True)
    t = step / PHYS_HZ
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

print("SIM_DONE", flush=True)
app.close()
