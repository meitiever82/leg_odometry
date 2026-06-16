#!/usr/bin/env python3
"""isaac/import_urdf.py — URDF → USD 导入(在 Isaac Sim 的 python 下运行)。

用法:
  $ISAACSIM_DIR/python.sh isaac/import_urdf.py /tmp/w2_swerve.urdf ~/rtabmap_ws/w2/usd/w2_swerve.usd

Isaac Sim 5.1.0-rc.19 实测可用 API(与计划一致,已用扩展源码/pybind .so 核对):
  - 扩展名:isaacsim.asset.importer.urdf
  - 命令名:URDFParseFile / URDFImportRobot(dest_path="" 时导入到当前 stage)
  - ImportConfig 属性:fix_base / merge_fixed_joints / convex_decomp / self_collision /
    replace_cylinders_with_capsules / make_default_prim /
    default_drive_strength / default_position_drive_damping(均为可直接赋值的 pybind property)

碰撞几何修复(Task 9b,2026-06-14)
==================================
**根因**:5.1.0-rc.19 importer 用 URDFParseAndImportFile + dest_path 时把输出拆成
configuration/{_base,_physics,_robot,_sensor}.usd 多层结构,几何放在 _base.usd 的
**顶层 sibling scope** `/visuals`、`/colliders`、`/meshes`(不是 defaultPrim 子树),
每个 link 的 `visuals`/`collisions` Xform 用 **internal reference**(空 assetPath)指向
`/colliders/<link>`。这些 scope 是 defaultPrim `/W2EVT1Urdf_swerve` 的兄弟。

当主程序 `add_reference_to_stage(prim_path="/World/w2")` 只引用 defaultPrim 子树时,
兄弟 scope `/visuals` `/colliders` `/meshes` 被丢弃 → internal reference 解析失败 →
**所有几何 + CollisionAPI 消失**,只剩直接挂在 link 上的 RigidBodyAPI(32 个)。
后果:轮子无碰撞体 → 与地面零摩擦 → Task 9 跑仿真机器人原地不动(30s 位移 0.000m)。

**修复**(本脚本,治本路线 A):
  1. 用 `dest_path=""` 让 importer 把机器人导入**当前内存 stage**(单 stage,
     `/visuals` `/colliders` `/meshes` 是真实并存的 concrete spec,几何齐全:Cyl=8 Caps=4
     CollAPI=24 RB=32)。
  2. 把每个 link 的 `visuals`/`collisions` Xform 引用的 `/colliders/<link>`、
     `/visuals/<link>` 子内容 **CopySpec 物理拷贝**进 link 子树,清掉 internal reference,
     使 defaultPrim 子树自包含。
  3. **只给 4 个 wheel_link 保留碰撞几何**(轮子接地需要);base_link 的 48MB SolidWorks
     mesh 碰撞体**主动丢弃**——PhysX 禁止动态刚体用非凸三角网格碰撞,且平地采集不撞墙,
     车体不需要碰撞。结果:4 轮各 1 个碰撞体、车体 0 碰撞 → 机器人靠 4 轮支撑。
  4. `replace_cylinders_with_capsules=True`:轮子碰撞用胶囊(滚动稳健,半径 0.075,
     轴随 URDF 的 rpy="pi/2 0 0" 旋到与轮自转轴一致)。
  5. 删空的 `/visuals` `/colliders` `/meshes` 顶层 scope,设 defaultPrim,导出单文件 USD。

修复后用 add_reference_to_stage 引用,实测 Cyl=8 Caps=4 CollAPI=4(4 轮)RB=32,
4 个 wheel_link 各有一个胶囊碰撞体,车体 0 碰撞。verify_usd.py 仍过(8 DOF)。
"""
import os
import sys

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import omni.kit.commands  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.stage import (  # noqa: E402
    create_new_stage,
    get_current_stage,
)

enable_extension("isaacsim.asset.importer.urdf")
from isaacsim.asset.importer.urdf import _urdf  # noqa: E402
from pxr import Sdf, UsdPhysics  # noqa: E402

urdf_path = os.path.abspath(os.path.expanduser(sys.argv[1]))
usd_path = os.path.abspath(os.path.expanduser(sys.argv[2]))
os.makedirs(os.path.dirname(usd_path), exist_ok=True)

# 4 个轮子 link —— 唯一需要保留碰撞几何的 link(机器人靠它们接地支撑)。
WHEEL_LINKS = {
    "front_left_wheel_link",
    "front_right_wheel_link",
    "rear_left_wheel_link",
    "rear_right_wheel_link",
}
# 顶层几何 scope(importer 放几何的地方;link 用 internal reference 指过去)。
GEOM_SCOPES = ("/visuals", "/colliders", "/meshes")

cfg = _urdf.ImportConfig()
cfg.fix_base = False
cfg.merge_fixed_joints = False      # 保留 rslidar/imu_link/camera_link 帧
cfg.convex_decomp = False
cfg.self_collision = False
# 轮子碰撞体:默认 cylinder(线接触,接地更像真实轮缘);W2_WHEEL_CAPSULE=1 退回 capsule。
# 注:Task 26 实测碰撞几何对 wheel-LS ω_z 抖几乎无影响(抖源是轮速 velocity 环阻尼,
# 见 w2_sim_app.py set_gains),cylinder 仅取其更贴真实的线接触;两者都需高摩擦材质防纵向打滑。
cfg.replace_cylinders_with_capsules = os.environ.get("W2_WHEEL_CAPSULE", "0") == "1"
cfg.make_default_prim = True
cfg.default_drive_strength = 1e4            # steer 位置驱动刚度,运行时还会再调
cfg.default_position_drive_damping = 1e3

# ---- 关键:dest_path="" 导入到当前内存 stage(单 stage,几何 scope 真实并存)----
create_new_stage()
status, robot_model = omni.kit.commands.execute(
    "URDFParseFile", urdf_path=urdf_path, import_config=cfg)
assert status, "URDFParseFile 失败"
ok, _result = omni.kit.commands.execute(
    "URDFImportRobot",
    urdf_robot=robot_model,
    import_config=cfg,
    urdf_path=urdf_path,
    dest_path="",  # 空 → 导入当前 stage,不拆多层文件
)
assert ok, f"URDFImportRobot 失败,res={_result}"
print("IMPORT_RES:", _result, flush=True)

stage = get_current_stage()
layer = stage.GetRootLayer()
root_prim = stage.GetDefaultPrim()
assert root_prim, "importer 未设置 defaultPrim(make_default_prim 未生效?)"
root_path = str(root_prim.GetPath())
print("ROOT_PRIM:", root_path, flush=True)


def internal_ref_targets(prim):
    """返回 prim 上 internal reference(空 assetPath)指向的 prim path 列表。"""
    tgts = []
    for spec in prim.GetPrimStack():
        for r in spec.referenceList.GetAddedOrExplicitItems():
            if (not r.assetPath) and r.primPath:
                tgts.append(str(r.primPath))
    return tgts


def link_name_of(prim_path):
    """/W2EVT1Urdf_swerve/front_left_wheel_link/collisions → front_left_wheel_link"""
    parts = prim_path[len(root_path) + 1:].split("/")
    return parts[0] if parts else ""


# 收集所有需要 reparent 的 (dst_xform_path, src_scope_path):
#   dst = link 下的 visuals/collisions Xform;src = /colliders/<link> 或 /visuals/<link>
work = []
for p in stage.Traverse():
    pp = str(p.GetPath())
    if not pp.startswith(root_path + "/"):
        continue
    for tgt in internal_ref_targets(p):
        if tgt.startswith(GEOM_SCOPES):
            work.append((pp, tgt))

print(f"REPARENT_WORK: {len(work)} internal refs", flush=True)

dropped_body_collision = 0
for dst_path, src_path in work:
    is_collision = dst_path.endswith("/collisions")
    link = link_name_of(dst_path)
    # 只给轮子保留碰撞几何;其它 link 的碰撞(主要是 base_link 48MB mesh)主动丢弃。
    if is_collision and link not in WHEEL_LINKS:
        dst_prim = stage.GetPrimAtPath(dst_path)
        dst_prim.GetReferences().ClearReferences()  # 断引用,collisions Xform 留空
        dropped_body_collision += 1
        continue
    src_prim = stage.GetPrimAtPath(src_path)
    if not src_prim or not layer.GetPrimAtPath(src_path):
        # src scope 此路径不在 layer 里(importer 结构异常),跳过——末尾 assert 会捕获漏拷的情况。
        continue
    for child in src_prim.GetChildren():
        dst_child = Sdf.Path(dst_path + "/" + child.GetName())
        Sdf.CopySpec(layer, Sdf.Path(src_path + "/" + child.GetName()), layer, dst_child)
    stage.GetPrimAtPath(dst_path).GetReferences().ClearReferences()

print(f"DROPPED_BODY_COLLISION: {dropped_body_collision}", flush=True)

# 清理残留的悬挂 internal reference:上身 link 的 **visual** 子 prim(waist_*/left_Link*/
# right_Link*/head_* 等)引用 `/meshes/<link>` 外部 SolidWorks mesh,在内存 stage 里这些
# mesh 数据未内联到 GEOM_SCOPES,删 scope 后引用悬挂 → 一堆 "Unresolved reference" 警告。
# 这些纯属上身**视觉**外观(headless 数据采集不渲染、不影响物理/关节/碰撞),直接清掉引用,
# 让导出的 USD 自洽无警告。轮子/传感器帧的几何与碰撞前面已 reparent 进 link 子树,不受影响。
cleared_dangling = 0
for p in stage.Traverse():
    pp = str(p.GetPath())
    if not pp.startswith(root_path + "/"):
        continue
    for tgt in internal_ref_targets(p):
        if tgt.startswith(GEOM_SCOPES):
            p.GetReferences().ClearReferences()
            cleared_dangling += 1
            break
print(f"CLEARED_DANGLING_VISUAL_REFS: {cleared_dangling}", flush=True)

# 删空的顶层几何 scope(几何已拷进 link 子树,这些 scope 现已无用)。
for scope in GEOM_SCOPES:
    if stage.GetPrimAtPath(scope):
        stage.RemovePrim(scope)

# 删除 importer 默认相机等非机器人 prim(导出只要 defaultPrim 子树,但顶层杂项一并清掉)。
for c in list(stage.GetPseudoRoot().GetChildren()):
    cp = str(c.GetPath())
    # Omniverse Kit 5.1.x 默认视口 prim;版本升级后如有新名称在此追加
    if cp != root_path and c.GetName() in (
        "Render", "OmniverseKit_Persp", "OmniverseKit_Front",
        "OmniverseKit_Top", "OmniverseKit_Right",
    ):
        stage.RemovePrim(c.GetPath())

# 确保 defaultPrim 仍设对
if not stage.GetDefaultPrim():
    stage.SetDefaultPrim(stage.GetPrimAtPath(root_path))

# 碰撞自检:统计 4 轮碰撞体
coll_prims = [str(p.GetPath()) for p in stage.Traverse()
              if p.HasAPI(UsdPhysics.CollisionAPI)]
wheel_coll = [c for c in coll_prims if "wheel_link" in c]
body_coll = [c for c in coll_prims if "wheel_link" not in c]
print(f"SELFCHECK CollAPI total={len(coll_prims)} wheels={len(wheel_coll)} "
      f"body={len(body_coll)}", flush=True)
for w in sorted(WHEEL_LINKS):
    has = any(w in c for c in coll_prims)
    print(f"  {w}: collision={has}", flush=True)
assert len(wheel_coll) == 4, f"轮子碰撞体数 {len(wheel_coll)} != 4"
assert len(body_coll) == 0, f"车体仍有 {len(body_coll)} 个碰撞体(应为 0): {body_coll}"

# 导出单文件 USD(自包含,几何在 defaultPrim 子树内,引用不丢)。
layer.Export(usd_path)
print("IMPORT_OK", root_path, "->", usd_path, flush=True)

# flush=True 必须:stdout 走管道时为块缓冲,app.close() 内部 hard-exit 会吞掉未刷新输出
app.close()
sys.exit(0)
