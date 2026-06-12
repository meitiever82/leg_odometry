#!/usr/bin/env python3
"""isaac/verify_usd.py — 加载 USD,确认 articulation 的 DOF 名单。

用法:
  $ISAACSIM_DIR/python.sh isaac/verify_usd.py ~/rtabmap_ws/w2/usd/w2_swerve.usd

期望:恰好 8 个底盘 DOF(4×steer + 4×wheel),上身 19 关节已锁 fixed 不出现。

注:articulation root 不在 /World/w2 本身——5.1.0-rc.19 importer 把
PhysicsArticulationRootAPI 挂在 /World/w2/base_footprint/base_footprint(实测)。
本脚本在 reference 加载后扫描 stage 找带 UsdPhysics.ArticulationRootAPI 的 prim,
用实际 path 构造 SingleArticulation,所以 root 位置变了也不用改脚本。
"""
import os
import sys

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from pxr import UsdPhysics  # noqa: E402

usd_path = os.path.abspath(os.path.expanduser(sys.argv[1]))

world = World(stage_units_in_meters=1.0)
add_reference_to_stage(usd_path=usd_path, prim_path="/World/w2")
world.scene.add_default_ground_plane()

# 找实际 articulation root(可能是 /World/w2 也可能是其子 prim)
stage = world.stage
root_candidates = [
    str(p.GetPath())
    for p in stage.Traverse()
    if p.HasAPI(UsdPhysics.ArticulationRootAPI) and str(p.GetPath()).startswith("/World/w2")
]
print("ARTICULATION_ROOT_CANDIDATES:", root_candidates, flush=True)
assert root_candidates, "stage 里没有带 ArticulationRootAPI 的 prim"
root_path = root_candidates[0]

robot = SingleArticulation(root_path)
world.reset()
robot.initialize()  # 不挂进 world.scene 的话 reset 不会自动初始化,dof_names 会是 None
names = robot.dof_names
print("DOFS:", names, flush=True)
expected = {f"{p}_{s}_joint" for p in
            ["front_left", "front_right", "rear_left", "rear_right"]
            for s in ["steer", "wheel"]}
assert expected.issubset(set(names)), f"缺关节: {expected - set(names)}"
assert len(names) == 8, f"DOF 数 {len(names)} != 8(上身没锁住?): {names}"
print("USD_OK", flush=True)
app.close()
