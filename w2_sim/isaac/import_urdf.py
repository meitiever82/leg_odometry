#!/usr/bin/env python3
"""isaac/import_urdf.py — URDF → USD 导入(在 Isaac Sim 的 python 下运行)。

用法:
  $ISAACSIM_DIR/python.sh isaac/import_urdf.py /tmp/w2_swerve.urdf ~/rtabmap_ws/w2/usd/w2_swerve.usd

Isaac Sim 5.1.0-rc.19 实测可用 API(与计划一致,已用扩展源码/pybind .so 核对):
  - 扩展名:isaacsim.asset.importer.urdf
  - 命令名:URDFParseAndImportFile(urdf_path, import_config, dest_path, get_articulation_root)
  - ImportConfig 属性:fix_base / merge_fixed_joints / convex_decomp / self_collision /
    default_drive_strength / default_position_drive_damping(均为可直接赋值的 pybind property)
"""
import os
import sys

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import omni.kit.commands  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

enable_extension("isaacsim.asset.importer.urdf")
from isaacsim.asset.importer.urdf import _urdf  # noqa: E402

urdf_path = os.path.abspath(os.path.expanduser(sys.argv[1]))
usd_path = os.path.abspath(os.path.expanduser(sys.argv[2]))
os.makedirs(os.path.dirname(usd_path), exist_ok=True)

cfg = _urdf.ImportConfig()
cfg.fix_base = False
cfg.merge_fixed_joints = False      # 保留 rslidar/imu_link/camera_link 帧
cfg.convex_decomp = False
cfg.self_collision = False
cfg.default_drive_strength = 1e4            # steer 位置驱动刚度,运行时还会再调
cfg.default_position_drive_damping = 1e3

status, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=urdf_path,
    import_config=cfg,
    dest_path=usd_path,
    get_articulation_root=True,
)
# 5.1.0-rc.19 importer 生成的 USD 没设 defaultPrim(根 prim 为 /<robot_name>),
# 导致 add_reference_to_stage 引用时组合不出任何 prim。这里补设 defaultPrim。
if status:
    from pxr import Usd  # noqa: E402

    stage = Usd.Stage.Open(usd_path)
    if not stage.GetDefaultPrim():
        roots = stage.GetPseudoRoot().GetChildren()
        stage.SetDefaultPrim(roots[0])
        stage.Save()
        print("DEFAULT_PRIM_SET:", roots[0].GetPath(), flush=True)

# flush=True 必须:stdout 走管道时为块缓冲,app.close() 内部 hard-exit 会吞掉未刷新的输出
print("IMPORT_OK" if status else "IMPORT_FAIL", prim_path, flush=True)
app.close()
sys.exit(0 if status else 1)
