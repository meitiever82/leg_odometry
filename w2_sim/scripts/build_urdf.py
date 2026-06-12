#!/usr/bin/env python3
# scripts/build_urdf.py — xacro 展开 + 锁上身关节 + mesh 路径绝对化。
# 用法: python3 scripts/build_urdf.py [输出路径, 默认 /tmp/w2_swerve.urdf]
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
XACRO = PKG / "urdf" / "W2EVT1Urdf_swerve.urdf.xacro"
MESH_DIR = Path.home() / "rtabmap_ws" / "w2" / "meshes"
# SolidWorks 上身全部关节 → fixed(design §4:保留质量分布,本期不驱动)
UPPER_BODY_JOINTS = (
    ["waist_updown_joint", "waist_pitch_joint", "waist_yaw_joint",
     "head_yaw_joint", "head_pitch_joint"]
    + [f"left_joint{i}" for i in range(1, 8)]
    + [f"right_joint{i}" for i in range(1, 8)]
)


def main(out_path):
    urdf_xml = subprocess.run(["xacro", str(XACRO)], check=True,
                              capture_output=True, text=True).stdout
    root = ET.fromstring(urdf_xml)
    locked = 0
    for joint in root.iter("joint"):
        if joint.get("name") in UPPER_BODY_JOINTS:
            joint.set("type", "fixed")
            for tag in ["axis", "limit", "dynamics", "calibration",
                        "mimic", "safety_controller"]:
                for child in joint.findall(tag):
                    joint.remove(child)
            locked += 1
    assert locked == len(UPPER_BODY_JOINTS), f"只锁到 {locked} 个上身关节"
    rewritten = 0
    for mesh in root.iter("mesh"):
        fn = mesh.get("filename", "")
        if fn.startswith("package://"):
            mesh.set("filename", str(MESH_DIR / Path(fn).name))
            rewritten += 1
    print(f"locked={locked} meshes_rewritten={rewritten}")
    Path(out_path).write_bytes(ET.tostring(root))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "/tmp/w2_swerve.urdf")
