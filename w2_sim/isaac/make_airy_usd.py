#!/usr/bin/env python3
"""isaac/make_airy_usd.py — 生成本地 AIRY OmniLidar USD(config/RS_Airy.usd)。

为什么需要这一步:
  Isaac Sim 5.x 的 RTX lidar 插件在 **sensor 创建时** 解析 OmniLidar prim 的
  omni:sensor:Core:* 配置并缓存;之后再改 prim 属性插件不会重读(实测:用 OS1
  模板创建后覆写 96 线 / 0~90° 仰角,prim 属性确实变了,但渲染时插件仍按模板的
  32 线 / scanType 缓存,报 "NumberOfEmitters 96 != param vector length 32"
  + "Unknown scan type 0" 并崩溃)。
  解决:先用 OS1 内置模板拿到完整合法 schema,把它覆写成 AIRY 规格后 **导出成本地
  USD**,主程序用 add_reference_to_stage 引用这个已烘焙好 AIRY 配置的 prim —— 这样
  插件在创建时直接解析到 AIRY 值,不依赖运行时覆写。

用法:
  $ISAACSIM_DIR/python.sh isaac/make_airy_usd.py [out.usd]
  默认输出: w2_sim/config/RS_Airy.usd
"""
import os
import sys
from pathlib import Path

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import omni.kit.commands  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.stage import get_current_stage  # noqa: E402
from pxr import Sdf  # noqa: E402

enable_extension("isaacsim.sensors.rtx")
app.update()

OUT = os.path.abspath(os.path.expanduser(
    sys.argv[1] if len(sys.argv) > 1
    else str(Path(__file__).resolve().parent.parent / "config" / "RS_Airy.usd")))
os.makedirs(os.path.dirname(OUT), exist_ok=True)

# RoboSense AIRY 规格(手册表1 + 真实 bag 实测)
CH = 96
AZ_STEPS = 900          # 360° / 0.4°
SCAN_HZ = 10
EL_STEP = 0.947         # 0~90° 均布(95×0.947 ≈ 89.97°)
NEAR_M = 0.1
FAR_M = 60.0
ACC_M = 0.015           # 1.5cm 1σ

# 用 "Example_Rotary" 通用旋转式模板创建合法 OmniLidar。
# 关键:必须用 Example_Rotary 而非 Ouster OS*——OS* 的真实 emitter 配置藏在 prim 的
# **payload**(./OS1_*.usda)里,lidar 插件直接读 payload(32 线),flatten 也不拉
# payload,导致覆写无效仍崩(NumberOfEmitters 96 != 32)。Example_Rotary 是纯 USD 属性
# 配置的通用旋转雷达(用 reference,无 payload),prim 本身即 OmniLidar,覆写后 flatten
# 能把 reference 压平成 concrete spec,插件读到完整 96 线 AIRY 配置。
_, prim = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar", path="/RS_Airy", parent=None,
    config="Example_Rotary")


def setv(name, value):
    a = prim.GetAttribute("omni:sensor:Core:" + name)
    assert a and a.IsValid(), f"missing attr {name}"
    assert a.Set(value), f"Set failed for {name}"


el = [i * EL_STEP for i in range(CH)]
zf = [0.0] * CH
zu = [0] * CH
chan = [i + 1 for i in range(CH)]

setv("scanType", "ROTARY")
setv("numberOfChannels", CH)
setv("numberOfEmitters", CH)
setv("scanRateBaseHz", SCAN_HZ)
setv("reportRateBaseHz", SCAN_HZ * AZ_STEPS)   # 每圈方位列 = report/scan = 900
setv("validStartAzimuthDeg", 0.0)
setv("validEndAzimuthDeg", 360.0)
setv("startAzimuthOffsetDeg", 0.0)
setv("nearRangeM", NEAR_M)
setv("farRangeM", FAR_M)
setv("rangeAccuracyM", ACC_M)
setv("maxReturns", 1)                           # 单回波 strongest
setv("minReflectance", 0.1)
# 关键:所有 per-emitter 数组长度都必须 = numberOfEmitters(96),否则插件校验
# "param vector length != NumberOfEmitters" 直接崩。Example_Rotary 模板这些数组原本
# 都是 128 长,逐一重置为 96(elevation 用 AIRY 仰角,其余按模板语义置 0/递增)。
setv("emitterState:s001:elevationDeg", el)
setv("emitterState:s001:azimuthDeg", zf)
setv("emitterState:s001:channelId", chan)
setv("emitterState:s001:fireTimeNs", zu)
setv("emitterState:s001:distanceCorrectionM", zf)
setv("emitterState:s001:focalDistM", zf)
setv("emitterState:s001:focalSlope", zf)
setv("emitterState:s001:horOffsetM", zf)
setv("emitterState:s001:vertOffsetM", zf)
setv("emitterState:s001:reportRateDiv", zf)
prim.GetAttribute("omni:sensor:marketName").Set("AIRY")
prim.GetAttribute("omni:sensor:modelName").Set("RS_AIRY_96ch_10hz_900res")
prim.GetAttribute("omni:sensor:modelVendor").Set("RoboSense")

stage = get_current_stage()
# Example_Rotary 的 prim 本身就是 OmniLidar(/RS_Airy),即引用根(defaultPrim)。
lidar_path = str(prim.GetPath())
root_path = lidar_path
print(f"  lidar={lidar_path} root={root_path} type={prim.GetTypeName()}", flush=True)

# 删除 Render / 默认相机等非传感器 prim(否则引用进主 stage 会带入垃圾)。
for c in list(stage.GetPseudoRoot().GetChildren()):
    cp = str(c.GetPath())
    if cp != root_path and c.GetName() in (
            "Render", "OmniverseKit_Persp", "OmniverseKit_Front",
            "OmniverseKit_Top", "OmniverseKit_Right"):
        stage.RemovePrim(c.GetPath())

# Flatten:把 OS1 reference + 我们的覆写 **压平成 concrete specs**,这样导出的 USD
# 里 AIRY 值是实打实的属性(不再是叠在 OS1 引用上的 override),插件创建时直接解析到。
flat = stage.Flatten()
flat.defaultPrim = Sdf.Path(root_path).name   # flatten 会丢 defaultPrim,显式补回
flat.Export(OUT)
print(f"AIRY_USD_OK -> {OUT}", flush=True)
print(f"  {CH}ch el[{el[0]:.2f},{el[-1]:.2f}]deg az={AZ_STEPS}steps "
      f"report={SCAN_HZ*AZ_STEPS}Hz range[{NEAR_M},{FAR_M}]m acc={ACC_M}m", flush=True)
app.close()
