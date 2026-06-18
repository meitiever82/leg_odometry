#!/usr/bin/env python
# lwt/eval.py —— [venv torch] 仿真测集:LS / 模型 三方对照。
# twist RMSE + 积分 APE/RPE + 不确定度校准。
# 用法: ~/rtabmap_ws/.venv-py312/bin/python lwt/eval.py runs/lwt_wheel.pt
import argparse, sys, numpy as np, torch
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from lwt.dataset import TwistDataset
from lwt.model import TwistTCN
from lwt.kinematics import ls_twist, integrate_twist, ape_percent, rpe_segment

def eval_episode(npz, m, norm, in_ch, W, dev, imu_wz_prior=False, data_driven=False, wz_anchor=False):
    d = np.load(npz); t = d["t_wheel"]
    ds = TwistDataset([npz], W, augment=False, with_imu=(in_ch == 9), norm=norm,
                      imu_wz_prior=imu_wz_prior, data_driven=data_driven, wz_anchor=wz_anchor)
    tw_gt = np.stack([d["gt_vx"], d["gt_vy"], d["gt_wz"]], 1)[W-1:]
    tt = t[W-1:]
    ls = ls_twist(d["steer"][W-1:], d["speed"][W-1:])
    preds, lvs = [], []
    with torch.no_grad():
        for i in range(len(ds)):
            x, _, lsp = ds[i]
            twp, lv = m(x[None].to(dev), lsp[None].to(dev)); preds.append(twp[0].cpu().numpy()); lvs.append(lv[0].cpu().numpy())
    pred = np.array(preds); sigma = np.exp(0.5*np.array(lvs))
    rmse = lambda a: np.sqrt(((a - tw_gt)**2).mean(0))
    cover = (np.abs(pred - tw_gt) < sigma).mean(0)
    truth_traj = integrate_twist(tt, tw_gt)
    def metr(tw):
        tr = integrate_twist(tt, tw)
        return ape_percent(tr, truth_traj[:, :3]), rpe_segment(tr, truth_traj[:, :3], 10.0)
    return dict(rmse_ls=rmse(ls), rmse_pred=rmse(pred), cover=cover,
                ape_ls=metr(ls)[0], ape_pred=metr(pred)[0],
                rpe_ls=metr(ls)[1], rpe_pred=metr(pred)[1])

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("ckpt"); a = ap.parse_args()
    ck = torch.load(a.ckpt, weights_only=False); dev = "cuda" if torch.cuda.is_available() else "cpu"
    prior = ck["cfg"]["model"].get("prior", "ls")
    m = TwistTCN(ck["in_ch"], ck["cfg"]["model"]["tcn_channels"], ck["cfg"]["model"]["tcn_layers"],
                 ck["cfg"]["model"]["kernel"], prior=prior).to(dev); m.load_state_dict(ck["model"]); m.eval()
    W = ck["cfg"]["data"]["window"]
    iwp = ck["cfg"]["model"].get("imu_wz_prior", False)
    dd = ck["cfg"]["model"].get("data_driven", False)
    wza = ck["cfg"]["model"].get("wz_anchor", False)
    if iwp: print("  [imu_wz_prior] 残差先验 wz = imu_yawrate")
    if dd: print("  [data-driven] prior='none' 直接回归(14ch raw 轮速+base IMU,无 LS 先验)")
    if wza: print("  [wz-anchor] 先验=[0,0,imu_yawrate],wz=陀螺+残差,vx/vy 从 raw 学(14ch,无 LS)")
    rows = [eval_episode(p, m, ck["norm"], ck["in_ch"], W, dev, iwp, dd, wza) for p, _ in ck["split"]["test"]]
    agg = lambda k: np.mean([r[k] for r in rows], 0)
    print("=== 仿真测集(%d ep)===" % len(rows))
    print(f"  twist RMSE   LS  vx/vy/wz = {agg('rmse_ls')}")
    print(f"  twist RMSE  模型 vx/vy/wz = {agg('rmse_pred')}")
    print(f"  积分 APE%%   LS={np.mean([r['ape_ls'] for r in rows]):.2f}  模型={np.mean([r['ape_pred'] for r in rows]):.2f}")
    print(f"  积分 RPE%%   LS={np.mean([r['rpe_ls'] for r in rows]):.2f}  模型={np.mean([r['rpe_pred'] for r in rows]):.2f}")
    print(f"  不确定度覆盖率(理想~0.68) vx/vy/wz = {agg('cover')}")
    ok = agg('rmse_pred').sum() < agg('rmse_ls').sum() and np.mean([r['ape_pred'] for r in rows]) < np.mean([r['ape_ls'] for r in rows])
    print("门槛(模型胜原始LS):", "PASS" if ok else "FAIL")

if __name__ == "__main__":
    main()
