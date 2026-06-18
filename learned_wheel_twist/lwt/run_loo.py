#!/usr/bin/env python
# lwt/run_loo.py —— 阶段六 Step C 编排:跑 3 折 fine-tune + held-out 评估 + 训练集 APE(过拟合检查)。
# 收集 held-out(headline)与 train-bag APE,打印对照表 + JSON,供 test_report 填表。
# 用法: ~/.venv-py312/bin/python lwt/run_loo.py --pretrain runs/lwt_pre_wn.pt --runs-dir runs
import argparse, sys, json, subprocess, re, numpy as np
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from lwt.finetune_real import REAL_BAGS, FOLDS

PY = sys.executable
ROOT = Path(__file__).resolve().parent.parent.parent   # other_odometry/


def eval_bag(ckpt, bag):
    npz, tum = REAL_BAGS[bag]
    out = subprocess.run([PY, str(Path(__file__).parent / "eval_real.py"), ckpt, npz, tum],
                         capture_output=True, text=True).stdout
    rj = re.search(r"RESULT_JSON (\{.*\})", out)
    rb = re.search(r"RESULT_BIAS (\{.*\})", out)
    j = json.loads(rj.group(1)) if rj else {}
    b = json.loads(rb.group(1)) if rb else {}
    j.update(b)
    return j


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pretrain", required=True)
    ap.add_argument("--runs-dir", default="learned_wheel_twist/runs")
    ap.add_argument("--epochs", type=int, default=None)
    ap.add_argument("--skip-train", action="store_true")
    a = ap.parse_args()
    rd = Path(a.runs_dir)

    results = {}
    for fold in ["A", "B", "C"]:
        out = str(rd / f"lwt_ft_fold{fold}")
        if not a.skip_train:
            cmd = [PY, str(Path(__file__).parent / "finetune_real.py"),
                   "--pretrain", a.pretrain, "--fold", fold, "--out", out]
            if a.epochs: cmd += ["--epochs", str(a.epochs)]
            print(f"\n##### TRAIN fold {fold} #####")
            subprocess.run(cmd, check=True)
        fdef = FOLDS[fold]
        held = fdef["test"]
        ft_ckpt = out + ".pt"
        # held-out:fine-tuned + pretrained(no-FT)
        ho_ft = eval_bag(ft_ckpt, held)
        ho_pre = eval_bag(a.pretrain, held)
        # train-bag APE(过拟合检查):用 fine-tuned 在 2 个训练 bag 上评。
        tr_ape = {b: eval_bag(ft_ckpt, b)["model"][2] for b in fdef["train"]}
        results[fold] = {"held": held, "ho_ft": ho_ft, "ho_pre": ho_pre, "tr_ape": tr_ape}
        print(f"\n=== fold {fold} held-out {held} ===")
        print(f"  FT     RPE@10/@50/APE = {ho_ft['model']}  wz_bias={ho_ft.get('wz_bias')} wheel_scale={ho_ft.get('wheel_scale')}")
        print(f"  preFT  RPE@10/@50/APE = {ho_pre['model']} wz_bias={ho_pre.get('wz_bias')} wheel_scale={ho_pre.get('wheel_scale')}")
        print(f"  LS     RPE@10/@50/APE = {ho_ft['ls']}")
        print(f"  hybrid(model-vxvy) = {ho_ft['hybrid']}")
        print(f"  train-bag APE (overfit check): {tr_ape}  held-APE={ho_ft['model'][2]}")

    print("\n\nLOO_RESULTS_JSON " + json.dumps(results))


if __name__ == "__main__":
    main()
