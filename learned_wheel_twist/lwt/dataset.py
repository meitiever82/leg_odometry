# lwt/dataset.py —— .npz → 滑窗样本、按episode分层划分、κ增广、torch Dataset。
import numpy as np

def make_windows(npz_path, window=25, with_imu=False):
    """一个 episode .npz → (X, Y)。X (M,window,C):C=8(steer4+speed4)或14(+imu_g3+imu_a3);
    Y (M,3)=窗末真值 twist。M = N-window+1。"""
    d = np.load(npz_path)
    steer, speed = d["steer"], d["speed"]
    feat = [steer, speed]
    if with_imu and "imu_g" in d:
        feat += [d["imu_g"], d["imu_a"]]
    F = np.concatenate(feat, axis=1)
    N = len(F); M = N - window + 1
    if M <= 0:
        return np.zeros((0, window, F.shape[1])), np.zeros((0, 3))
    X = np.stack([F[i:i+window] for i in range(M)], 0)
    gt = np.stack([d["gt_vx"], d["gt_vy"], d["gt_wz"]], 1)
    Y = gt[window-1:]
    return X.astype(np.float32), Y.astype(np.float32)

def stratified_split(episodes, ratios=(0.8, 0.1, 0.1), seed=0):
    """episodes: [(name, kappa), ...] → (train, val, test),按 kappa 排序后分层轮转,防 episode 跨集泄漏。"""
    rng = np.random.default_rng(seed)
    eps = sorted(episodes, key=lambda x: x[1])
    tr, va, te = [], [], []
    for i in range(0, len(eps), 10):
        grp = eps[i:i+10]; rng.shuffle(grp)
        ntr = max(1, round(len(grp)*ratios[0])); nva = max(0, round(len(grp)*ratios[1]))
        tr += grp[:ntr]; va += grp[ntr:ntr+nva]; te += grp[ntr+nva:]
    return tr, va, te
