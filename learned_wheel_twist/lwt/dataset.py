# lwt/dataset.py —— .npz → 滑窗样本、按episode分层划分、κ增广、torch Dataset。
import numpy as np
import torch
from lwt.kinematics import ls_twist

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

def augment_kappa(window, steer_bias_max_deg=1.0, speed_scale_std=0.015, rng=None):
    """对一个窗(window,C>=8;[:,:4]=转向角rad,[:,4:8]=轮速)叠加额外标定偏差,
    使有效 κ 覆盖真机量级。全窗同一组 (δ,s)(模拟本episode固定标定状态)。真值 twist 不变(调用方不动 Y)。"""
    rng = rng or np.random.default_rng()
    dlt = rng.uniform(-1, 1, 4) * np.radians(steer_bias_max_deg)
    scl = rng.normal(1.0, speed_scale_std, 4)
    window[:, :4] = window[:, :4] + dlt[None, :]
    window[:, 4:8] = window[:, 4:8] * scl[None, :]
    return window


class TwistDataset(torch.utils.data.Dataset):
    """整合:多 episode 滑窗 + 可选 κ增广 + LS先验 + (C,T) 排布。返回 (x[C,T], y[3], ls_prior[3])。
    增广在 __getitem__ 内对原始窗逐样本随机施加(每次不同),真值 y 不变。"""
    def __init__(self, npz_paths, window=25, augment=False, with_imu=False,
                 steer_bias_max_deg=1.0, speed_scale_std=0.015, norm=None, seed=0):
        self.window, self.augment, self.with_imu = window, augment, with_imu
        self.sbm, self.sss = steer_bias_max_deg, speed_scale_std
        self.X, self.Y = [], []
        for p in npz_paths:
            X, Y = make_windows(p, window, with_imu)
            if len(X): self.X.append(X); self.Y.append(Y)
        self.X = np.concatenate(self.X, 0); self.Y = np.concatenate(self.Y, 0)
        self.norm = norm
        self.rng = np.random.default_rng(seed)

    def fit_norm(self):
        """用本集(不增广)统计 per-channel mean/std,供 train/val/test 共用。"""
        flat = self.X.reshape(-1, self.X.shape[-1])
        self.norm = (flat.mean(0), flat.std(0) + 1e-6); return self.norm

    def __len__(self): return len(self.X)

    def __getitem__(self, i):
        w = self.X[i].copy()
        if self.augment:
            w = augment_kappa(w, self.sbm, self.sss, self.rng)
        ls = np.array(ls_twist(w[-1, :4], w[-1, 4:8]))
        if self.norm is not None:
            w = (w - self.norm[0]) / self.norm[1]
        x = torch.from_numpy(w.T.copy()).float()
        return x, torch.from_numpy(self.Y[i]).float(), torch.from_numpy(ls).float()
