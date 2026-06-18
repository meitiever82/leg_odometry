# lwt/dataset.py —— .npz → 滑窗样本、按episode分层划分、κ增广、torch Dataset。
import numpy as np
import torch
from lwt.kinematics import ls_twist
from lwt.realism import RealismAug

def build_features(d, with_imu=False, data_driven=False, wz_anchor=False):
    """从 .npz 字典构造逐帧特征矩阵 F (N,C),通道布局:
    - 默认(phase-1):C=8  = steer4 + speed4。
    - with_imu(phase-2):C=9  = steer4 + speed4 + imu_yawrate1(重力投影 base yaw-rate)。
    - data_driven(phase-3):C=14 = steer4 + speed4 + imu_g_base3 + imu_a_base3。
      纯 raw 轮速 + 完整 base-frame IMU,**无任何 LS 特征**,网络从数据学运动学。
    - wz_anchor(phase-4):C=14,与 data_driven 同通道(raw 轮速+base IMU,无 LS 特征)。
      区别只在先验向量(见 TwistDataset):prior=[0,0,imu_yawrate(窗末)]。
    data_driven/wz_anchor 与 with_imu 互斥(data_driven/wz_anchor 优先)。"""
    steer, speed = d["steer"], d["speed"]
    feat = [steer, speed]
    if data_driven or wz_anchor:
        for k in ("imu_g_base", "imu_a_base"):
            if k not in (d.files if hasattr(d, "files") else d):
                raise KeyError(f"缺 {k}(需用 phase-3 extract.py 重新抽取 imu_g_base/imu_a_base)")
        feat += [d["imu_g_base"], d["imu_a_base"]]
    elif with_imu:
        if "imu_yawrate" not in (d.files if hasattr(d, "files") else d):
            raise KeyError("缺 imu_yawrate(需用 phase-2 extract.py 重新抽取)")
        feat += [d["imu_yawrate"]]
    return np.concatenate(feat, axis=1)


def make_windows(npz_path, window=25, with_imu=False, data_driven=False, wz_anchor=False):
    """一个 episode .npz → (X, Y, YR)。X (M,window,C),C 见 build_features;
    Y (M,3)=窗末真值 twist;YR (M,)=窗末 imu_yawrate(供 wz_anchor 先验,未标准化)。M = N-window+1。"""
    d = np.load(npz_path)
    F = build_features(d, with_imu, data_driven, wz_anchor)
    N = len(F); M = N - window + 1
    if M <= 0:
        return np.zeros((0, window, F.shape[1])), np.zeros((0, 3)), np.zeros((0,))
    X = np.stack([F[i:i+window] for i in range(M)], 0)
    gt = np.stack([d["gt_vx"], d["gt_vy"], d["gt_wz"]], 1)
    Y = gt[window-1:]
    keys = d.files if hasattr(d, "files") else d
    if "imu_yawrate" in keys:
        yawrate = np.asarray(d["imu_yawrate"]).reshape(-1)  # (N,1)->(N,) 重力投影 base yaw-rate
        YR = yawrate[window-1:]
    else:
        YR = np.zeros(M)   # wz_anchor 才需要;无 IMU 的旧 mock 退化为 0
    return X.astype(np.float32), Y.astype(np.float32), YR.astype(np.float32)

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


def make_segments(npz_path, window=25, segment_sec=10.0, wheel_hz=50,
                  data_driven=True, stride_sec=None):
    """一个 episode .npz → 连续 L 秒分段列表,供 approach-2 轨迹损失。

    每段含 S=round(segment_sec*wheel_hz) 个"输出帧",每个输出帧消费它自己的 W 窗
    (重叠)。第一个输出帧的窗末位于全局索引 window-1,故段需要 (window-1)+S 个原始帧。

    返回 segs: list,每元素 dict:
      Xseg (S, window, C)  每输出帧的输入窗(C 见 build_features)
      tseg (S,)            该段输出帧时间戳(原点未平移,积分器自处理)
      Gseg (S, 3)          该段输出帧 GT twist (vx,vy,wz)
      YRseg (S,)           窗末 imu_yawrate(机理检查/可选先验)
    分段步长 stride_sec 默认 = segment_sec(不重叠);设小可增样本数。"""
    d = np.load(npz_path)
    F = build_features(d, with_imu=False, data_driven=data_driven, wz_anchor=False).astype(np.float32)
    t = np.asarray(d["t_wheel"], float)
    gt = np.stack([d["gt_vx"], d["gt_vy"], d["gt_wz"]], 1).astype(np.float32)
    keys = d.files if hasattr(d, "files") else d
    yawrate = (np.asarray(d["imu_yawrate"]).reshape(-1).astype(np.float32)
               if "imu_yawrate" in keys else np.zeros(len(F), np.float32))
    N = len(F)
    S = int(round(segment_sec * wheel_hz))
    if N < (window - 1) + S:
        return []
    stride = int(round((stride_sec if stride_sec else segment_sec) * wheel_hz))
    stride = max(1, stride)
    segs = []
    # out0 = 第一个输出帧在原始序列中的窗末索引,从 window-1 开始,步进 stride。
    last_out0 = N - S
    for out0 in range(window - 1, last_out0 + 1, stride):
        idx = np.arange(out0, out0 + S)               # 各输出帧窗末全局索引 (S,)
        Xseg = np.stack([F[j - window + 1: j + 1] for j in idx], 0)  # (S,window,C)
        segs.append(dict(X=Xseg.astype(np.float32), t=t[idx].astype(np.float64),
                         G=gt[idx], YR=yawrate[idx]))
    return segs


class SegmentDataset(torch.utils.data.Dataset):
    """连续 L 秒分段数据集(approach-2 轨迹损失)。每项返回一段:
      x_seg  (S, C, window)  逐输出帧输入窗(已标准化,C×T 排布,模型可直接 roll)
      t_seg  (S,)            时间戳(float64)
      g_seg  (S, 3)          GT twist
      yr_seg (S,)            窗末 imu_yawrate
    norm 复用 per-frame 训练集统计(同模型同输入分布)。data_driven 模式 14ch。"""
    def __init__(self, npz_paths, window=25, segment_sec=10.0, wheel_hz=50,
                 norm=None, data_driven=True, stride_sec=None, realism=None, seed=0,
                 wheel_noise=None):
        self.window, self.norm = window, norm
        # 阶段六 Step A:仅轮速通道高斯噪声(IMU 不动)。与 realism 正交,可单独启用。
        self.wheel_noise = (wheel_noise if (wheel_noise and wheel_noise.get("enable")) else None)
        # approach-2 step2a: sim 保真增广(仅 data_driven 14ch)。per-segment 抽一组常值
        # κ-bias / 陀螺残差偏置(模拟该 episode 固定标定/对齐状态),保证轨迹损失看到一致偏置;
        # 白噪声/bumps 仍逐输出帧窗随机施加。realism=None 或 enable=False 则不增广。
        self.realism = (realism if (realism and realism.get("enable") and data_driven) else None)
        self.rng = np.random.default_rng(seed)
        self.segs = []
        for p in npz_paths:
            self.segs += make_segments(p, window, segment_sec, wheel_hz,
                                       data_driven=data_driven, stride_sec=stride_sec)

    def __len__(self):
        return len(self.segs)

    def _augment_segment(self, X):
        """对一段 (S,window,C) 施加 realism:per-segment 常值 κ-bias + 陀螺残差偏置(标定/对齐状态),
        per-window 随机白噪声 + bumps。bumps 逐输出帧窗独立(瞬态不需跨窗一致)。"""
        from lwt.realism import (apply_kappa_bulk, apply_gyro_realism,
                                 apply_speed_bumps, apply_accel_noise, G0)
        c = self.realism
        kb, gy, bp, an = (c.get("kappa_bulk", {}), c.get("gyro", {}),
                          c.get("bumps", {}), c.get("accel_noise", {}))
        # --- per-segment 常值:κ-bias(左右轮速差分)+ 陀螺三轴残差偏置 ---
        if kb.get("enable"):
            apply_kappa_bulk(X.reshape(-1, X.shape[-1]), True, kb.get("kappa_std", 0.04), self.rng)
        if gy.get("enable") and gy.get("sigma_bias", 0.0) > 0:
            b = self.rng.normal(0.0, gy["sigma_bias"], 3).astype(X.dtype)
            X[:, :, G0:G0 + 3] += b[None, None, :]
        # --- per-window:bumps → 陀螺白噪声 → 加速度白噪声 ---
        for j in range(X.shape[0]):
            w = X[j]
            apply_speed_bumps(w, enable=bp.get("enable", False), event_rate=bp.get("event_rate", 0.03),
                              speed_blip=bp.get("speed_blip", 0.05), accel_z=bp.get("accel_z", 2.0),
                              pitchroll_rate=bp.get("pitchroll_rate", 0.05),
                              event_len_min=bp.get("event_len_min", 3),
                              event_len_max=bp.get("event_len_max", 10), rng=self.rng)
            apply_gyro_realism(w, enable=gy.get("enable", False), sigma_bias=0.0,
                               sigma_noise=gy.get("sigma_noise", 0.005), rng=self.rng)
            apply_accel_noise(w, enable=an.get("enable", False), sigma=an.get("sigma", 0.05), rng=self.rng)
        return X

    def __getitem__(self, i):
        s = self.segs[i]
        X = s["X"].copy()                              # (S,window,C)
        if self.realism is not None:
            X = self._augment_segment(X)
        if self.wheel_noise is not None:
            from lwt.realism import apply_wheel_noise
            wn = self.wheel_noise
            for j in range(X.shape[0]):
                apply_wheel_noise(X[j], enable=True, speed_std=wn.get("speed_std", 0.01),
                                  steer_std_deg=wn.get("steer_std_deg", 0.3), rng=self.rng)
        if self.norm is not None:
            X = (X - self.norm[0]) / self.norm[1]
        x = torch.from_numpy(np.transpose(X, (0, 2, 1)).copy()).float()  # (S,C,window)
        return (x, torch.from_numpy(s["t"]).double(),
                torch.from_numpy(s["G"]).float(), torch.from_numpy(s["YR"]).float())


class TwistDataset(torch.utils.data.Dataset):
    """整合:多 episode 滑窗 + 可选 κ增广 + LS先验 + (C,T) 排布。返回 (x[C,T], y[3], ls_prior[3])。
    增广在 __getitem__ 内对原始窗逐样本随机施加(每次不同),真值 y 不变。"""
    def __init__(self, npz_paths, window=25, augment=False, with_imu=False,
                 steer_bias_max_deg=1.0, speed_scale_std=0.015, norm=None, seed=0,
                 imu_wz_prior=False, data_driven=False, wz_anchor=False, realism=None,
                 wheel_noise=None):
        self.window, self.augment, self.with_imu = window, augment, with_imu
        # 阶段六 Step A:仅轮速通道高斯噪声(IMU 不动),与 realism/κ-aug 正交。
        self.wheel_noise = (wheel_noise if (wheel_noise and wheel_noise.get("enable")) else None)
        # approach-2 step2a: 仅 data_driven 14ch 启用 sim 保真增广(取代 κ-aug,因 κ-bulk 已含轮速尺度)。
        self._realism = RealismAug(realism, np.random.default_rng(seed + 7)) \
            if (realism and realism.get("enable") and data_driven) else None
        # phase-4: wz_anchor → 14ch(同 data_driven 特征),但先验=[0,0,imu_yawrate(窗末)]。
        # vx/vy 从 raw 学(先验 0),wz=去偏陀螺+残差。与 data_driven 互斥(若同传以 data_driven 优先)。
        self.wz_anchor = wz_anchor and not data_driven
        # phase-3: data_driven → 14ch raw 轮速+base IMU,无 LS 先验(ls_prior 返回 0)。
        self.data_driven = data_driven
        # phase-2b: imu_wz_prior → 残差先验的 wz 分量改用窗末 imu_yawrate(无偏陀螺),
        # 取代 κ-偏置的轮速 LS wz;vx/vy 仍用 LS。仅在 with_imu(in_ch=9)时生效。
        self.imu_wz_prior = imu_wz_prior and with_imu and not data_driven and not self.wz_anchor
        self.sbm, self.sss = steer_bias_max_deg, speed_scale_std
        self.X, self.Y, self.YR = [], [], []
        for p in npz_paths:
            X, Y, YR = make_windows(p, window, with_imu, data_driven, self.wz_anchor)
            if len(X): self.X.append(X); self.Y.append(Y); self.YR.append(YR)
        self.X = np.concatenate(self.X, 0); self.Y = np.concatenate(self.Y, 0)
        self.YR = np.concatenate(self.YR, 0)
        self.norm = norm
        self.rng = np.random.default_rng(seed)

    def fit_norm(self):
        """用本集(不增广)统计 per-channel mean/std,供 train/val/test 共用。"""
        flat = self.X.reshape(-1, self.X.shape[-1])
        self.norm = (flat.mean(0), flat.std(0) + 1e-6); return self.norm

    def __len__(self): return len(self.X)

    def __getitem__(self, i):
        w = self.X[i].copy()
        if self._realism is not None:
            # sim 保真增广(含 κ-bulk 轮速尺度);取代旧 κ-aug。标签 Y 不变。
            w = self._realism(w)
        elif self.augment:
            w = augment_kappa(w, self.sbm, self.sss, self.rng)
        if self.wheel_noise is not None:
            from lwt.realism import apply_wheel_noise
            wn = self.wheel_noise
            apply_wheel_noise(w, enable=True, speed_std=wn.get("speed_std", 0.01),
                              steer_std_deg=wn.get("steer_std_deg", 0.3), rng=self.rng)
        if self.wz_anchor:
            # phase-4 wz_anchor:先验 = [0, 0, imu_yawrate(窗末,未标准化)]。
            # vx/vy 先验 0(从 raw 学),wz 锚定去偏陀螺;模型走 ls_prior+残差。
            ls = np.array([0.0, 0.0, float(self.YR[i])], dtype=np.float64)
        elif self.data_driven:
            # 纯 data-driven:无 LS 先验,ls_prior=0(模型 prior='none' 会忽略它)。
            # κ 增广仍施加于轮速通道作为正则,但不构造 LS 特征/先验。
            ls = np.zeros(3, dtype=np.float64)
        else:
            ls = np.array(ls_twist(w[-1, :4], w[-1, 4:8]))
            if self.imu_wz_prior:
                ls[2] = w[-1, 8]   # 窗末 imu_yawrate(第 9 通道,未标准化)取代 LS wz
        if self.norm is not None:
            w = (w - self.norm[0]) / self.norm[1]
        x = torch.from_numpy(w.T.copy()).float()
        return x, torch.from_numpy(self.Y[i]).float(), torch.from_numpy(ls).float()
