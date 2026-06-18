# lwt/model.py —— 因果膨胀 TCN + 残差(在 LS 先验上)+ 对角协方差头。
import torch, torch.nn as nn

class _TCNBlock(nn.Module):
    def __init__(self, c, k, d, dropout):
        super().__init__()
        pad = (k-1)*d
        self.conv1 = nn.Conv1d(c, c, k, padding=pad, dilation=d)
        self.conv2 = nn.Conv1d(c, c, k, padding=pad, dilation=d)
        self.pad = pad; self.relu = nn.ReLU(); self.drop = nn.Dropout(dropout)
    def _crop(self, y): return y[..., :-self.pad] if self.pad else y
    def forward(self, x):
        y = self.drop(self.relu(self._crop(self.conv1(x))))
        y = self.drop(self.relu(self._crop(self.conv2(y))))
        return x + y

class TwistTCN(nn.Module):
    """因果 TCN twist 估计器,两种工作模式:

    - prior in {ls, imu_wz}(phase-1/2):twist = ls_prior + Δ(head),残差学在解析先验之上。
    - prior == none(phase-3 data-driven):twist = head(features),**直接回归**,不加任何
      LS/运动学先验(ls_prior 被忽略,等价 prior=0)。网络须自己从数据学出 轮速→twist 的几何。

    输入 x (B,in_ch,T);ls_prior (B,3) 仅在 prior!=none 时使用(none 模式下可传 0)。
    输出 twist (B,3),logvar (B,3)。
    """
    def __init__(self, in_ch=8, channels=48, layers=4, kernel=3, dropout=0.1,
                 zero_init_residual=False, prior="ls"):
        super().__init__()
        self.prior = prior
        self.inp = nn.Conv1d(in_ch, channels, 1)
        self.blocks = nn.ModuleList([_TCNBlock(channels, kernel, 2**i, dropout) for i in range(layers)])
        self.head_delta = nn.Linear(channels, 3)
        self.head_logvar = nn.Linear(channels, 3)
        # zero_init 仅对残差模式有意义(让初始 twist≈先验);data-driven 须输出完整 twist,关掉它。
        if zero_init_residual and prior != "none":
            nn.init.zeros_(self.head_delta.weight); nn.init.zeros_(self.head_delta.bias)
    def forward(self, x, ls_prior):
        h = self.inp(x)
        for b in self.blocks: h = b(h)
        feat = h[..., -1]
        out = self.head_delta(feat)
        logvar = self.head_logvar(feat).clamp(-8, 4)
        twist = out if self.prior == "none" else ls_prior + out
        return twist, logvar
