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
    """输入 (B,in_ch,T) + LS先验 (B,3) → twist (B,3)=LS+Δ, logvar (B,3)。"""
    def __init__(self, in_ch=8, channels=48, layers=4, kernel=3, dropout=0.1, zero_init_residual=False):
        super().__init__()
        self.inp = nn.Conv1d(in_ch, channels, 1)
        self.blocks = nn.ModuleList([_TCNBlock(channels, kernel, 2**i, dropout) for i in range(layers)])
        self.head_delta = nn.Linear(channels, 3)
        self.head_logvar = nn.Linear(channels, 3)
        if zero_init_residual:
            nn.init.zeros_(self.head_delta.weight); nn.init.zeros_(self.head_delta.bias)
    def forward(self, x, ls_prior):
        h = self.inp(x)
        for b in self.blocks: h = b(h)
        feat = h[..., -1]
        delta = self.head_delta(feat)
        logvar = self.head_logvar(feat).clamp(-8, 4)
        return ls_prior + delta, logvar
