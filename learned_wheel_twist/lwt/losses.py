# lwt/losses.py —— 逐轴加权 MSE(预热)+ 异方差高斯 NLL。w=逐轴权重(1/训练std)。
import torch


def weighted_mse(pred, target, w):
    return (((pred - target) ** 2) * w[None, :]).mean()


def gaussian_nll(pred, target, logvar, w):
    """0.5·mean( w·((e²/σ²) + logσ²) ),e=pred-target,σ²=exp(logvar)。"""
    inv = torch.exp(-logvar)
    return 0.5 * (w[None, :] * (((pred - target) ** 2) * inv + logvar)).mean()
