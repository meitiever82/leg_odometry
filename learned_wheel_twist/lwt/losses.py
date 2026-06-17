# lwt/losses.py —— 逐轴加权 MSE(预热)+ 异方差高斯 NLL。w=逐轴权重(1/训练std)。
import torch


def weighted_mse(pred, target, w):
    return (((pred - target) ** 2) * w[None, :]).mean()


def gaussian_nll(pred, target, logvar, w):
    """0.5·mean( w·((e²/σ²) + logσ²) ),e=pred-target,σ²=exp(logvar)。"""
    inv = torch.exp(-logvar)
    return 0.5 * (w[None, :] * (((pred - target) ** 2) * inv + logvar)).mean()


def batch_bias_penalty(pred, target, weight, w):
    """惩罚整个 batch 的逐轴系统性偏差(均值误差),与 weighted_mse/gaussian_nll 同的逐轴权重 w。
    weight·Σ_axis( w_axis · (mean_batch(pred-target))² )。零偏→0,偏差平方→二次增长。"""
    mean_err = (pred - target).mean(dim=0)          # (3,) per-axis bias
    return weight * ((mean_err ** 2) * w).sum()
