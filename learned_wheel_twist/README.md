# learned_wheel_twist

Learned wheel odometry twist estimator using Temporal Convolutional Networks (TCN).

## Overview

This package implements a residual+TCN-based twist (v_x, ω_z) estimator trained on wheel encoder and steering angle data. The core acceptance criterion is that pure wheel-speed estimates approach laser-validated ground truth on real hardware.

## Two-Stage Pipeline

### Stage 1: Data Extraction (System Python 3)
Extract wheel and steering data from rosbag2 recordings using the system Python interpreter.

```bash
python3 scripts/extract.py <rosbag_dir> --output <output.npz>
```

Produces compressed `.npz` containing aligned wheel speeds, steering angles, and ground-truth velocities (if available).

### Stage 2: Training & Evaluation (Torch Environment)
Train and evaluate the TCN model using the dedicated PyTorch environment.

```bash
~/.venv-py312/bin/python -m lwt.train --config config/default.yaml --data <input.npz>
~/.venv-py312/bin/python -m lwt.eval --model <model.pth> --data <test.npz>
~/.venv-py312/bin/python -m lwt.eval_real --model <model.pth> --rosbag <real_rosbag>
```

## Usage

| Command | Purpose |
|---------|---------|
| `extract` | Extract data from rosbag2 → .npz |
| `train` | Train TCN on extracted .npz |
| `eval` | Evaluate on test set (.npz) |
| `eval_real` | Evaluate on real hardware (rosbag) |
| `tensorboard --logdir runs` | View training metrics |

## Configuration

Default configuration in `config/default.yaml` covers:
- **Data**: Window size (25 frames @ 50 Hz = 0.5 s), dataset paths
- **Split**: Train/val/test ratios (0.8/0.1/0.1)
- **Augmentation**: Stochastic steering bias and speed scale per sample
- **Model**: TCN architecture (48 channels, 4 layers, kernel=3)
- **Training**: Batch size 256, learning rate 1e-3, 60 epochs with MSE warmup
- **Geometry**: Wheelbase 0.435 m, track 0.400 m, wheel radius 0.075 m

## Two Python Interpreters

- **System `/usr/bin/python3`**: Rosbag2 extraction (ROS2 stack)
- **Torch `/root/.venv-py312/bin/python`**: Training & inference (PyTorch, NumPy, scikit-learn)

Keep these separate to avoid ABI conflicts between ROS2 system libraries and conda PyTorch.
