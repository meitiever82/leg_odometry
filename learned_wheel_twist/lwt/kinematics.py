# lwt/kinematics.py —— 轮速运动学薄封装:LS twist 先验、twist→SE2 积分、APE/RPE。
import numpy as np

WHEELBASE, TRACK, WHEEL_RADIUS = 0.435, 0.400, 0.075
WHEEL_POS = np.array([[ WHEELBASE/2,  TRACK/2], [ WHEELBASE/2, -TRACK/2],
                      [-WHEELBASE/2,  TRACK/2], [-WHEELBASE/2, -TRACK/2]])

def _A():
    A = np.zeros((8, 3))
    for i, (x, y) in enumerate(WHEEL_POS):
        A[2*i] = [1, 0, -y]; A[2*i+1] = [0, 1, x]
    return A

_PINV = np.linalg.inv(_A().T @ _A()) @ _A().T   # 3x8

def ls_twist(angles, speeds):
    """8×3 LS:转向角[...,4] rad + 轮速[...,4] m/s → twist[...,3] (vx,vy,wz)。支持 batch。"""
    angles = np.asarray(angles, float); speeds = np.asarray(speeds, float)
    b = np.empty(angles.shape[:-1] + (8,))
    b[..., 0::2] = speeds * np.cos(angles)
    b[..., 1::2] = speeds * np.sin(angles)
    out = b @ _PINV.T
    return (out[0], out[1], out[2]) if angles.ndim == 1 else out

def integrate_twist(times, twists):
    """中点法 SE2 积分。times (N,), twists (N,3)=(vx,vy,wz) base 系 → (N,4)=[t,x,y,yaw],首帧原点。"""
    times = np.asarray(times, float); tw = np.asarray(twists, float)
    n = len(times); traj = np.zeros((n, 4)); traj[:, 0] = times
    x = y = yaw = 0.0
    for i in range(1, n):
        dt = times[i] - times[i-1]
        vx, vy, wz = tw[i-1]
        ym = yaw + 0.5*wz*dt
        x += (vx*np.cos(ym) - vy*np.sin(ym))*dt
        y += (vx*np.sin(ym) + vy*np.cos(ym))*dt
        yaw += wz*dt
        traj[i, 1:] = [x, y, yaw]
    return traj

def _path_len(xy):
    return float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))

def ape_percent(traj, truth):
    """traj/truth: (N,3+)[t,x,y,...];truth 插值到 traj 时戳、首点对齐,RMSE/truth路径长 %。"""
    tx = np.interp(traj[:, 0], truth[:, 0], truth[:, 1]) ; ty = np.interp(traj[:, 0], truth[:, 0], truth[:, 2])
    tx, ty = tx - tx[0], ty - ty[0]
    ex, ey = traj[:, 1]-traj[0, 1], traj[:, 2]-traj[0, 2]
    rmse = np.sqrt(np.mean((ex-tx)**2 + (ey-ty)**2))
    return 100.0 * rmse / max(_path_len(np.column_stack([tx, ty])), 1e-9)

def rpe_segment(traj, truth, seg_len=10.0):
    """分段相对位姿误差:沿 truth 每累计 seg_len 米取一段,比 traj 与 truth 在该段的相对位移,
    返回相对位移误差的 RMSE / seg_len 的百分比(航位推算对长程更公平的指标)。"""
    tx = np.interp(traj[:, 0], truth[:, 0], truth[:, 1]); ty = np.interp(traj[:, 0], truth[:, 0], truth[:, 2])
    T = np.column_stack([tx, ty]); E = traj[:, 1:3]
    s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(T, axis=0), axis=1))])
    errs = []; j = 0
    for i in range(len(s)):
        while j < len(s) and s[j] - s[i] < seg_len:
            j += 1
        if j >= len(s):
            break
        dT = T[j] - T[i]; dE = E[j] - E[i]
        errs.append(np.linalg.norm(dE - dT))
    if not errs:
        return 0.0
    return 100.0 * float(np.sqrt(np.mean(np.square(errs)))) / seg_len
