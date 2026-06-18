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

def se2_align(est_xy, ref_xy):
    """SE2 刚体对齐(旋转+平移,无缩放,Umeyama)est_xy → ref_xy。两者 (N,2) 同序对应。
    返回 (R(2,2), t(2,), est_aligned(N,2))。供轨迹叠加与 SE2-APE。"""
    est = np.asarray(est_xy, float); ref = np.asarray(ref_xy, float)
    mu_e = est.mean(0); mu_r = ref.mean(0)
    E = est - mu_e; R_ = ref - mu_r
    H = E.T @ R_
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, d])
    Rot = Vt.T @ D @ U.T
    t = mu_r - Rot @ mu_e
    return Rot, t, (est @ Rot.T + t)


def ape_se2_percent(traj, truth):
    """SE2 对齐后的 APE%:truth 插值到 traj 时戳,做 Umeyama SE2 刚体对齐后取位置 RMSE,
    归一化到 truth 路径长(与 ape_percent 同口径,但用刚体对齐而非首点对齐)。"""
    tx = np.interp(traj[:, 0], truth[:, 0], truth[:, 1]); ty = np.interp(traj[:, 0], truth[:, 0], truth[:, 2])
    ref = np.column_stack([tx, ty]); est = traj[:, 1:3]
    _, _, est_a = se2_align(est, ref)
    rmse = np.sqrt(np.mean(np.sum((est_a - ref) ** 2, axis=1)))
    return 100.0 * rmse / max(_path_len(ref), 1e-9)


def _local_heading(P, s, i, look):
    """轨迹 P (N,2) 在第 i 点的切线航向:弧长前瞻 look 米找首个 k>i,φ=atan2(dy,dx);无则退 i+1。"""
    k = i + 1
    while k < len(s) and s[k] - s[i] < look:
        k += 1
    if k >= len(P):
        k = min(i + 1, len(P) - 1)
    return float(np.arctan2(P[k, 1] - P[i, 1], P[k, 0] - P[i, 0]))

def _Rneg(phi):
    """R(-φ) = [[cosφ, sinφ],[-sinφ, cosφ]],把世界位移旋入 i 点局部航向系。"""
    c, sn = np.cos(phi), np.sin(phi)
    return np.array([[c, sn], [-sn, c]])

def rpe_segment(traj, truth, seg_len=10.0):
    """分段相对位姿误差(对全局刚体变换不变):沿 truth 每累计 seg_len 米取一段 i→j,
    将两条轨迹各自的段位移分别旋入 i 点本轨迹切线航向定义的局部系后再比差,
    返回相对位移误差 RMSE / seg_len 的百分比(航位推算对长程更公平、且不受激光世界系任意旋转影响)。"""
    tx = np.interp(traj[:, 0], truth[:, 0], truth[:, 1]); ty = np.interp(traj[:, 0], truth[:, 0], truth[:, 2])
    T = np.column_stack([tx, ty]); E = traj[:, 1:3]
    sT = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(T, axis=0), axis=1))])
    sE = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(E, axis=0), axis=1))])
    look = min(1.0, seg_len * 0.2)
    errs = []; j = 0
    for i in range(len(sT)):
        while j < len(sT) and sT[j] - sT[i] < seg_len:
            j += 1
        if j >= len(sT):
            break
        phiT = _local_heading(T, sT, i, look)
        phiE = _local_heading(E, sE, i, look)
        dT_local = _Rneg(phiT) @ (T[j] - T[i])
        dE_local = _Rneg(phiE) @ (E[j] - E[i])
        errs.append(np.linalg.norm(dE_local - dT_local))
    if not errs:
        return 0.0
    return 100.0 * float(np.sqrt(np.mean(np.square(errs)))) / seg_len
