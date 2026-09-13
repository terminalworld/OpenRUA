"""Analytic Panda FK (modified DH) + numeric IK with posture regularization. World frame."""
import math
import numpy as np
from scipy.optimize import least_squares

BASE_T = np.eye(4); BASE_T[:3, 3] = [-0.66, 0.0, 0.912]   # world -> panda_link0
DH = [  # (a, d, alpha) modified DH, Craig convention, per joint i
    (0.0, 0.333, 0.0),
    (0.0, 0.0, -math.pi / 2),
    (0.0, 0.316, math.pi / 2),
    (0.0825, 0.0, math.pi / 2),
    (-0.0825, 0.384, -math.pi / 2),
    (0.0, 0.0, math.pi / 2),
    (0.088, 0.0, math.pi / 2),
]
FLANGE = (0.0, 0.107, 0.0)
HAND_ROT = -math.pi / 4  # panda_link8 -> panda_hand about z
LIMITS = np.array([[-2.9, 2.9], [-1.76, 1.76], [-2.9, 2.9], [-3.07, -0.07], [-2.9, 2.9], [-0.02, 3.75], [-2.9, 2.9]])


def _tf(a, d, alpha, theta):
    ca, sa, ct, st = math.cos(alpha), math.sin(alpha), math.cos(theta), math.sin(theta)
    return np.array([
        [ct, -st, 0, a],
        [st * ca, ct * ca, -sa, -sa * d],
        [st * sa, ct * sa, ca, ca * d],
        [0, 0, 0, 1]])


def fk_all(q):
    """Return list of 4x4 world transforms: link1..link7, link8(flange), hand."""
    T = BASE_T.copy()
    out = []
    for (a, d, al), th in zip(DH, q):
        T = T @ _tf(a, d, al, th)
        out.append(T)
    T = T @ _tf(*FLANGE, 0.0)
    out.append(T)
    T = T @ _tf(0, 0, 0, HAND_ROT)
    out.append(T)
    return out


def fk_hand(q):
    return fk_all(q)[-1]


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def ik(pos, quat, seed, w_post=0.02, tol=1e-4, tries=8):
    """Numeric IK: hand frame at pos (world) with orientation quat. Returns q or None."""
    pos = np.asarray(pos, float); Rt = quat_to_R(quat)
    seed = np.asarray(seed, float)

    def resid(q):
        T = fk_hand(q)
        ep = (T[:3, 3] - pos) * 10.0
        eo = (T[:3, :3] - Rt).ravel() * 2.0
        return np.concatenate([ep, eo, w_post * (q - seed)])

    best = None
    rng = np.random.default_rng(0)
    for k in range(tries):
        q0 = seed if k == 0 else np.clip(seed + rng.normal(0, 0.3 * k / tries + 0.1, 7), LIMITS[:, 0], LIMITS[:, 1])
        r = least_squares(resid, q0, bounds=(LIMITS[:, 0], LIMITS[:, 1]), xtol=1e-10, ftol=1e-10, max_nfev=2000)
        # polish: drop the posture term so the pose is met exactly
        wp, w_post = w_post, 1e-4
        r = least_squares(resid, r.x, bounds=(LIMITS[:, 0], LIMITS[:, 1]), xtol=1e-12, ftol=1e-12, max_nfev=2000)
        w_post = wp
        T = fk_hand(r.x)
        perr = np.linalg.norm(T[:3, 3] - pos)
        Re = Rt.T @ T[:3, :3]
        oerr = math.acos(max(-1, min(1, (np.trace(Re) - 1) / 2)))
        if perr < tol and oerr < 6e-3:
            return r.x
        if best is None or r.cost < best[0]:
            best = (r.cost, r.x, perr, oerr)
    print(f"  ik: no exact solution (best perr={best[2]:.4f} oerr={best[3]:.4f})")
    return None


def min_link_z(q):
    return min(T[2, 3] for T in fk_all(q))
