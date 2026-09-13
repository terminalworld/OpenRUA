"""Analytic Panda FK (modified DH, checked against /compute_fk) and a
seed-regularised 6-DoF numeric IK, so straight-line paths stay continuous
in joint space (the machine's /compute_ik is position-only and wanders
through the null space between nearby targets)."""
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot

# (a, d, alpha) per joint, then flange; Franka's published parameters
DH = [(0, 0.333, 0), (0, 0, -np.pi / 2), (0, 0.316, np.pi / 2),
      (0.0825, 0, np.pi / 2), (-0.0825, 0.384, -np.pi / 2),
      (0, 0, np.pi / 2), (0.088, 0, np.pi / 2)]
FLANGE_D = 0.107
HAND_YAW = -np.pi / 4          # panda_link8 -> panda_hand (tf_static)
BASE_T = np.array([-0.51, 0.0, 0.42])  # world -> panda_link0 (tf)
LIMITS = np.array([[-2.9, 2.9], [-1.76, 1.76], [-2.9, 2.9], [-3.07, -0.07],
                   [-2.9, 2.9], [-0.02, 3.75], [-2.9, 2.9]])
TCP = 0.1034


def _mdh(a, d, alpha, th):
    ca, sa, ct, st = np.cos(alpha), np.sin(alpha), np.cos(th), np.sin(th)
    return np.array([[ct, -st, 0, a],
                     [st * ca, ct * ca, -sa, -d * sa],
                     [st * sa, ct * sa, ca, d * ca],
                     [0, 0, 0, 1]])


def fk_hand(q):
    """4x4 world pose of panda_hand for 7 joint angles."""
    T = np.eye(4)
    T[:3, 3] = BASE_T
    for (a, d, al), th in zip(DH, q):
        T = T @ _mdh(a, d, al, th)
    T = T @ _mdh(0, FLANGE_D, 0, HAND_YAW)
    return T


def fk_tcp(q):
    T = fk_hand(q)
    return T[:3, 3] + TCP * T[:3, 2]


def target_R(yaw_deg):
    """Hand rotation: z down, fingers (hand y) along world y when yaw=0."""
    return Rot.from_euler("z", yaw_deg, degrees=True).as_matrix() @ np.diag([1.0, -1.0, -1.0])


def ik(pos_tcp, yaw_deg, seed, reg=0.02, pos_w=1000.0, rot_w=100.0):
    """Joints placing the TCP at world pos with a top-down hand at yaw.
    Regularised toward the seed to pick the nearest branch."""
    Rt = target_R(yaw_deg)
    pos_tcp = np.asarray(pos_tcp, float)
    seed = np.asarray(seed, float)

    def resid(q):
        T = fk_hand(q)
        p = T[:3, 3] + TCP * T[:3, 2]
        dR = Rot.from_matrix(T[:3, :3] @ Rt.T).as_rotvec()
        return np.concatenate([pos_w * (p - pos_tcp), rot_w * dR, reg * (q - seed)])

    sol = least_squares(resid, seed, bounds=(LIMITS[:, 0], LIMITS[:, 1]),
                        xtol=1e-10, ftol=1e-10, gtol=1e-10, max_nfev=2000)
    q = sol.x
    T = fk_hand(q)
    p = T[:3, 3] + TCP * T[:3, 2]
    perr = np.linalg.norm(p - pos_tcp)
    rerr = np.degrees(np.linalg.norm(Rot.from_matrix(T[:3, :3] @ Rt.T).as_rotvec()))
    if perr > 0.002 or rerr > 1.0:
        return None, (perr, rerr)
    return q, (perr, rerr)
