"""Local kinematics for the Panda (from robot.urdf), in the WORLD frame.

fk(q)            -> dict link -> (pos, R) for panda_link1..7, panda_hand, tcp
hand(q)          -> (pos, R) of panda_hand
ik(pos, R, q0)   -> damped-least-squares IK that stays continuous with q0
"""
import numpy as np
from scipy.spatial.transform import Rotation as Rot

BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 (measured via TF)
TCP = 0.1034
# (xyz, rpy) of each joint origin, from the URDF; all axes are local Z
_J = [
    ((0, 0, 0.333), (0, 0, 0)),
    ((0, 0, 0), (-np.pi / 2, 0, 0)),
    ((0, -0.316, 0), (np.pi / 2, 0, 0)),
    ((0.0825, 0, 0), (np.pi / 2, 0, 0)),
    ((-0.0825, 0.384, 0), (-np.pi / 2, 0, 0)),
    ((0, 0, 0), (np.pi / 2, 0, 0)),
    ((0.088, 0, 0), (np.pi / 2, 0, 0)),
]
LIM = np.array([[-2.9, 2.9], [-1.76, 1.76], [-2.9, 2.9], [-3.07, -0.07],
                [-2.9, 2.9], [-0.02, 3.75], [-2.9, 2.9]])
_FIX = [(np.eye(3), np.array(xyz)) for xyz, _ in _J]
_ROT = [Rot.from_euler("xyz", rpy).as_matrix() for _, rpy in _J]
_HAND_R = Rot.from_euler("z", -np.pi / 4).as_matrix()


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def fk(q):
    q = np.asarray(q, float)
    p, R = BASE.copy(), np.eye(3)
    out = {}
    for i in range(7):
        p = p + R @ _FIX[i][1]
        R = R @ _ROT[i] @ _rz(q[i])
        out[f"panda_link{i+1}"] = (p.copy(), R.copy())
    p = p + R @ np.array([0, 0, 0.107])
    R = R @ _HAND_R
    out["panda_hand"] = (p.copy(), R.copy())
    out["tcp"] = (p + TCP * R[:, 2], R.copy())
    return out


def hand(q):
    return fk(q)["panda_hand"]


def tcp(q):
    return fk(q)["tcp"]


def _err(q, pos, R, use_tcp):
    p, Rq = fk(q)["tcp" if use_tcp else "panda_hand"]
    e_p = pos - p
    e_r = Rot.from_matrix(R @ Rq.T).as_rotvec()
    return np.concatenate([e_p, e_r])


def _jac(q, pos, R, use_tcp, h=1e-6):
    e0 = _err(q, pos, R, use_tcp)
    J = np.zeros((6, 7))
    for i in range(7):
        dq = np.zeros(7); dq[i] = h
        J[:, i] = (_err(q + dq, pos, R, use_tcp) - e0) / h
    return e0, J


def ik(pos, R, q0, use_tcp=True, iters=200, tol_p=5e-4, tol_r=2e-3,
       lam=0.05, max_step=0.15, null_bias=0.0):
    """Return q near q0 reaching (pos, R) or raise RuntimeError.
    null_bias>0 pulls toward q0 in the nullspace (keeps posture)."""
    pos = np.asarray(pos, float)
    q = np.array(q0, float)
    for _ in range(iters):
        e, J = _jac(q, pos, R, use_tcp)
        if np.linalg.norm(e[:3]) < tol_p and np.linalg.norm(e[3:]) < tol_r:
            return np.clip(q, LIM[:, 0], LIM[:, 1])
        # J maps dq -> d(err)?? err = target - current so d err = -J_true dq;
        # we computed J of err directly, so solve J dq = -e
        JJt = J @ J.T + lam**2 * np.eye(6)
        dq = -J.T @ np.linalg.solve(JJt, e)
        if null_bias > 0:
            N = np.eye(7) - J.T @ np.linalg.solve(JJt, J)
            dq += null_bias * (N @ (np.array(q0) - q))
        n = np.linalg.norm(dq)
        if n > max_step:
            dq *= max_step / n
        q = np.clip(q + dq, LIM[:, 0] + 0.02, LIM[:, 1] - 0.02)
    e = _err(q, pos, R, use_tcp)
    raise RuntimeError(f"local IK did not converge: pos err {np.linalg.norm(e[:3]):.4f} m, "
                       f"rot err {np.degrees(np.linalg.norm(e[3:])):.1f} deg")


def path_min_z(q_from, q_to, n=20, extra=()):
    """Lowest world z of any link origin / tcp / extra hand-frame points
    along a straight joint-space path."""
    q_from, q_to = np.asarray(q_from), np.asarray(q_to)
    worst = (np.inf, None, None)
    for t in np.linspace(0, 1, n + 1):
        q = q_from + t * (q_to - q_from)
        f = fk(q)
        pts = [(k, v[0]) for k, v in f.items() if k not in ("panda_link1", "panda_link2")]
        ph, Rh = f["panda_hand"]
        for k, off in extra:
            pts.append((k, ph + Rh @ np.asarray(off)))
        k, p = min(pts, key=lambda kp: kp[1][2])
        if p[2] < worst[0]:
            worst = (p[2], k, (t, p))
    return worst


# hand-frame points of the palm/finger envelope for clearance checks
HAND_PTS = [("palm+y", (0, 0.1, 0.03)), ("palm-y", (0, -0.1, 0.03)),
            ("palm+y_tip", (0, 0.1, 0.06)), ("palm-y_tip", (0, -0.1, 0.06)),
            ("cam", (0.05, 0, 0.03)), ("finger+y", (0, 0.045, 0.1034)),
            ("finger-y", (0, -0.045, 0.1034))]


# world-frame obstacle boxes (xmin,xmax,ymin,ymax,zmin,zmax), incl. margins
OBST = {
    "table": (-1.5, 1.5, -1.5, 1.5, 0.0, 0.905),
    "bottle_fallen": (-0.37, -0.23, -0.09, 0.07, 0.9, 0.955),
    "bowl": (-0.06, 0.10, -0.12, 0.04, 0.9, 0.965),
    "board": (-0.26, 0.06, -0.28, -0.16, 0.9, 1.17),
    "cabinet": (-0.16, 0.16, 0.20, 0.46, 0.9, 1.15),
    "drawer": (-0.11, 0.11, 0.06, 0.24, 0.9, 1.0),
}


def body_points(q, hand_pts=HAND_PTS, seg=3):
    f = fk(q)
    names = ["panda_link2", "panda_link3", "panda_link4", "panda_link5",
             "panda_link6", "panda_link7", "panda_hand"]
    pts = []
    for a, b in zip(names, names[1:]):
        pa, pb = f[a][0], f[b][0]
        for t in np.linspace(0, 1, seg + 1)[1:]:
            pts.append((b, pa + t * (pb - pa)))
    ph, Rh = f["panda_hand"]
    pts.append(("tcp", f["tcp"][0]))
    for k, off in hand_pts:
        pts.append((k, ph + Rh @ np.asarray(off)))
    return pts


def in_box(p, b):
    return (b[0] <= p[0] <= b[1]) and (b[2] <= p[1] <= b[3]) and (b[4] <= p[2] <= b[5])


def collisions(q_from, q_to, n=25, ignore=(), hand_pts=HAND_PTS, skip_pts=()):
    """List of (t, point_name, obstacle) hits along a straight joint path."""
    q_from, q_to = np.asarray(q_from), np.asarray(q_to)
    hits = []
    for t in np.linspace(0, 1, n + 1):
        q = q_from + t * (q_to - q_from)
        for k, p in body_points(q, hand_pts):
            if k in skip_pts:
                continue
            for name, b in OBST.items():
                if name in ignore:
                    continue
                if in_box(p, b):
                    hits.append((round(float(t), 2), k, name, p.round(3)))
    return hits
