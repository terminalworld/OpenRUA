import numpy as np
from rob import *
from goto import ik_checked, margin
from coll import fk_all, CAPS
from scipy.spatial.transform import Rotation as Rot



def dist_aabb(p, lo, hi, rad):
    return np.linalg.norm(np.maximum(0, np.maximum(lo - p, p - hi))) - rad


def check(r, q):
    P = fk_all(r, q); worst = (9, None)
    obs = [("cab", np.array([-0.18, 0.215, 0.90]), np.array([0.14, 0.42, 1.13])),
           ("handles", np.array([-0.05, 0.185, 0.99]), np.array([0.04, 0.215, 1.11])),
           ("boards", np.array([-0.26, -0.36, 0.90]), np.array([-0.03, -0.19, 1.22])),
           ("drawerL", np.array([-0.115, 0.075, 0.90]), np.array([-0.10, 0.25, 0.985])),
           ("drawerR", np.array([0.10, 0.075, 0.90]), np.array([0.115, 0.25, 0.985])),
           ("drawerF", np.array([-0.115, 0.075, 0.90]), np.array([0.115, 0.095, 0.985]))]
    _, Rh = r.fk_hand(q); ph = P["panda_hand"]
    hand_pts = [(ph + Rh @ np.array([sx * 0.0316, sy * 0.1013, sz]), 0.005)
                for sx in (-1, 1) for sy in (-1, 1) for sz in (0.0, 0.066)]
    hand_pts += [(ph + Rh @ np.array([sx * 0.01, sy * 0.045, 0.105]), 0.005)
                 for sx in (-1, 1) for sy in (-1, 1)]
    hand_pts += [(ph + Rh @ np.array([0, 0, -0.05]), 0.05)]  # wrist flange behind the hand
    samples = [(P[a] + t * (P[bb] - P[a]), rad) for a, bb, rad in CAPS for t in np.linspace(0, 1, 6)]
    samples += [(p, rad, ) for p, rad in hand_pts]
    for p, rad in samples:
        if True:
            a = "hand" if rad <= 0.05 else "arm"
            for nm, lo, hi in obs:
                d = dist_aabb(p, lo, hi, rad)
                if d < worst[0]: worst = (d, (nm, a, np.round(p, 3)))
            d = p[2] - rad - 0.902
            if d < worst[0]: worst = (d, ("table", a, np.round(p, 3)))
            if p[2] - rad < 1.03:
                d = np.hypot(p[0] + 0.01, p[1] + 0.075) - 0.085 - rad
                if d < worst[0]: worst = (d, ("bowl", a, np.round(p, 3)))
    return worst


def R_for(R0, u, phi):
    ax = np.cross([0, 0, 1.0], u); ax /= np.linalg.norm(ax)
    R_align = Rot.from_rotvec(ax * np.pi / 2).as_matrix()
    return Rot.from_rotvec(np.asarray(u) * phi).as_matrix() @ R_align @ R0


