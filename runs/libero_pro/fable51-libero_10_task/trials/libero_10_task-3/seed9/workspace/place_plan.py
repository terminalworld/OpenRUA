import numpy as np
from rob import *
from goto import ik_checked, margin
from coll import fk_all, CAPS
from scipy.spatial.transform import Rotation as Rot

r = Robot("pp")
q0 = r.arm_q(); t0, R0 = r.tcp()
print("tcp", np.round(t0, 3))
b = R0.T @ np.array([0, 0, 1.0])          # bottle axis (base->tip) in hand frame
CS = {(-1.0,0,0): np.array([-0.01,0.14,0.972])}


def dist_aabb(p, lo, hi, rad):
    return np.linalg.norm(np.maximum(0, np.maximum(lo - p, p - hi))) - rad


def check(q):
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


def R_for(u, phi):
    ax = np.cross([0, 0, 1.0], u); ax /= np.linalg.norm(ax)
    R_align = Rot.from_rotvec(ax * np.pi / 2).as_matrix()
    return Rot.from_rotvec(np.asarray(u) * phi).as_matrix() @ R_align @ R0


best = []
for uk, C in CS.items():
    u = np.array(uk); u = u / np.linalg.norm(u); G_REL = C + 0.0725 * u
    for phi_deg in [340,350,0,10,20]:
        R = R_for(u, np.radians(phi_deg))
        assert np.allclose(R @ b, u, atol=1e-6)
        Z = R[:, 2]
        if Z[2] > -0.4: continue
        q = ik_checked(r, G_REL, R, q0, max_dist=4.0)
        if q is None: print(u, phi_deg, "Zz", round(Z[2], 2), "no IK"); continue
        c = check(q)
        qh = ik_checked(r, G_REL + [0, 0, 0.12], R, q, max_dist=1.0)
        ch = check(qh) if qh is not None else (None, None)
        print(f"u={u} phi={phi_deg} Z={np.round(Z,2)} Y={np.round(R[:,1],2)} margin={margin(q):.2f} clear={c[0]:.3f} {c[1]} | high {ch}")
        if qh is not None:
            best.append((min(c[0], ch[0]), margin(q), u, phi_deg, R, q, qh, G_REL))
best.sort(key=lambda x: -x[0])
for bst in best[:5]:
    print("BEST", bst[0], bst[1], bst[2], bst[3])
np.save("snaps/place_best.npy", np.array(best[:5], dtype=object), allow_pickle=True)
