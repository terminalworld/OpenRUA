"""Plan the drawer-close push: fingertips on the bottom-drawer handle bar, hand pointing +y,
steep tilt while the wrist is over the bowl, flattening out as the hand nears the cabinet."""
import numpy as np, sys
from rob import *
from goto import ik_checked, margin
from coll import fk_all, CAPS
from scene import dist_aabb

BOWL_C = np.array([0.022, -0.062]); BOWL_R = 0.057; BOWL_Z = 0.952  # measured: shallow bowl


def bowl_clear(p, rad):
    rho = np.hypot(*(p[:2] - BOWL_C))
    d_rim = np.hypot(rho - BOWL_R, p[2] - BOWL_Z) - 0.005 - rad
    if rho < BOWL_R:
        h = 0.905 + 0.047 * (rho / BOWL_R) ** 2
        return min(d_rim, p[2] - rad - h)
    if p[2] < BOWL_Z:
        return min(d_rim, rho - 0.065 - rad)
    return d_rim


def check2(r, q, shift=0.0):
    """shift = how far the bottom drawer has been pushed in (m)."""
    P = fk_all(r, q); worst = (9, None)
    s = np.array([0, shift, 0])
    obs = [("cab", np.array([-0.18, 0.215, 0.90]), np.array([0.14, 0.42, 1.13])),
           ("handles", np.array([-0.05, 0.188, 0.995]), np.array([0.04, 0.215, 1.11])),
           ("boards", np.array([-0.26, -0.36, 0.90]), np.array([-0.03, -0.19, 1.22])),
           ("drawerL", np.array([-0.115, 0.075, 0.90]) + s, np.array([-0.10, 0.25, 0.985]) + s),
           ("drawerR", np.array([0.10, 0.075, 0.90]) + s, np.array([0.115, 0.25, 0.985]) + s),
           ("drawerF", np.array([-0.115, 0.075, 0.90]) + s, np.array([0.115, 0.095, 0.985]) + s)]
    _, Rh = r.fk_hand(q); ph = P["panda_hand"]
    hand_pts = [(ph + Rh @ np.array([sx * 0.0316, sy * 0.1013, sz]), 0.005)
                for sx in (-1, 1) for sy in (-1, 1) for sz in (0.0, 0.066)]
    hand_pts += [(ph + Rh @ np.array([sx * 0.0316, 0, sz]), 0.005) for sx in (-1, 1) for sz in (0.0, 0.066)]
    samples = [(P[a] + t * (P[bb] - P[a]), rad) for a, bb, rad in CAPS[:-1] for t in np.linspace(0, 1, 6)]
    flange = ph - 0.045 * Rh[:, 2]  # link7 cylinder ends ~4.5 cm behind the hand mounting face
    samples += [(P["panda_link7"] + t * (flange - P["panda_link7"]), 0.045) for t in np.linspace(0, 1, 6)]
    samples += [(ph + 0.045 * (np.cos(th) * Rh[:, 0] + np.sin(th) * Rh[:, 1]) - 0.01 * Rh[:, 2], 0.012)
                for th in np.linspace(0, 2 * np.pi, 12, endpoint=False)]  # flange disc
    samples += hand_pts
    for p, rad in samples:
        a = "hand" if rad <= 0.05 else "arm"
        for nm, lo, hi in obs:
            d = dist_aabb(p, lo, hi, rad)
            if d < worst[0]: worst = (d, (nm, a, np.round(p, 3)))
        d = p[2] - rad - 0.902
        if d < worst[0]: worst = (d, ("table", a, np.round(p, 3)))
        d = bowl_clear(p, rad)
        if d < worst[0]: worst = (d, ("bowl", a, np.round(p, 3)))
    return worst


def R_tilt(tilt_deg):
    t = np.radians(tilt_deg)
    Z = np.array([0, np.cos(t), -np.sin(t)]); Y = np.array([1.0, 0, 0]); X = np.cross(Y, Z)
    return np.column_stack([X, Y, Z])


XP, ZP = 0.0, 0.952
WAY = [  # (name, tcp, tilt_deg, shift)
    ("high", np.array([XP, -0.02, 1.15]), 45, 0.0),
    ("pre", np.array([XP, 0.03, ZP]), 60, 0.0),
    ("y06", np.array([XP, 0.06, ZP]), 60, 0.015),
    ("y10", np.array([XP, 0.10, ZP]), 45, 0.055),
    ("y14", np.array([XP, 0.14, ZP]), 38, 0.095),
    ("y17", np.array([XP, 0.17, ZP]), 35, 0.125),
    ("y19", np.array([XP, 0.19, ZP]), 35, 0.145),
]

if __name__ == "__main__":
    r = Robot("cp2")
    q0 = r.arm_q(); t0, R0 = r.tcp()
    print("tcp", np.round(t0, 3), "gap", round(r.finger_gap(), 4))
    qs = [q0]; plan = []
    for name, tcp, tilt, shift in WAY:
        R = R_tilt(tilt)
        q = ik_checked(r, tcp, R, qs[-1], max_dist=3.0 if len(qs) == 1 else 1.0, tries=10)
        if q is None: print(name, "no IK"); sys.exit(1)
        c = check2(r, q, shift)
        print(f"{name}: tilt={tilt} margin={margin(q):.2f} dq={np.abs(q - qs[-1]).max():.2f} clear={c[0]:.3f} {c[1]}")
        qs.append(q); plan.append((name, tcp, R, shift, q))
    np.save("snaps/close_plan.npy", np.array(plan, dtype=object), allow_pickle=True)
