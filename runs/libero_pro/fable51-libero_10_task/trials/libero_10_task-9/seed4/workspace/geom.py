"""Coarse geometric model of the scene for clearance checks (world frame, metres).

Everything is boxes.  Hand/finger/mug are sampled as point sets and tested
against the static solids with an SDF (negative = penetrating).
"""
import numpy as np

# ---------- static solids (axis aligned boxes: [xmin,xmax,ymin,ymax,zmin,zmax]) ----------
TABLE_Z = 0.90
MW_TOP = 1.108
OPEN_X = (-0.15, 0.05); OPEN_Z = (0.955, 1.085)     # opening in the front face y=0.25..0.27
CAV_X = (-0.151, 0.057); CAV_Y = (0.27, 0.42); CAV_Z = (0.942, 1.087)
BODY_X = (-0.18, 0.17); BODY_Y = (0.25, 0.455)

STATIC = [
    # table
    [-2, 2, -2, 2, TABLE_Z - 0.1, TABLE_Z],
    # microwave front frame: below opening (sill front), above opening (lip), left, right
    [BODY_X[0], BODY_X[1], BODY_Y[0], CAV_Y[0], TABLE_Z, OPEN_Z[0]],
    [BODY_X[0], BODY_X[1], BODY_Y[0], CAV_Y[0], OPEN_Z[1], MW_TOP],
    [BODY_X[0], OPEN_X[0], BODY_Y[0], CAV_Y[0], TABLE_Z, MW_TOP],
    [OPEN_X[1], BODY_X[1], BODY_Y[0], CAV_Y[0], TABLE_Z, MW_TOP],
    # cavity shell: floor, ceiling, left, right, back
    [BODY_X[0], BODY_X[1], CAV_Y[0], BODY_Y[1], TABLE_Z, CAV_Z[0]],
    [BODY_X[0], BODY_X[1], CAV_Y[0], BODY_Y[1], CAV_Z[1], MW_TOP],
    [BODY_X[0], CAV_X[0], CAV_Y[0], BODY_Y[1], TABLE_Z, MW_TOP],
    [CAV_X[1], BODY_X[1], CAV_Y[0], BODY_Y[1], TABLE_Z, MW_TOP],
    [BODY_X[0], BODY_X[1], CAV_Y[1], BODY_Y[1], TABLE_Z, MW_TOP],
    # control panel block protruding at the front right
    [0.055, BODY_X[1], 0.23, BODY_Y[0], TABLE_Z, MW_TOP],
]

# door: hinge at (-0.19, 0.26), free edge at (-0.27,-0.04); modelled as an
# oriented box 0.31 long, 0.02 thick, z 0.90..1.11
DOOR_HINGE = np.array([-0.19, 0.26])
DOOR_TIP = np.array([-0.27, -0.04])


def door_boxes(tip=DOOR_TIP):
    d = tip - DOOR_HINGE
    L = np.linalg.norm(d); u = d / L; n = np.array([-u[1], u[0]])
    return [(DOOR_HINGE, u, n, L, 0.02)]


def box_sdf(p, b):
    c = np.array([(b[0] + b[1]) / 2, (b[2] + b[3]) / 2, (b[4] + b[5]) / 2])
    h = np.array([(b[1] - b[0]) / 2, (b[3] - b[2]) / 2, (b[5] - b[4]) / 2])
    q = np.abs(p - c) - h
    return np.linalg.norm(np.maximum(q, 0), axis=-1) + np.minimum(q.max(-1), 0)


def door_sdf(p, tip=DOOR_TIP):
    (h0, u, n, L, t), = door_boxes(tip)
    rel = p[:, :2] - h0
    a = rel @ u; b = rel @ n
    q = np.stack([np.abs(a - L / 2) - L / 2, np.abs(b) - t / 2,
                  np.abs(p[:, 2] - (0.90 + 1.11) / 2) - 0.105], -1)
    return np.linalg.norm(np.maximum(q, 0), axis=-1) + np.minimum(q.max(-1), 0)


def static_sdf(p, door_tip=DOOR_TIP, include_door=True):
    p = np.atleast_2d(p)
    d = np.min([box_sdf(p, b) for b in STATIC], axis=0)
    if include_door:
        d = np.minimum(d, door_sdf(p, door_tip))
    return d


# ---------- hand model (panda_hand frame; z toward fingertips) ----------
HAND_BODY = [-0.035, 0.035, -0.102, 0.102, -0.03, 0.063]
CAM_MOUNT = [0.035, 0.065, -0.03, 0.03, -0.03, 0.02]
WRIST = [-0.05, 0.05, -0.05, 0.05, -0.20, -0.03]
FINGER_LEN = (0.063, 0.104)     # z range of a finger (tips measured at the TCP)
FINGER_HALF_W = 0.010            # along hand x
FINGER_THICK = 0.014             # along hand y, outward from the inner face


def box_points(b, step=0.005):
    xs = np.arange(b[0], b[1] + 1e-9, step); ys = np.arange(b[2], b[3] + 1e-9, step)
    zs = np.arange(b[4], b[5] + 1e-9, step)
    # surface only: 6 faces
    pts = []
    for X, Y in [(xs, ys)]:
        g = np.array(np.meshgrid(X, Y, indexing="ij")).reshape(2, -1).T
        pts += [np.c_[g, np.full(len(g), b[4])], np.c_[g, np.full(len(g), b[5])]]
    g = np.array(np.meshgrid(xs, zs, indexing="ij")).reshape(2, -1).T
    pts += [np.c_[g[:, 0], np.full(len(g), b[2]), g[:, 1]], np.c_[g[:, 0], np.full(len(g), b[3]), g[:, 1]]]
    g = np.array(np.meshgrid(ys, zs, indexing="ij")).reshape(2, -1).T
    pts += [np.c_[np.full(len(g), b[0]), g], np.c_[np.full(len(g), b[1]), g]]
    return np.vstack(pts)


def hand_points(finger_pos=(0.0, 0.0), step=0.005):
    """Points of hand+fingers in the hand frame. finger_pos: inner-face
    offsets of finger1 (+y) and finger2 (-y), metres."""
    pts = [box_points(HAND_BODY, step), box_points(CAM_MOUNT, step), box_points(WRIST, step)]
    p1, p2 = finger_pos
    pts.append(box_points([-FINGER_HALF_W, FINGER_HALF_W, p1, p1 + FINGER_THICK, *FINGER_LEN], step))
    pts.append(box_points([-FINGER_HALF_W, FINGER_HALF_W, -p2 - FINGER_THICK, -p2, *FINGER_LEN], step))
    return np.vstack(pts)


# ---------- mug model (mug frame: origin at base centre, z up, handle +y) ----------
MUG_R = 0.046; MUG_RI = 0.041; MUG_H = 0.112
HANDLE = [-0.012, 0.012, 0.046, 0.082, 0.02, 0.10]


def mug_points(step=0.005):
    ang = np.arange(0, 2 * np.pi, step / MUG_R)
    zs = np.arange(0, MUG_H + 1e-9, step)
    g = np.array(np.meshgrid(ang, zs, indexing="ij")).reshape(2, -1).T
    outer = np.c_[MUG_R * np.cos(g[:, 0]), MUG_R * np.sin(g[:, 0]), g[:, 1]]
    return np.vstack([outer, box_points(HANDLE, step)])


def transform(pts, R, t):
    return pts @ R.T + np.asarray(t)


def clearance(pts, **kw):
    d = static_sdf(pts, **kw)
    i = int(np.argmin(d))
    return float(d[i]), pts[i]
