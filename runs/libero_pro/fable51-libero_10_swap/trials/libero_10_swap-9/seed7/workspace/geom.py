"""Geometry model: mug held in the tilted pinch grasp + hand housing vs microwave."""
import numpy as np
from scipy.spatial.transform import Rotation as Rot

TH0 = 45.0
def a_of(th):
    t = np.radians(th); return np.array([0, np.cos(t), -np.sin(t)])
def n_of(th):
    t = np.radians(th); return np.array([0, np.sin(t), np.cos(t)])

def mug_points(tip, th):
    """sample points of the mug for a fingertip position `tip` at tilt th (deg)."""
    a = a_of(th); P = tip - 0.01 * a
    R = Rot.from_euler("x", -(th - TH0), degrees=True)
    C = P + R.apply([0, 0.081, -0.0125]); u = R.apply([0, 0, 1.0])
    e1 = R.apply([1.0, 0, 0]); e2 = R.apply([0, 1.0, 0])
    pts = []
    for h, rad in ((-0.0525, 0.0375), (0.0525, 0.047), (0.0, 0.042)):
        c = C + h * u
        for ang in np.linspace(0, 2 * np.pi, 24, endpoint=False):
            pts.append(c + rad * (np.cos(ang) * e1 + np.sin(ang) * e2))
    # handle outer bar (approx at the pad) and top piece
    pts.append(P); pts.append(P + R.apply([0, 0.03, 0.02]))
    return np.array(pts)

def hand_points(tip, th):
    a = a_of(th); n = n_of(th); O = tip - 0.1034 * a
    pts = []
    for s in np.linspace(0, 0.058, 4):
        for k in (-0.03, 0.03):
            for x in (-0.10, 0.0, 0.10):
                pts.append(O + s * a + k * n + [x, 0, 0])
    for s in np.linspace(0.0, 0.12, 4):       # wrist cylinder
        for k in (-0.045, 0.045):
            for x in (-0.045, 0.045):
                pts.append(O - s * a + k * n + [x, 0, 0])
    for s in np.linspace(0.058, 0.1034, 3):   # fingers (open)
        for k in (-0.01, 0.01):
            for x in (-0.045, 0.045):
                pts.append(O + s * a + k * n + [x, 0, 0])
    return np.array(pts)

def in_box(p, lo, hi):
    return np.all((p >= lo) & (p <= hi), axis=1)

def collides(pts, margin=0.0):
    m = margin
    body = in_box(pts, [-0.27 - m, -0.345 - m, 0.9], [0.09 + m, -0.12 + m, 1.107 + m])
    cav = in_box(pts, [-0.222 + m, -0.345 - 0.05, 0.944 + m], [-0.014 - m, -0.16 - m, 1.09 - m])
    door = in_box(pts, [-0.280 - m, -0.61, 0.9], [-0.257 + m, -0.35, 1.107 + m])
    table = pts[:, 2] < 0.9 + m
    return np.any((body & ~cav) | door | table)

def ok(tip, th, margin=0.005):
    return (not collides(mug_points(tip, th), margin)) and (not collides(hand_points(tip, th), margin))
