#!/usr/bin/env python3
"""Pick the alphabet soup can and drop it in the basket."""
import sys

from arm import *
from seg import cloud, region

SOUP = np.array([-0.204, -0.1175])           # silhouette-edge midpoint (measure_can.py)
BASKET = np.array([0.0, 0.255])
Z_HOVER, Z_GRASP, Z_CARRY, Z_DROP = 0.62, 0.46, 0.72, 0.75
Z_GOOD_ENOUGH = 0.48   # fingertips sit ~1 cm below FK TCP when tilted; can body is 0.425..0.512


def can_centre(cams=("agentview", "sideview", "frontview")):
    """Midpoint of the can's x/y silhouette edges (robust to partial rim)."""
    xr, yr, zr = (SOUP[0] - 0.05, SOUP[0] + 0.05), (SOUP[1] - 0.05, SOUP[1] + 0.05), (0.435, 0.512)
    xs, ys = [], []
    for cam in cams:
        P = cloud(cam, a.node)
        X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
        m = (X > xr[0]) & (X < xr[1]) & (Y > yr[0]) & (Y < yr[1]) & (Z > zr[0]) & (Z < zr[1])
        if m.sum() < 100:
            continue
        xe, ye = np.percentile(X[m], [1, 99]), np.percentile(Y[m], [1, 99])
        print(f"  {cam}: n={m.sum()} xmid={xe.mean():.4f} (w {xe[1]-xe[0]:.3f}) ymid={ye.mean():.4f} (w {ye[1]-ye[0]:.3f})")
        if xe[1] - xe[0] > 0.055:
            xs.append(xe.mean())
        if ye[1] - ye[0] > 0.055:
            ys.append(ye.mean())
    if not xs or not ys:
        return None
    return np.array([np.median(xs), np.median(ys)])
# fingers open along world y; hand leans 30 deg so the fingertips point
# down and toward the arm base (wrist farther out -> joint4 off its limit)
Q_SOUP = q_grasp(np.pi / 2, np.radians(30))
READY = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]

a = Arm()
print("start tcp", np.round(a.hand_pose()[0], 4))

print("== open gripper")
a.gripper(GRIP["open_m"])

print("== hover above soup (tilted)")
assert a.move_to([*SOUP, Z_HOVER], q=Q_SOUP, seconds=5, seed=READY)

print("== re-measure can from silhouette edges")
c = can_centre()
if c is not None and np.linalg.norm(c - SOUP) < 0.03:
    SOUP = c
    print("using measured centre", np.round(SOUP, 4))
else:
    print("measurement unusable, keeping prior centre", c)

print("== hover again over refined centre")
assert a.move_to([*SOUP, Z_HOVER], q=Q_SOUP, seconds=3)

print("== descend to grasp height")
ok = a.move_to([*SOUP, Z_GRASP], q=Q_SOUP, seconds=5, steps=4)
p, _ = a.hand_pose()
if not ok and p[2] > Z_GOOD_ENOUGH:
    print("descent blocked at", np.round(p, 4), "; lifting clear")
    a.move_to([p[0], p[1], Z_HOVER], q=Q_SOUP, seconds=3, steps=2)
    sys.exit(1)
print("grasping at tcp", np.round(p, 4))

print("== close gripper")
g = a.gripper(GRIP["closed_m"])
gap = g[0] + abs(g[1])
print("finger gap (sum) =", gap)
assert gap > 0.02, "closed on air"

print("== lift")
assert a.move_to([*SOUP, Z_CARRY], q=Q_SOUP, seconds=4, steps=3)
print("fingers after lift", a.finger_gap())

print("== move above basket")
assert a.move_to([*BASKET, Z_DROP], q=q_grasp(np.pi / 2), seconds=5)
print("fingers above basket", a.finger_gap())

print("== release")
a.gripper(GRIP["open_m"])

print("== retreat up")
a.move_to([*BASKET, Z_DROP + 0.05], q=q_grasp(np.pi / 2), seconds=2)
print("DONE soup")
