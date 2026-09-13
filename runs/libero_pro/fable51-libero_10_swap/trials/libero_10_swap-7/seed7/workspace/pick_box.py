#!/usr/bin/env python3
"""Pick the cream cheese box (8x4x3 cm, long axis along x) and drop it in the basket."""
import sys

from arm import *
from seg import cloud

BASKET = np.array([0.0, 0.255])
QV = q_grasp(np.pi / 2)        # vertical hand, fingers close along y (the 4 cm side)
Z_HOVER, Z_GRASP, Z_CARRY = 0.60, 0.437, 0.80
a = Arm()


def box_centre(guess):
    """Centroid of the box top from birdview + agentview (it is only 3 cm tall)."""
    cs = []
    for cam in ("birdview", "agentview"):
        P = cloud(cam, a.node)
        X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
        m = (abs(X - guess[0]) < 0.07) & (abs(Y - guess[1]) < 0.06) & (Z > 0.435) & (Z < 0.47)
        if m.sum() < 30:
            continue
        xe, ye = np.percentile(X[m], [2, 98]), np.percentile(Y[m], [2, 98])
        print(f"  {cam}: n={m.sum()} x=[{xe[0]:.3f},{xe[1]:.3f}] y=[{ye[0]:.3f},{ye[1]:.3f}] ztop={Z[m].max():.4f}")
        cs.append([xe.mean(), ye.mean()])
    return np.mean(cs, 0) if cs else None


print("== measure box")
BOX = box_centre(np.array([0.116, -0.209]))
assert BOX is not None
print("box centre", np.round(BOX, 4))

print("== open; go high then over to the box hover (continuity-checked)")
a.gripper(GRIP["open_m"])
p, _ = a.hand_pose()
assert a.move_pose([p[0], p[1], 0.90], a.link8_quat(), steps=3, seconds=3)
if not a.move_pose([*BOX, Z_HOVER + 0.15], QV, steps=8, seconds=10, tol_m=0.02):
    # fall back: plain IK move but only if it is a moderate joint move
    sol = a.solve_ik([*BOX, Z_HOVER + 0.15], q=QV, seed=[0, -0.5, 0, -2.0, 0, 1.8, 0.785])
    assert sol is not None
    print("  fallback joint move, joints", np.round(sol, 3))
    a.move_joints(sol, seconds=8)
assert a.move_to([*BOX, Z_HOVER], q=QV, seconds=3, steps=2)
print("  finger axis", np.round(finger_axis_world(a.link8_quat()), 3))

print("== re-measure and centre")
c = box_centre(BOX)
if c is not None and np.linalg.norm(c - BOX) < 0.03:
    BOX = c
print("box centre", np.round(BOX, 4))
assert a.move_to([*BOX, Z_HOVER], q=QV, seconds=2)

print("== descend")
ok = a.move_to([*BOX, Z_GRASP], q=QV, seconds=5, steps=4, tol_m=0.006)
p, _ = a.hand_pose()
if not ok and p[2] > 0.45:
    print("blocked at", np.round(p, 4), "lifting")
    a.move_to([p[0], p[1], Z_HOVER], q=QV, seconds=3, steps=2)
    sys.exit(1)

print("== close")
g = a.gripper(GRIP["closed_m"])
gap = g[0] + abs(g[1])
print("finger gap (sum) =", round(gap, 4))
if not 0.03 < gap < 0.055:
    print("no box in hand; opening and lifting")
    a.gripper(GRIP["open_m"])
    a.move_to([*BOX, Z_HOVER], q=QV, seconds=3, steps=2)
    sys.exit(1)

print("== lift")
assert a.move_to([*BOX, Z_CARRY], q=QV, seconds=4, steps=3)
print("fingers after lift", np.round(a.finger_gap(), 4))

print("== to basket")
if not a.move_pose([*BASKET, Z_CARRY], QV, steps=8, seconds=10, tol_m=0.02):
    sys.exit("basket move refused")
assert a.move_to([*BASKET, 0.72], q=QV, seconds=3, steps=2, tol_m=0.02)
p, _ = a.hand_pose()
print("  tcp above basket", np.round(p, 4), "fingers", np.round(a.finger_gap(), 4))
assert abs(p[0] - BASKET[0]) < 0.04 and abs(p[1] - BASKET[1]) < 0.04

print("== release")
a.gripper(GRIP["open_m"])
a.move_to([*BASKET, Z_CARRY], q=QV, seconds=2, steps=2)
print("DONE box")
