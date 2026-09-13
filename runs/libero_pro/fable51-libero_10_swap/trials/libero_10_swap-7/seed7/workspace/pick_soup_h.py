#!/usr/bin/env python3
"""Pick the alphabet soup can with a HORIZONTAL grasp and drop it in the basket.

MACHINE FACT: this gripper's pads are ~26 mm thick, inner faces only +-34 mm
from the TCP when open (68 mm gap). The can body is 62 mm, its rim 66 mm, so a
top-down straddle catches the rim. Instead approach from -y with the hand
horizontal (axis +y), fingers closing along x on the body below the rim.
"""
import sys

from arm import *
from seg import cloud

BASKET = np.array([0.0, 0.255])
Z_G = 0.485          # pads ~0.475..0.495: below rim (0.512), hand clear of table (0.425)
APPROACH = 0.10      # start this far short of the can along -y
QH = q_from_axes((1, 0, 0), (0, 1, 0))          # fingers along x, fingertips toward +y
QV = q_grasp(np.pi / 2)                          # vertical hand, fingers along y
SEED_H = [-0.7, 0.3, 0, -2.0, 0, 2.3, 0.0]
Z_HOVER = 0.62

a = Arm()


def can_centre(guess, cams=("agentview", "sideview", "frontview")):
    xs, ys = [], []
    for cam in cams:
        P = cloud(cam, a.node)
        X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
        m = (abs(X - guess[0]) < 0.06) & (abs(Y - guess[1]) < 0.06) & (Z > 0.435) & (Z < 0.512)
        if m.sum() < 100:
            continue
        xe, ye = np.percentile(X[m], [1, 99]), np.percentile(Y[m], [1, 99])
        print(f"  {cam}: n={m.sum()} xmid={xe.mean():.4f} (w {xe[1]-xe[0]:.3f}) ymid={ye.mean():.4f} (w {ye[1]-ye[0]:.3f})")
        if xe[1] - xe[0] > 0.055:
            xs.append(xe.mean())
        if ye[1] - ye[0] > 0.055:
            ys.append(ye.mean())
    return np.array([np.median(xs), np.median(ys)]) if xs and ys else None


def extent(name, box):
    """z/x/y extents of cloud points inside a world box (from agentview+sideview)."""
    pts = []
    for cam in ("agentview", "sideview"):
        P = cloud(cam, a.node).reshape(-1, 3)
        m = np.all(np.isfinite(P), 1)
        for i in range(3):
            m &= (P[:, i] > box[i][0]) & (P[:, i] < box[i][1])
        pts.append(P[m])
    pts = np.concatenate(pts)
    if len(pts):
        print(f"  {name}: n={len(pts)} x=[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] "
              f"y=[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z=[{pts[:,2].min():.3f},{pts[:,2].max():.3f}]")
    return pts


print("== open, clear the can")
a.gripper(GRIP["open_m"])
p, _ = a.hand_pose()
if p[2] < Z_HOVER - 0.02:
    a.move_to([p[0], p[1], Z_HOVER], q=q_grasp(np.pi / 2, np.radians(30)), seconds=3, steps=2)

print("== measure can")
SOUP = can_centre(np.array([-0.234, -0.14]))
assert SOUP is not None, "can not found"
print("can centre", np.round(SOUP, 4))

print("== high pre-pose, hand horizontal pointing +y")
pre_xy = SOUP - [0, APPROACH]
assert a.move_to([*pre_xy, Z_HOVER], q=QH, seconds=6, seed=SEED_H)
print("  finger axis now", np.round(finger_axis_world(a.hand_pose()[1]), 3))
p, _ = a.hand_pose()
print("  gripper extents at high pose (tcp", np.round(p, 3), ")")
extent("fingers", [(p[0] - 0.08, p[0] + 0.08), (p[1] - 0.03, p[1] + 0.03), (p[2] - 0.08, p[2] + 0.08)])
extent("hand", [(p[0] - 0.15, p[0] + 0.15), (p[1] - 0.20, p[1] - 0.06), (p[2] - 0.12, p[2] + 0.12)])

print("== descend beside the can")
assert a.move_to([*pre_xy, Z_G], q=QH, seconds=5, steps=3, tol_m=0.006)

print("== slide +y onto the can body")
ok = a.move_to([*SOUP, Z_G], q=QH, seconds=5, steps=4, tol_m=0.006)
p, _ = a.hand_pose()
if not ok:
    print("slide blocked at", np.round(p, 4), "-> backing off")
    a.move_to([*pre_xy, Z_G], q=QH, seconds=3, steps=2)
    a.move_to([*pre_xy, Z_HOVER], q=QH, seconds=3, steps=2)
    sys.exit(1)

print("== close")
g = a.gripper(GRIP["closed_m"])
gap = g[0] + abs(g[1])
print("finger gap (sum) =", round(gap, 4))
if gap < 0.045:
    print("did not grip the can; opening and backing off")
    a.gripper(GRIP["open_m"])
    a.move_to([*pre_xy, Z_G], q=QH, seconds=3, steps=2)
    a.move_to([*pre_xy, Z_HOVER], q=QH, seconds=3, steps=2)
    sys.exit(1)

print("== lift")
assert a.move_to([*SOUP, 0.72], q=QH, seconds=4, steps=3)
print("fingers after lift", np.round(a.finger_gap(), 4))

print("== to basket")
cur = a.arm_q()
done = False
for q in (QH, q_from_axes((1, 0, 0), (0.7, 1, 0)), q_from_axes((0, 1, 0), (1, 0, 0)), QV):
    sol = a.solve_ik([*BASKET, 0.75], q=q, seed=cur)
    if sol is not None and abs(sol[0] - 0.46) < 0.8 and abs(sol[2]) < 1.6:
        print("  basket orientation", np.round(q, 3), "joints", np.round(sol, 3))
        a.move_joints(sol, seconds=6)
        done = True
        break
assert done, "no basket IK"
p, _ = a.hand_pose()
print("  tcp above basket", np.round(p, 4), "fingers", np.round(a.finger_gap(), 4))
assert abs(p[0] - BASKET[0]) < 0.03 and abs(p[1] - BASKET[1]) < 0.03

print("== release")
a.gripper(GRIP["open_m"])
a.move_to([p[0], p[1], p[2] + 0.05], q=q, seconds=2)
print("DONE soup")
