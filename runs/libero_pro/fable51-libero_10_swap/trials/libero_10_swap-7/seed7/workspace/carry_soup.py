#!/usr/bin/env python3
"""Carry the (already grasped) can to the basket and release it."""
from arm import *

BASKET = np.array([0.0, 0.255])
Z_CARRY = 0.80
QH = q_from_axes((1, 0, 0), (0, 1, 0))
ph = np.radians(150)
QB = q_from_axes((-np.sin(ph), np.cos(ph), 0), (np.cos(ph), np.sin(ph), 0))  # hand horizontal, pointing 150deg


def slerp(q0, q1, t):
    q0, q1 = np.array(q0, float), np.array(q1, float)
    d = q0.dot(q1)
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        r = q0 + t * (q1 - q0)
        return tuple(r / np.linalg.norm(r))
    th = np.arccos(d)
    return tuple((np.sin((1 - t) * th) * q0 + np.sin(t * th) * q1) / np.sin(th))


a = Arm()
g = a.finger_gap()
print("fingers", np.round(g, 4))
assert g[0] + abs(g[1]) > 0.045, "not holding the can"
p, _ = a.hand_pose()
print("tcp", np.round(p, 4))

print("== up to carry height")
assert a.move_to([p[0], p[1], Z_CARRY], q=QH, seconds=3, steps=2)
p, _ = a.hand_pose()

print("== rotate hand in place (slerp waypoints)")
seed = a.arm_q()
wps = []
for t in np.linspace(0, 1, 7)[1:]:
    q = slerp(QH, QB, t)
    sol = a.solve_ik(p, q=q, seed=seed)
    assert sol is not None, f"no IK at t={t}"
    print(f"  t={t:.2f} joints {np.round(sol,3)}")
    wps.append(sol)
    seed = sol
a.move_joints(wps, seconds=8)
p2, _ = a.hand_pose()
print("  tcp after rotate", np.round(p2, 4), "fingers", np.round(a.finger_gap(), 4))

print("== translate to basket")
assert a.move_to([*BASKET, Z_CARRY], q=QB, seconds=6, steps=4, tol_m=0.02)
p, _ = a.hand_pose()
print("  tcp above basket", np.round(p, 4), "fingers", np.round(a.finger_gap(), 4))
assert abs(p[0] - BASKET[0]) < 0.04 and abs(p[1] - BASKET[1]) < 0.04

print("== release")
a.gripper(GRIP["open_m"])
print("fingers", np.round(a.finger_gap(), 4))
a.move_to([p[0], p[1], p[2] + 0.05], q=QB, seconds=2)
print("DONE soup")
