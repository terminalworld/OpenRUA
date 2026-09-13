#!/usr/bin/env python3
"""Grasp the yellow mug by its handle (fingers along x, hand tilted,
approaching from -y/above), carry it over the microwave, insert, then push it deeper."""
import sys
import numpy as np
from ctl import Robot, quat_from_axes, log
from geom import a_of, ok, hand_points, collides

def Q(th):
    return quat_from_axes(a_of(th), [1.0, 0.0, 0.0])

G = np.array([0.0215, 0.099, 0.958])        # fingertip at grasp (theta 45)
X = -0.13
r = Robot("mug")

def plan(steps, seed, with_mug=True):
    """steps: list of (tip, theta). returns joint list."""
    out = []
    for tip, th in steps:
        tip = np.array(tip, float)
        if with_mug:
            geo = ok(tip, th, margin=0.003)
        else:
            geo = not collides(hand_points(tip, th), 0.003)
        s = r.ik(tip, Q(th), seed=seed, at_tcp=True)
        jump = None if s is None else round(float(np.abs(np.array(s) - np.array(seed)).max()), 2)
        log(f"  {np.round(tip,3)} th={th} geo={'ok' if geo else 'HIT'} ik={'ok' if s else 'FAIL'} jump={jump} q={None if s is None else np.round(s,2)}")
        if s is None:
            sys.exit("IK failed")
        out.append(s); seed = s
    return out

seed = r.joints()
A45 = a_of(45)
log("pre");   pre = plan([(G - 0.10 * A45 + [0, 0, 0.12], 45), (G - 0.10 * A45, 45)], seed, False)
log("app");   app = plan([(G - 0.06 * A45, 45), (G - 0.03 * A45, 45), (G, 45)], pre[-1], False)
log("lift");  lift = plan([(G + [0, 0, 0.07], 45), ((G[0], G[1], 1.22), 65)], app[-1])
log("carry"); carry = plan([((-0.04, -0.10, 1.22), 65), ((X, -0.30, 1.22), 65), ((X, -0.40, 1.22), 65), ((X, -0.47, 1.22), 65)], lift[-1])
log("down");  down = plan([((X, -0.47, 1.12), 65), ((X, -0.47, 1.05), 65)], carry[-1])
log("trans"); trans = plan([((X, -0.44, 1.04), 65), ((X, -0.44, 1.04), 55), ((X, -0.44, 1.03), 50), ((X, -0.40, 1.02), 50), ((X, -0.40, 1.02), 45)], down[-1])
log("ins");   ins = plan([((X, -0.37, 1.02), 45), ((X, -0.338, 1.02), 45), ((X, -0.338, 1.012), 45)], trans[-1])
log("ret");   ret = plan([((X, -0.37, 1.02), 45), ((X, -0.40, 1.02), 45)], ins[-1], False)
log("push");  push = plan([((X, -0.36, 0.975), 25), ((X, -0.36, 0.965), 25), ((X, -0.34, 0.965), 25), ((X, -0.322, 0.965), 25)], ret[-1], False)
log("out");   out = plan([((X, -0.36, 0.975), 25), ((X, -0.40, 1.02), 45), ((X, -0.44, 1.10), 55)], push[-1], False)
if "dry" in sys.argv:
    sys.exit(0)

log("== open"); r.gripper(0.04)
log("== pre-grasp"); r.move_joints(pre, [6.0, 10.0])
log("== approach"); r.move_joints(app, [2.0, 4.0, 6.0])
log("hand", np.round(r.hand_pose()[0], 3))
log("== close"); f = r.gripper(0.0)
if not (0.005 < f[0] < 0.02):
    log("!! grasp failed"); sys.exit(1)
log("== lift"); r.move_joints(lift, [4.0, 9.0])
log("fingers", r.fingers())
log("== carry"); r.move_joints(carry, [5.0, 10.0, 13.0, 16.0])
log("== descend"); r.move_joints(down, [3.0, 6.0])
log("== transition"); r.move_joints(trans, [3.0, 6.0, 9.0, 12.0, 15.0])
log("fingers", r.fingers())
log("== insert"); r.move_joints(ins, [3.0, 6.0, 8.0], hold=3.0)
log("hand", np.round(r.hand_pose()[0], 3), "fingers", r.fingers())
log("== release"); r.gripper(0.04)
log("== retreat"); r.move_joints(ret, [3.0, 6.0])
log("== close fingers"); r.gripper(0.0)
log("== push"); r.move_joints(push, [4.0, 6.0, 8.0, 10.0], hold=3.0)
log("== out"); r.move_joints(out, [3.0, 6.0, 9.0])
log("done")
