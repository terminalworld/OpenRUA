import sys, numpy as np, kin, motion
from ctl import Ctl
kin.OBST.pop("bottle_fallen", None)
kin.OBST["drawer"] = (-0.11, 0.11, 0.07, 0.24, 0.9, 0.99)
kin.OBST["handle_low"] = (-0.05, 0.06, 0.035, 0.08, 0.9, 0.965)
kin.OBST["handles_up"] = (-0.05, 0.06, 0.19, 0.22, 0.99, 1.11)
kin.OBST["bowl"] = (-0.03, 0.09, -0.10, 0.03, 0.9, 0.96)   # measured rim, small margin
PTS = list(kin.HAND_PTS) + [("palmcorner+", (0.025, 0.1, 0.058)), ("palmcorner-", (0.025, -0.1, 0.058)),
        ("palmcorner2+", (-0.025, 0.1, 0.058)), ("palmcorner2-", (-0.025, -0.1, 0.058)), ("camlow", (0.05, 0, 0.058))]
skip = ("tcp", "finger+y", "finger-y")
def Rt(deg):
    th = np.radians(deg); return motion.R_from_axes([0, np.sin(th), -np.cos(th)], [-1, 0, 0])
c = Ctl()
X0 = -0.025
q = np.array(c.arm_q())
stages = [("pre", [X0, 0.03, 1.10], Rt(20), ()), ("down", [X0, 0.03, 0.945], Rt(20), ("drawer", "handle_low")),
          ("push", [X0, 0.185, 0.945], Rt(40), ("drawer", "handle_low")), ("up", [X0, 0.185, 1.10], Rt(40), ("drawer", "handle_low"))]
plan = []
for name, tgt, R, ign in stages:
    qs = motion.cart_path(q, tgt, R)
    hits = motion.check(qs, q, ignore=ign, skip_pts=skip if ign else (), hand_pts=PTS)
    print(f"{name}: dq {np.abs(qs[-1]-q).max():.2f} hits {len(hits)} {hits[:2]}")
    ph, Rh = kin.hand(qs[-1]); print("   camlow", (ph + Rh @ np.array((0.05,0,0.058))).round(3), "corner2-", (ph + Rh @ np.array((-0.025,-0.1,0.058))).round(3))
    plan.append((name, qs, ign)); q = qs[-1]
if "--go" in sys.argv:
    print("close fingers", c.gripper(0.0))
    for name, qs, ign in plan:
        motion.execute(c, qs, ignore=ign, skip_pts=skip if ign else (), hand_pts=PTS, label=name)
c.node.destroy_node()
