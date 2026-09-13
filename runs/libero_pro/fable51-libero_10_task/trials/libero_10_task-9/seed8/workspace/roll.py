"""Roll the lying mug by pushing its (upward) handle sideways toward -x with closed fingertips."""
import sys
from common import *
c = Ctl("roll"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
scene_no_white(sc)
Rf = hand_R(np.pi / 2)
r.report("start")
z_h = Rf[:, 2]
def origin(tip): return np.asarray(tip) - 0.1034 * z_h
Y, Z = 0.02, 0.986
tips = [np.array([-0.135, Y, 1.10]), np.array([-0.135, Y, Z]), np.array([-0.205, Y, Z]), np.array([-0.205, Y, 1.10])]
q = r.arm_q()
for t in tips:
    qq = c.ik_valid(origin(t), Rf, seed=q, tries=6); print(np.round(t, 3), None if qq is None else np.round(qq, 2)); q = qq or q
if dry: sys.exit()
if r.finger_gap() > 0.01: r.gripper(0.0)
assert c.line(origin(tips[0]), Rf, n=4, t=4.0), "approach failed"; r.report("above")
assert c.line(origin(tips[1]), Rf, n=3, t=3.0), "descend failed"; r.report("down")
try:
    c.line(origin(tips[2]), Rf, n=5, t=5.0)
except RuntimeError as e:
    print("push:", e)
r.report("pushed")
assert c.line(origin(tips[3]), Rf, n=3, t=3.0), "lift failed"; r.report("up")
for cam in ("agentview", "frontview", "sideview"):
    r.snap(cam)
