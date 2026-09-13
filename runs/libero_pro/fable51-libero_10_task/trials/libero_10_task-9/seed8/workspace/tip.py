"""Tip the upside-down mug over toward +y by pushing its top from -y with closed fingertips."""
import sys
from common import *
c = Ctl("tip"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
scene_no_white(sc)
cx, cy = float(sys.argv[1]), float(sys.argv[2])
yh = np.array([1.0, 0, 0]); zh = np.array([0, 0, -1.0]); xh = np.cross(yh, zh)
Rf = R_from_axes(xh, yh, zh)     # vertical fingers, finger backs facing +-y
def origin(tip): return np.asarray(tip) + [0, 0, 0.1034]
Z = 0.99
t0 = np.array([cx, cy - 0.067, Z]); t1 = np.array([cx, cy + 0.03, Z])
q = r.arm_q()
for name, t in (("t0", t0), ("t1", t1)):
    qq = c.ik_valid(origin(t), Rf, seed=q, tries=8); print(name, np.round(t, 3), None if qq is None else np.round(qq, 2)); q = qq or q
if dry: sys.exit()
if r.finger_gap() > 0.01: r.gripper(0.0)
q0 = c.ik_valid(origin(t0) + [0, 0, 0.10], Rf, seed=r.arm_q(), tries=8)
assert c.goto_q(q0, t=5.0), "pre failed"; r.report("pre")
assert c.line(origin(t0), Rf, n=3, t=3.0), "descend failed"; r.report("t0")
try:
    c.line(origin(t1), Rf, n=5, t=5.0)
except RuntimeError as e:
    print("push:", e)
r.report("t1")
p, _ = r.fk()
assert c.line(p + [0, 0, 0.12], Rf, n=2, t=2.0), "lift failed"; r.report("up")
r.snap("agentview"); r.snap("frontview")
