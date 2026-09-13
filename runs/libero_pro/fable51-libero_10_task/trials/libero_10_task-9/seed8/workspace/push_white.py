"""Slide the (upside-down) white mug along direction e with closed fingertips (hand pointing down)."""
import sys
from common import *
c = Ctl("push_white"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
scene_no_white(sc)
cen = np.array([-0.15, 0.135])
e = np.array([0.5, -0.866])
Z = 0.96
start = np.r_[cen - 0.077 * e, Z]
end = np.r_[cen + 0.043 * e, Z]
def origin(tip): return np.asarray(tip) + [0, 0, 0.1034]
q = r.arm_q()
for name, t in (("start", start), ("end", end)):
    qq = c.ik_valid(origin(t), R_DOWN, seed=q, tries=8); print(name, np.round(t, 3), None if qq is None else np.round(qq, 2)); q = qq or q
if dry: sys.exit()
if r.finger_gap() > 0.01: r.gripper(0.0)
q0 = c.ik_valid(origin(start) + [0, 0, 0.12], R_DOWN, seed=r.arm_q(), tries=8)
assert c.goto_q(q0, t=4.0), "pre failed"; r.report("pre")
assert c.line(origin(start), R_DOWN, n=3, t=3.0), "descend failed"; r.report("start")
try:
    c.line(origin(end), R_DOWN, n=5, t=5.0)
except RuntimeError as ex:
    print("push:", ex)
r.report("end")
p, _ = r.fk()
assert c.line(p + [0, 0, 0.15], R_DOWN, n=2, t=2.0), "lift failed"; r.report("up")
