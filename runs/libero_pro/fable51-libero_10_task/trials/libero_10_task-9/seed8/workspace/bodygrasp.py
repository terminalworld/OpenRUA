"""Vertical body pinch on the upside-down mug (near its base = top), yawed 45 deg to clear the door.
Then carry it to a free spot, set it down (still upside down), release."""
import sys
from common import *
c = Ctl("bodygrasp"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
scene_no_white(sc)
yh = np.array([0.7071, 0.7071, 0]); zh = np.array([0, 0, -1.0]); xh = np.cross(yh, zh)
Rv = R_from_axes(xh, yh, zh)
def origin(tip): return np.asarray(tip) + [0, 0, 0.1034]
CEN = np.array([-0.151, 0.135])
DST = np.array([-0.05, 0.05])
Zt_hi, Zt_lo = 1.06, 0.978
seq = [("above", np.r_[CEN, Zt_hi]), ("down", np.r_[CEN, Zt_lo]), ("lift", np.r_[CEN, Zt_lo + 0.04]),
       ("carry", np.r_[DST, Zt_lo + 0.04]), ("set", np.r_[DST, Zt_lo])]
q = r.arm_q()
for name, t in seq:
    qq = c.ik_valid(origin(t), Rv, seed=q, tries=8); print(name, np.round(t, 3), None if qq is None else np.round(qq, 2)); q = qq or q
if dry: sys.exit()
r.report("start")
if r.finger_gap() < 0.07: r.gripper(0.08)
q0 = c.ik_valid(origin(seq[0][1]), Rv, seed=r.arm_q(), tries=8)
assert c.goto_q(q0, t=5.0), "above failed"; r.report("above")
try:
    c.line(origin(seq[1][1]), Rv, n=4, t=4.0)
except RuntimeError as e:
    print("down:", e)
r.report("down")
gap = r.gripper(0.0)
print("gap after close", gap)
if gap < 0.05:
    sys.exit("grasp missed")
p, _ = r.fk()
assert c.line(p + [0, 0, 0.04], Rv, n=2, t=2.0), "lift failed"; r.report("lift")
r.snap("agentview", "/workspace/lift_a.png")
assert c.line(origin(seq[3][1]), Rv, n=4, t=4.0), "carry failed"; r.report("carry")
try:
    c.line(origin(seq[4][1]) + [0, 0, 0.004], Rv, n=3, t=3.0)
except RuntimeError as e:
    print("set:", e)
r.report("set")
r.gripper(0.08)
p, _ = r.fk()
assert c.line(p + [0, 0, 0.10], Rv, n=2, t=2.0), "retreat failed"; r.report("up")
r.snap("agentview"); r.snap("frontview")
