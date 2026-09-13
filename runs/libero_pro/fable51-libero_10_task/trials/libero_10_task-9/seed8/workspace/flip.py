"""Pinch the outer handle bar of the upside-down mug from above (vertical fingers straddling the loop plane),
lift: the COM is 7.8 cm off the pin so the mug swings to hang with its base pointing away from the bar.
Carry to L and lower: it lands base-edge first and lays down on its side with the handle bar on top."""
import sys
from common import *
c = Ctl("flip"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
scene_no_white(sc)
mx, my, ang = float(sys.argv[1]), float(sys.argv[2]), np.radians(float(sys.argv[3]))
u = np.array([np.cos(ang), np.sin(ang), 0]); d = np.array([-u[1], u[0], 0])
zh = np.array([0, 0, -1.0]); Rb = R_from_axes(np.cross(d, zh), d, zh)      # fingers close along d (across the bar)
B = np.r_[mx, my, 0] + 0.075 * u
def origin(tip): return np.asarray(tip) + [0, 0, 0.1034]
L = np.array([-0.15, -0.15])
Z_GRIP, Z_HI, Z_LAND = 0.935, 1.10, 1.005
seq = [("above", np.r_[B[:2], 1.06]), ("down", np.r_[B[:2], Z_GRIP]), ("lift", np.r_[B[:2], Z_HI]),
       ("carry", np.r_[L, Z_HI]), ("land", np.r_[L, Z_LAND])]
q = r.arm_q()
for name, t in seq:
    qq = c.ik_valid(origin(t), Rb, seed=q, tries=8); print(name, np.round(t, 3), None if qq is None else np.round(qq, 2)); q = qq or q
if dry: sys.exit()
r.report("start")
if r.finger_gap() < 0.07: r.gripper(0.08)
q0 = c.ik_valid(origin(seq[0][1]), Rb, seed=r.arm_q(), tries=8)
assert c.goto_q(q0, t=5.0), "above failed"; r.report("above")
try:
    c.line(origin(seq[1][1]), Rb, n=4, t=4.0)
except RuntimeError as e:
    print("down:", e)
r.report("down")
gap = r.gripper(0.0); print("gap after close", gap)
if not (0.008 < gap < 0.03): sys.exit("bar grasp missed")
assert c.line(origin(seq[2][1]), Rb, n=4, t=6.0), "lift failed"; r.report("lift")
r.snap("agentview", "/workspace/hang_a.png"); r.snap("sideview", "/workspace/hang_s.png")
assert c.line(origin(seq[3][1]), Rb, n=4, t=4.0), "carry failed"; r.report("carry")
try:
    c.line(origin(seq[4][1]), Rb, n=5, t=6.0)
except RuntimeError as e:
    print("land:", e)
r.report("land")
r.gripper(0.08)
p, _ = r.fk()
assert c.line(p + [0, 0, 0.12], Rb, n=2, t=2.0), "retreat failed"; r.report("up")
r.snap("agentview"); r.snap("frontview")
