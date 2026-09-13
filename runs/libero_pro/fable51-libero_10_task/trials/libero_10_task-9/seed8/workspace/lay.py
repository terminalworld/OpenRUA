"""Rigid bar pinch of the upside-down mug (vertical fingers across the outer bar, which is vertical),
lift, yaw so the body is at -y of the bar, then pitch the hand to horizontal (+y): the mug then lies with
the handle bar on top, base toward -y. Lower onto the table, release."""
import sys
from common import *
c = Ctl("lay"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
scene_no_white(sc)
mx, my, ang = float(sys.argv[1]), float(sys.argv[2]), np.radians(float(sys.argv[3]))
u = np.array([np.cos(ang), np.sin(ang), 0]); d = np.array([-u[1], u[0], 0])
zh = np.array([0, 0, -1.0]); Rb = R_from_axes(np.cross(d, zh), d, zh)
Rc = R_from_axes(np.array([0, -1.0, 0]), np.array([-1.0, 0, 0]), zh)      # body (x_hand) toward -y
Rh = hand_R(np.pi / 2, 0.0)                                              # horizontal, pointing +y, body below
B = np.r_[mx, my, 0] + 0.075 * u
def origin(tip, R): return np.asarray(tip) - 0.1034 * R[:, 2]
L = np.array([-0.12, -0.12])
Z_GRIP, Z_HI, Z_LAY = 0.935, 1.15, 1.02
seq = [("above", np.r_[B[:2], 1.06], Rb), ("down", np.r_[B[:2], Z_GRIP], Rb), ("lift", np.r_[B[:2], Z_HI], Rb),
       ("yaw", np.r_[B[:2], Z_HI], Rc), ("pitch45", np.r_[L, Z_HI], hand_R(np.pi / 2, np.pi / 4)), ("pitch", np.r_[L, Z_HI], Rh), ("lay", np.r_[L, Z_LAY], Rh), ("out", np.r_[L, Z_LAY + 0.12], Rh)]
q = r.arm_q()
for name, t, R in seq:
    qq = c.ik_valid(origin(t, R), R, seed=q, tries=8); print(name, np.round(t, 3), None if qq is None else np.round(qq, 2)); q = qq or q
if dry: sys.exit()
r.report("start")
if r.finger_gap() < 0.07: r.gripper(0.08)
q0 = c.ik_valid(origin(seq[0][1], Rb), Rb, seed=r.arm_q(), tries=8)
assert c.goto_q(q0, t=5.0), "above failed"; r.report("above")
try:
    c.line(origin(seq[1][1], Rb), Rb, n=4, t=4.0)
except RuntimeError as e:
    print("down:", e)
r.report("down")
gap = r.gripper(0.0); print("gap after close", gap)
if not (0.008 < gap < 0.03): sys.exit("bar grasp missed")
assert c.line(origin(seq[2][1], Rb), Rb, n=4, t=6.0), "lift failed"; r.report("lift")
for name, t, R in seq[3:6]:
    qq = c.ik_valid(origin(t, R), R, seed=r.arm_q(), tries=8)
    assert qq is not None and c.goto_q(qq, t=6.0), name + " failed"; r.report(name)
    if r.finger_gap() < 0.008: sys.exit("lost the bar")
r.snap("agentview", "/workspace/hang_a.png")
try:
    c.line(origin(seq[6][1], Rh), Rh, n=5, t=6.0)
except RuntimeError as e:
    print("lay:", e)
r.report("lay")
r.gripper(0.08)
assert c.line(origin(seq[7][1], Rh), Rh, n=3, t=3.0), "retreat failed"; r.report("out")
r.snap("agentview"); r.snap("frontview")
