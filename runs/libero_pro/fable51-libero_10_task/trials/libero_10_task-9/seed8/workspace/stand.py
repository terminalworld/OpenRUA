"""Lying mug (axis along y, rim +y, bar on top along y): pinch the bar from above (fingers close along x),
lift, rotate the hand about x by 90 deg -> hand points +y, mug hangs upright (base down, handle -y).
Carry into the microwave cavity, set down, release, retreat."""
import sys
from common import *
c = Ctl("stand"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
scene_no_white(sc)
bx, by = float(sys.argv[1]), float(sys.argv[2])          # bar centre (top view)
def Rx(th):
    th = np.radians(th); return np.array([[1, 0, 0], [0, np.cos(th), -np.sin(th)], [0, np.sin(th), np.cos(th)]])
Rb = R_from_axes(np.array([0, 1.0, 0]), np.array([1.0, 0, 0]), np.array([0, 0, -1.0]))   # vertical, close along x
R45, Rf = Rx(45) @ Rb, Rx(90) @ Rb                                                       # Rf: z=+y, x=+z, y=+x
def origin(tip, R): return np.asarray(tip) - 0.1034 * R[:, 2]
CX, CY = -0.03, 0.365          # target mug centre in the cavity
OFF = 0.066                    # mug axis is this far beyond the tips along z_hand once upright
Z_CARRY, Z_IN, Z_SET = 1.10, 1.012, 1.004
seq = [("above", np.r_[bx, by, 1.10], Rb), ("down", np.r_[bx, by, 1.013], Rb), ("lift", np.r_[bx, by, 1.15], Rb),
       ("rot45", np.r_[bx, by, 1.15], R45), ("rot90", np.r_[bx, by, 1.15], Rf),
       ("carry", np.r_[CX, 0.05, Z_CARRY], Rf), ("front", np.r_[CX, 0.15, Z_IN], Rf), ("in", np.r_[CX, CY - OFF, Z_IN], Rf),
       ("set", np.r_[CX, CY - OFF, Z_SET], Rf), ("back", np.r_[CX, 0.10, Z_IN], Rf), ("up", np.r_[CX, 0.05, 1.25], Rf)]
q = r.arm_q()
for name, t, R in seq:
    qq = c.ik_valid(origin(t, R), R, seed=q, tries=8); print(name, np.round(t, 3), None if qq is None else np.round(qq, 2)); q = qq or q
if dry: sys.exit()
S = {n: (t, R) for n, t, R in seq}
def go(name, t=5.0):
    tt, R = S[name]; qq = c.ik_valid(origin(tt, R), R, seed=r.arm_q(), tries=8)
    assert qq is not None and c.goto_q(qq, t=t), name + " failed"; r.report(name)
def line(name, n=4, t=4.0, tolerant=False):
    tt, R = S[name]
    try:
        ok = c.line(origin(tt, R), R, n=n, t=t)
        if not ok: sys.exit(name + ": line failed")
    except RuntimeError as e:
        if not tolerant: raise
        print(name, e)
    r.report(name)
r.report("start")
if r.finger_gap() < 0.07: r.gripper(0.08)
go("above"); line("down", tolerant=True)
gap = r.gripper(0.0); print("gap after close", gap)
if not (0.008 < gap < 0.03): sys.exit("bar grasp missed")
line("lift", t=5.0); go("rot45", 6.0); go("rot90", 6.0)
if r.finger_gap() < 0.008: sys.exit("lost the bar")
r.snap("agentview", "/workspace/hang_a.png")
go("carry", 6.0); line("front", n=4, t=5.0); line("in", n=5, t=6.0, tolerant=True); line("set", n=2, t=2.0, tolerant=True)
r.gripper(0.06)
line("back", n=4, t=4.0); line("up", n=3, t=3.0)
r.snap("agentview"); r.snap("frontview")
