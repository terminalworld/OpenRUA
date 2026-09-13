"""Body-pinch the upside-down mug near its base (top), rotate so the handle points -x, carry to a free
spot, then lower while dragging so the mug lays down on its side with the rim end toward the robot."""
import sys
from common import *
c = Ctl("laydown"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
scene_no_white(sc)
cx, cy = float(sys.argv[1]), float(sys.argv[2])
def Rpin(ang):
    yh = np.array([np.cos(ang), np.sin(ang), 0]); zh = np.array([0, 0, -1.0]); xh = np.cross(yh, zh)
    return R_from_axes(xh, yh, zh)
A0, A1 = np.radians(-15), np.radians(-43)
R0, R1 = Rpin(A0), Rpin(A1)
def origin(tip): return np.asarray(tip) + [0, 0, 0.1034]
DST = np.array([-0.10, -0.10])
m = np.array([np.cos(A1 + np.pi / 2), np.sin(A1 + np.pi / 2)])   # (0.68,0.73): direction the pin end drifts; rim end goes -m
# (drift along m, tip z): rim touches the table at tip z ~0.985; drag first, then descend so the mug tilts rim-first toward -m
LAYS = [(0.03, 0.986), (0.045, 0.98), (0.055, 0.97), (0.065, 0.96), (0.075, 0.95), (0.08, 0.94)]
def lay(i):
    d, zt = LAYS[i]
    return np.r_[DST + m * d, zt]
lays = range(len(LAYS))
q = r.arm_q()
for name, t, R in [("above", np.r_[cx, cy, 1.06], R0), ("down", np.r_[cx, cy, 0.978], R0), ("rot", np.r_[cx, cy, 1.0], R1),
                   ("dst", np.r_[DST, 1.0], R1)] + [(f"lay{p}", lay(p), R1) for p in lays]:
    qq = c.ik_valid(origin(t), R, seed=q, tries=8); print(name, np.round(t, 3), None if qq is None else np.round(qq, 2)); q = qq or q
if dry: sys.exit()
r.report("start")
if r.finger_gap() < 0.07: r.gripper(0.08)
q0 = c.ik_valid(origin([cx, cy, 1.06]), R0, seed=r.arm_q(), tries=8)
assert c.goto_q(q0, t=5.0), "above failed"; r.report("above")
try:
    c.line(origin([cx, cy, 0.978]), R0, n=4, t=4.0)
except RuntimeError as e:
    print("down:", e)
r.report("down")
gap = r.gripper(0.0); print("gap after close", gap)
if gap < 0.05: sys.exit("grasp missed")
p, _ = r.fk()
assert c.line(p + [0, 0, 0.022], R0, n=2, t=2.0), "lift failed"; r.report("lift")
q1 = c.ik_valid(origin([cx, cy, 1.0]), R1, seed=r.arm_q(), tries=8)
assert q1 is not None and c.goto_q(q1, t=4.0), "rot failed"; r.report("rot")
assert c.line(origin(np.r_[DST, 1.0]), R1, n=4, t=4.0), "carry failed"; r.report("dst")
for p_ in lays:
    try:
        c.line(origin(lay(p_)), R1, n=2, t=2.5)
    except RuntimeError as e:
        print(f"lay{p_}:", e)
    r.report(f"lay{p_}")
r.gripper(0.08)
p, _ = r.fk()
assert c.line(p + [0, 0, 0.12], R1, n=2, t=2.0), "retreat failed"; r.report("up")
r.snap("agentview"); r.snap("frontview")
