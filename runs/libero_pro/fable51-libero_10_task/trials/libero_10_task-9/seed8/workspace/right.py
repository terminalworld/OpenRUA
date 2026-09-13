"""Stand the lying mug up: hook the rim's inner top edge with the closed fingertips (hand pointing +y,
pitched 45 deg) and sweep along an arc about the base's far bottom edge until the mug tips onto its base."""
import sys
from common import *
c = Ctl("right"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
scene_no_white(sc)
R45 = hand_R(np.pi / 2, ph=np.pi / 4)
z_h = R45[:, 2]
def origin(tip): return np.asarray(tip) - 0.1034 * z_h
X = -0.15            # mug axis x
Y_RIM, Z_AX = -0.048, 0.947
P = np.array([0.06, 0.90])           # pivot (y,z): base's far bottom edge
c0 = np.array([Y_RIM, Z_AX + 0.041]) - P   # rim inner top edge rel. pivot
def tip(th, dy=0.0):
    th = np.radians(th)
    Rm = np.array([[np.cos(th), np.sin(th)], [-np.sin(th), np.cos(th)]])   # rim end up & +y
    cth = P + Rm @ c0
    n = np.array([np.sin(th), np.cos(th)])
    E = cth - 0.005 * n                       # finger top edge at the rim plane, 5 mm inside
    T = E + np.array([0.0141, -0.0141]) + np.array([dy, 0])   # tip 1 cm along the finger inside the rim plane
    return np.array([X, T[0], T[1]])
T0 = tip(0)
pre = np.array([X, T0[1] - 0.03, 1.10])
down = np.array([X, T0[1] - 0.03, T0[2]])
seq = [("pre", pre), ("down", down), ("in", T0)] + [(f"th{th}", tip(th, dy=0.01 * th / 65)) for th in (15, 30, 45, 55, 65)]
q = r.arm_q()
for name, t in seq:
    qq = c.ik_valid(origin(t), R45, seed=q, tries=6); print(name, np.round(t, 3), None if qq is None else np.round(qq, 2)); q = qq or q
if dry: sys.exit()
r.report("start")
if r.finger_gap() > 0.01: r.gripper(0.0)
q_pre = c.ik_valid(origin(pre), R45, seed=r.arm_q(), tries=8)
assert c.goto_q(q_pre, t=5.0), "pre failed"
r.report("pre")
for name, t in seq[1:]:
    try:
        ok = c.line(origin(t), R45, n=3, t=3.0)
        if not ok: sys.exit(name + ": line failed")
    except RuntimeError as e:
        print(name, e)
    r.report(name)
p, _ = r.fk()
c.line(p + [0, 0, 0.10], R45, n=2, t=2.0); r.report("up")
for cam in ("agentview", "frontview", "birdview"):
    r.snap(cam)
