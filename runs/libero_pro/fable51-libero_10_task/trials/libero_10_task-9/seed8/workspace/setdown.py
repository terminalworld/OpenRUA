"""Lay the horizontally hanging mug down on the table in front of the microwave and release it."""
import sys
from common import *
c = Ctl("setdown"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
scene_no_white(sc)
sc.attach("carried_mug", None, None, detach=True)  # mug pose unknown; move slowly, tolerate contacts
Rf = hand_R(np.pi / 2)
r.report("start")
X = -0.047
P1 = np.array([X, -0.08, 1.10])
P2 = np.array([X, -0.08, 1.035])   # mug underside ~0.915
P3 = np.array([X, -0.08, 1.015])   # expect contact with table before this
if dry:
    q = r.arm_q()
    for P in (P1, P2, P3):
        qq = c.ik_valid(P, Rf, seed=q, tries=6); print(np.round(P, 3), None if qq is None else np.round(qq, 2)); q = qq or q
    sys.exit()
assert c.line(P1, Rf, n=4, t=4.0), "move back failed"; r.report("P1")
assert c.line(P2, Rf, n=3, t=3.0), "lower failed"; r.report("P2")
try:
    c.line(P3, Rf, n=2, t=2.0)
except RuntimeError as e:
    print("P3:", e)
r.report("P3")
r.gripper(0.08)
p, _ = r.fk()
assert c.line(p + [0, 0, 0.10], Rf, n=3, t=3.0), "lift failed"
r.report("lifted")
for cam in ("agentview", "frontview", "sideview", "birdview"):
    r.snap(cam)
