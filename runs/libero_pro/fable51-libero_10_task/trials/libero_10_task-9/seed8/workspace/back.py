import sys
from common import *
c = Ctl("back"); r, sc = c.r, c.sc
scene_no_white(sc)
r.report("start")
Rf = hand_R(np.pi / 2)
p, _ = r.fk()
# retreat straight back (-y) at current height, then settle at y=0.03
P1 = np.array([-0.047, 0.03, p[2] + 0.01])
assert c.line(P1, Rf, n=5, t=5.0, ignore=("carried_mug",)), "retreat failed"
r.report("retreated")
for cam in ("sideview", "frontview", "robot0_eye_in_hand", "agentview"):
    r.snap(cam)
print("gap", r.finger_gap())
